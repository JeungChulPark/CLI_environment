#!/usr/bin/env python3
"""dump_candidates.py — 후보 BBox 전수 특징 덤프 (READ-ONLY probe).

운영 모듈(yolo_ism, yolo_ism_object_n)을 import만 하고 수정하지 않는다.
프레임마다:
  1) 프롬프트별 YOLO 패스(운영 build_ism_inputs_imu 방식)로 후보 생성
  2) 객체별 score_threshold + top_k=3 적용 → 후보 박스 합집합
  3) 합집합 박스 전부에 대해 DINOv2 배치 forward(블록 2/9/11) 1회
  4) 합집합 박스 전부에 대해 MobileSAM 1회 호출 (semantic 승자만이 아님 →
     E3(선택 실패)과 E7(appe 게이트 실패)을 분리해 관측하기 위함)
  5) (박스 × 전 객체) 쌍마다 semantic 변형 6종, appearance 변형(블록11/9/2,
     42-view 전체), color 변형(masked HSV/Lab/moments, bbox 전체 대조),
     mask 품질 지표를 계산해 pairs.csv 로 저장
  6) 라벨링용 crop PNG 저장

산출:
  features/boxes.csv   박스 단위 (프레임/좌표/mask/crop 경로)
  features/pairs.csv   박스×객체 단위 (모든 점수)
  crops/<uid>.png      라벨링용 crop
"""
import argparse, csv, glob, json, os, sys

import cv2
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                      # ism_accuracy_analysis
REPO = os.path.dirname(os.path.dirname(ROOT))     # sam6d_ws (_ism_research_2026_07 아래)
sys.path.insert(0, REPO)
import yolo_ism as yi                             # noqa: E402
import yolo_ism_object_n as o_n                   # noqa: E402

BLOCKS = [2, 9, 11]
HBINS, SBINS = 16, 8


# ---------------------------------------------------------------- color utils
def hsv_hist(bgr, mask):
    """masked HSV 2D histogram (H16×S8), L1 정규화."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = (mask.astype(np.uint8) * 255) if mask is not None else None
    h = cv2.calcHist([hsv], [0, 1], m, [HBINS, SBINS], [0, 180, 0, 256]).flatten()
    s = h.sum()
    return h / s if s > 0 else h


def lab_moments(bgr, mask):
    """masked Lab 채널별 mean/std/skew (9차원)."""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    sel = mask.astype(bool) if mask is not None else np.ones(bgr.shape[:2], bool)
    if sel.sum() < 10:
        return np.zeros(9, np.float32)
    out = []
    for c in range(3):
        v = lab[..., c][sel]
        mu, sd = v.mean(), v.std() + 1e-6
        out += [mu, sd, float(((v - mu) ** 3).mean() / sd ** 3)]
    return np.array(out, np.float32)


def hist_inter(a, b):
    return float(np.minimum(a, b).sum())


# ------------------------------------------------------- template color cache
def build_template_color(o, cache_path, rebuild=False):
    """템플릿 42장의 masked HSV hist + Lab moments."""
    if os.path.isfile(cache_path) and not rebuild:
        z = np.load(cache_path)
        return z["hsv"], z["mom"]
    hs, ms = [], []
    for rp in yi._template_paths(o["template_dir"]):
        idx = os.path.basename(rp).split("_")[1].split(".")[0]
        mp = os.path.join(o["template_dir"], f"mask_{idx}.png")
        if not os.path.isfile(mp):
            continue
        bgr = cv2.imread(rp)
        mk = cv2.imread(mp, cv2.IMREAD_GRAYSCALE) > 0
        hs.append(hsv_hist(bgr, mk)); ms.append(lab_moments(bgr, mk))
    hsv = np.stack(hs); mom = np.stack(ms)
    os.makedirs(os.path.dirname(cache_path), exist_ok=True)
    np.savez(cache_path, hsv=hsv, mom=mom)
    return hsv, mom


# ---------------------------------------------------------------------- probe
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sources", nargs="+", default=[
        f"SAM_loop1={REPO}/outputs/rgbd_imu_sdk_bag/SAM_loop1/input/frame_*/rgb.png",
        f"SAM_loop2={REPO}/outputs/rgbd_imu_sdk_bag/SAM_loop2/input/frame_*/rgb.png",
        f"SAM_occlusion={REPO}/outputs/rgbd_imu_sdk_bag/SAM_occlusion/input/frame_*/rgb.png",
        f"SAM_circle={REPO}/outputs/rgbd_imu_sdk_bag/SAM_circle/input/frame_*/rgb.png",
        f"sam_105314={REPO}/outputs/pem_inputs/sam_105314/frame_*/rgb.png",
        f"sam_110633={REPO}/outputs/pem_inputs/sam_110633/frame_*/rgb.png",
    ])
    ap.add_argument("--per-source", type=int, default=25, help="소스별 균등 샘플 프레임 수")
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "features"))
    ap.add_argument("--crop-dir", default=os.path.join(ROOT, "datasets", "crops"))
    ap.add_argument("--rebuild-color", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True); os.makedirs(a.crop_dir, exist_ok=True)

    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, False)
    segmentor = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    prompts, groups = o_n.build_prompt_groups(objs)
    img_sz = int(defaults.get("imgsz", 640))
    conf_min = min(float(o.get("score_threshold", 0.02)) for o in objs)

    # 객체별 GPU 상주 템플릿 캐시 (블록별 flat patch + segment id) + color prototype
    for o in objs:
        o["_tcls"] = o["tcls"].to(device)
        o["_flat"], o["_seg"] = {}, None
        for b in BLOCKS:
            tp = o["tappe_blocks"][b] if b in o["tappe_blocks"] else None
            if tp is None:
                raise SystemExit(f"{o['name']}: block {b} template cache 없음 — config 의 "
                                 f"appe_blocks/nms_rank_blocks 로 생성된 캐시를 확인하라")
            o["_flat"][b] = torch.cat(tp, 0).to(device)
            if o["_seg"] is None:
                o["_seg"] = torch.cat([torch.full((t.shape[0],), i, dtype=torch.long)
                                       for i, t in enumerate(tp)]).to(device)
        o["_nview"] = len(o["tappe_blocks"][BLOCKS[0]])
        o["_chsv"], o["_cmom"] = build_template_color(
            o, os.path.join(ROOT, "features", "tplcolor", f"{o['name']}.npz"), a.rebuild_color)

    from ultralytics import YOLOWorld
    yolos = []
    for p in prompts:
        y = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt")); y.set_classes([p]); yolos.append(y)

    frames = []
    for s in a.sources:
        tag, g = s.split("=", 1)
        fs = sorted(glob.glob(g))
        if not fs:
            print(f"[warn] no frames: {g}"); continue
        step = max(1, len(fs) // a.per_source)
        frames += [(tag, f) for f in fs[::step][: a.per_source]]
    print(f"[dump] {len(frames)} frames, {len(objs)} objects, {len(prompts)} prompts, dev={device}")

    box_rows, pair_rows = [], []
    for fi, (tag, fp) in enumerate(frames):
        bgr = cv2.imread(fp)
        if bgr is None:
            continue
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        norm_full = yi.normalize_rgb(rgb)
        fname = os.path.basename(os.path.dirname(fp))

        # ---- 1) 프롬프트별 YOLO 패스 ----
        pboxes = {i: [] for i in range(len(prompts))}
        for pk, y in enumerate(yolos):
            r = y.predict(bgr, conf=conf_min, imgsz=img_sz, verbose=False, device=device)
            if not (len(r) and r[0].boxes is not None and len(r[0].boxes)):
                continue
            b = r[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
                x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
                pboxes[pk].append(((x1, y1, x2, y2), float(b.conf[j])))

        # ---- 2) 객체별 후보(운영 규칙) → 박스 합집합 ----
        cand_of = {}          # box -> {obj_name: (conf, rank)}
        for pi, objs_here in groups.items():
            cand = sorted(pboxes.get(pi, []), key=lambda t: t[1], reverse=True)
            for o in objs_here:
                sel = [(bx, sc) for bx, sc in cand
                       if sc >= float(o.get("score_threshold", 0.0))][: int(o.get("top_k", 3))]
                for rank, (bx, sc) in enumerate(sel, start=1):
                    cand_of.setdefault(bx, {})[o["name"]] = (sc, rank)
        boxes = list(cand_of.keys())
        if not boxes:
            continue

        # ---- 3) DINOv2 배치 forward (블록 2/9/11) ----
        crops, keep = [], []
        for bx in boxes:
            c = yi.crop_resize_pad(norm_full, list(bx))
            if c is not None:
                crops.append(c); keep.append(bx)
        if not crops:
            continue
        boxes = keep
        cls_all, patch_all = o_n.dinov2_blocks_forward(model, crops, device, BLOCKS)

        # ---- 4) MobileSAM 1회 (모든 박스) ----
        masks = yi.segment_boxes(segmentor, bgr, [list(b) for b in boxes], device)

        for bi, bx in enumerate(boxes):
            x1, y1, x2, y2 = bx
            uid = f"{tag}__{fname}__{x1}_{y1}_{x2}_{y2}"
            mask = masks[bi]
            crop_bgr = bgr[y1:y2, x1:x2]
            cpath = os.path.join(a.crop_dir, uid + ".png")
            if not os.path.isfile(cpath) and crop_bgr.size:
                cv2.imwrite(cpath, crop_bgr)

            # mask 품질
            if mask is not None:
                sub = mask[y1:y2, x1:x2]
                marea = int(sub.sum())
                ncomp = int(cv2.connectedComponents(sub.astype(np.uint8))[0] - 1)
                big = 0
                if ncomp > 0:
                    n, lab = cv2.connectedComponents(sub.astype(np.uint8))
                    sizes = [(lab == k).sum() for k in range(1, n)]
                    big = max(sizes) / max(1, marea)
            else:
                marea, ncomp, big = 0, 0, 0.0
            barea = (x2-x1) * (y2-y1)

            # query patch (블록별) + mask fg 필터
            qfg, nq = {}, 0
            for b in BLOCKS:
                qp = patch_all[b][bi]
                if mask is not None:
                    fg = yi.masked_query_patches(qp.cpu(), mask, list(bx), pool)[0].to(device)
                else:
                    fg = qp
                qfg[b] = fg
                nq = fg.shape[0]

            # color (mask 내부 vs bbox 전체)
            q_hsv_m = hsv_hist(crop_bgr, mask[y1:y2, x1:x2] if mask is not None else None)
            q_hsv_b = hsv_hist(crop_bgr, None)
            q_mom_m = lab_moments(crop_bgr, mask[y1:y2, x1:x2] if mask is not None else None)

            box_rows.append({"uid": uid, "source": tag, "frame": fname, "img": fp,
                             "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                             "bbox_area": barea, "aspect": round((x2-x1)/max(1, y2-y1), 3),
                             "mask_area": marea, "mask_bbox_ratio": round(marea/max(1, barea), 4),
                             "n_components": ncomp, "largest_comp_frac": round(float(big), 4),
                             "n_valid_patch": nq, "crop": cpath,
                             "candidate_for": ";".join(sorted(cand_of[bx].keys()))})

            # ---- 5) 박스 × 전 객체 점수 ----
            for o in objs:
                nm = o["name"]
                sims = (o["_tcls"] @ cls_all[bi].to(device)).cpu()      # [42]
                order = torch.argsort(sims, descending=True)
                s_sorted = sims[order]
                sem = {"top1": float(s_sorted[0]),
                       "top3": float(s_sorted[:3].mean()),
                       "top5": float(s_sorted[:5].mean()),     # 운영 점수
                       "median": float(sims.median()),
                       "softmax": float((torch.softmax(sims / 0.05, 0) * sims).sum()),
                       "margin12": float(s_sorted[0] - s_sorted[1]),
                       "mean": float(sims.mean())}
                t_top1 = int(order[0])

                appe = {}
                for b in BLOCKS:
                    if qfg[b].shape[0] == 0:
                        appe[b] = {"cls_top1": 0.0, "max42": 0.0, "mean42": 0.0,
                                   "argmax": -1, "top5cls_max": 0.0}
                        continue
                    sim = qfg[b] @ o["_flat"][b].T                       # [Nq, Ntot]
                    per = torch.full((qfg[b].shape[0], o["_nview"]), -1.0,
                                     device=device, dtype=sim.dtype)
                    per.scatter_reduce_(1, o["_seg"].unsqueeze(0).expand(qfg[b].shape[0], -1),
                                        sim, reduce="amax")
                    pv = per.mean(0).clamp(0, 1).cpu()                   # [42] view별 appe
                    appe[b] = {"cls_top1": float(pv[t_top1]),
                               "max42": float(pv.max()), "mean42": float(pv.mean()),
                               "argmax": int(pv.argmax()),
                               "top5cls_max": float(pv[order[:5]].max())}

                c_int_m = max(hist_inter(q_hsv_m, t) for t in o["_chsv"])
                c_int_b = max(hist_inter(q_hsv_b, t) for t in o["_chsv"])
                dmom = np.linalg.norm(o["_cmom"] - q_mom_m[None], axis=1).min()

                cf = cand_of[bx].get(nm)
                pair_rows.append({
                    "uid": uid, "source": tag, "frame": fname, "object": nm,
                    "is_candidate": int(cf is not None),
                    "yolo_conf": round(cf[0], 4) if cf else 0.0,
                    "yolo_rank": cf[1] if cf else 0,
                    "sem_top1": round(sem["top1"], 4), "sem_top3": round(sem["top3"], 4),
                    "sem_top5": round(sem["top5"], 4), "sem_median": round(sem["median"], 4),
                    "sem_softmax": round(sem["softmax"], 4), "sem_mean": round(sem["mean"], 4),
                    "sem_margin12": round(sem["margin12"], 4), "cls_top1_view": t_top1,
                    "appe11_clstop1": round(appe[11]["cls_top1"], 4),
                    "appe11_max42": round(appe[11]["max42"], 4),
                    "appe11_mean42": round(appe[11]["mean42"], 4),
                    "appe11_argmax": appe[11]["argmax"],
                    "appe11_top5cls": round(appe[11]["top5cls_max"], 4),
                    "appe9_max42": round(appe[9]["max42"], 4),
                    "appe9_clstop1": round(appe[9]["cls_top1"], 4),
                    "appe2_max42": round(appe[2]["max42"], 4),
                    "appe2_clstop1": round(appe[2]["cls_top1"], 4),
                    "appe2_mean42": round(appe[2]["mean42"], 4),
                    "color_hsv_masked": round(c_int_m, 4),
                    "color_hsv_bbox": round(c_int_b, 4),
                    "color_labmom_dist": round(float(dmom), 3),
                    "sim_thr": o["similarity_threshold"], "appe_gate": o["appe_gate"],
                })
        if (fi + 1) % 10 == 0:
            print(f"  {fi+1}/{len(frames)} frames, {len(box_rows)} boxes, {len(pair_rows)} pairs")

    with open(os.path.join(a.out_dir, "boxes.csv"), "w", newline="") as f:
        wtr = csv.DictWriter(f, fieldnames=list(box_rows[0].keys())); wtr.writeheader(); wtr.writerows(box_rows)
    with open(os.path.join(a.out_dir, "pairs.csv"), "w", newline="") as f:
        wtr = csv.DictWriter(f, fieldnames=list(pair_rows[0].keys())); wtr.writeheader(); wtr.writerows(pair_rows)
    print(f"\n[done] boxes={len(box_rows)} pairs={len(pair_rows)} -> {a.out_dir}")


if __name__ == "__main__":
    main()
