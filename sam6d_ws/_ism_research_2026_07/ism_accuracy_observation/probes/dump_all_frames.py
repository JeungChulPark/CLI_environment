#!/usr/bin/env python3
"""dump_all_frames.py — SAM_* 6 데이터 전수 프레임 관찰 덤프 (READ-ONLY).

운영 모듈은 import 만 하고 수정하지 않는다. 판정 로직은 운영 baseline과 동일:
프롬프트별 YOLO 패스 → 객체별 score_threshold + top_k=3 → sem_top5 argmax 선택
→ MobileSAM → appe11(CLS-top1 view) → appe_gate.

관찰을 위해 baseline 을 바꾸지 않고 추가로 수집하는 것:
  (a) RAW YOLO 패스 (conf=0.001, iou=0.95) — "raw 에 후보가 없음(Y1)" 과
      "raw 엔 있는데 conf/NMS/top_k 에서 제거됨(Y2/Y3)" 을 라벨 없이 분리하기 위함
  (b) 운영 후보 박스 **전부** 에 MobileSAM 을 적용 (baseline 은 승자에만 적용)
      → 선택 실패(Y4)와 게이트 실패(Y6) 를 같은 프레임에서 함께 관측
  (c) 박스 × 전 클래스 점수 (Top-3 관찰용) — baseline 판정에는 쓰지 않음
  (d) HSV 히스토그램 원본 (masked / bbox / background, H32·S32·V32 + H16xS8)
      → bin 수·색공간·유사도 변형은 오프라인에서 재계산

산출:
  frame_dumps/<dataset>_boxes.csv     박스 단위
  frame_dumps/<dataset>_pairs.csv     박스×클래스 단위 (운영 점수 + HSV score)
  frame_dumps/<dataset>_yolo.csv      프레임×프롬프트 단위 raw/post/top_k 후보 통계
  hsv_features/<dataset>_hsv.npz      히스토그램 원본
  frame_dumps/crops/<uid>.png         라벨링용 crop (샘플만)
"""
import argparse, csv, os, sys
from pathlib import Path

import cv2
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(ROOT))  # sam6d_ws
sys.path.insert(0, REPO)
import yolo_ism as yi                    # noqa: E402
import yolo_ism_object_n as o_n          # noqa: E402

CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
COLOR_TOPIC = "/camera/camera/color/image_raw"
BLOCKS = [2, 9, 11]
IOUS = (0.3, 0.5, 0.7)


# ------------------------------------------------------------------ HSV utils
def hists(bgr, mask):
    """masked/unmasked HSV 히스토그램 묶음. mask=None 이면 전체."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = (mask.astype(np.uint8) * 255) if mask is not None else None
    h1 = cv2.calcHist([hsv], [0], m, [32], [0, 180]).flatten()
    s1 = cv2.calcHist([hsv], [1], m, [32], [0, 256]).flatten()
    v1 = cv2.calcHist([hsv], [2], m, [32], [0, 256]).flatten()
    hs = cv2.calcHist([hsv], [0, 1], m, [16, 8], [0, 180, 0, 256]).flatten()
    out = []
    for a in (h1, s1, v1, hs):
        s = a.sum()
        out.append((a / s if s > 0 else a).astype(np.float32))
    return np.concatenate(out)          # 32+32+32+128 = 224


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    u = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / u if u > 0 else 0.0


def group_boxes(boxes, thr):
    """IoU>thr 인 박스들을 union-find 로 묶어 proposal group id 부여."""
    n = len(boxes); par = list(range(n))
    def find(x):
        while par[x] != x:
            par[x] = par[par[x]]; x = par[x]
        return x
    for i in range(n):
        for j in range(i+1, n):
            if iou(boxes[i], boxes[j]) > thr:
                a, b = find(i), find(j)
                if a != b:
                    par[a] = b
    return [find(i) for i in range(n)]


def bag_frames(ds):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    ts = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(os.path.join(CONV, ds))], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == COLOR_TOPIC]
        i = -1
        for conn, t, raw in reader.messages(connections=conns):
            i += 1
            msg = reader.deserialize(raw, conn.msgtype)
            buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            img = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR) if msg.encoding.lower() == "rgb8" else buf.copy()
            yield i, t, img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--datasets", nargs="+", default=DATASETS)
    ap.add_argument("--stride", type=int, default=1, help="1 = 전수")
    ap.add_argument("--save-crop-every", type=int, default=1,
                    help="N 프레임마다 그 프레임의 crop 저장 (라벨링용)")
    a = ap.parse_args()

    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, False)
    seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    prompts, groups = o_n.build_prompt_groups(objs)
    img_sz = int(defaults.get("imgsz", 640))
    conf_min = min(float(o.get("score_threshold", 0.02)) for o in objs)

    for o in objs:
        o["_tcls"] = o["tcls"].to(device)
        o["_flat"] = {b: torch.cat(o["tappe_blocks"][b], 0).to(device) for b in BLOCKS}
        o["_seg"] = torch.cat([torch.full((t.shape[0],), i, dtype=torch.long)
                               for i, t in enumerate(o["tappe_blocks"][BLOCKS[0]])]).to(device)
        o["_nview"] = len(o["tappe_blocks"][BLOCKS[0]])

    # 템플릿 렌더 HSV prototype (렌더 도메인)
    tpl_h = {}
    for o in objs:
        hs = []
        for rp in yi._template_paths(o["template_dir"]):
            idx = os.path.basename(rp).split("_")[1].split(".")[0]
            mp = os.path.join(o["template_dir"], f"mask_{idx}.png")
            if not os.path.isfile(mp):
                continue
            hs.append(hists(cv2.imread(rp), cv2.imread(mp, cv2.IMREAD_GRAYSCALE) > 0))
        tpl_h[o["name"]] = np.stack(hs)
    np.savez_compressed(os.path.join(ROOT, "hsv_features", "template_render_hsv.npz"),
                        **{k: v for k, v in tpl_h.items()})

    from ultralytics import YOLOWorld
    yolos = []
    for p in prompts:
        y = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt")); y.set_classes([p]); yolos.append(y)

    cropdir = os.path.join(ROOT, "frame_dumps", "crops"); os.makedirs(cropdir, exist_ok=True)

    for ds in a.datasets:
        brows, prows, yrows, H = [], [], [], []
        nfr = 0
        for fi, tstamp, bgr in bag_frames(ds):
            if fi % a.stride:
                continue
            nfr += 1
            h, w = bgr.shape[:2]
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            norm = yi.normalize_rgb(rgb)

            pb, raw_stat = {}, {}
            for pk, y in enumerate(yolos):
                # RAW (관찰 전용): 사실상 필터 없음
                rr = y.predict(bgr, conf=0.001, iou=0.95, max_det=300, imgsz=img_sz,
                               verbose=False, device=device)
                nraw = len(rr[0].boxes) if (len(rr) and rr[0].boxes is not None) else 0
                rawb = []
                if nraw:
                    b = rr[0].boxes
                    for j in range(len(b)):
                        xy = b.xyxy[j].tolist()
                        rawb.append(([max(0, min(int(xy[0]), w-1)), max(0, min(int(xy[1]), h-1)),
                                      max(1, min(int(xy[2]), w)), max(1, min(int(xy[3]), h))],
                                     float(b.conf[j])))
                # 운영 패스
                r = y.predict(bgr, conf=conf_min, imgsz=img_sz, verbose=False, device=device)
                post = []
                if len(r) and r[0].boxes is not None and len(r[0].boxes):
                    b = r[0].boxes
                    for j in range(len(b)):
                        xy = b.xyxy[j].tolist()
                        x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
                        x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
                        post.append(((x1, y1, x2, y2), float(b.conf[j])))
                pb[pk] = sorted(post, key=lambda t: t[1], reverse=True)
                raw_stat[pk] = (nraw, rawb)

            # 프롬프트×객체 후보 통계
            cand_of = {}
            for pi, oh in groups.items():
                nraw, rawb = raw_stat[pi]
                cand = pb[pi]
                for o in oh:
                    thr = float(o.get("score_threshold", 0.0)); tk = int(o.get("top_k", 3))
                    sel = [(bx, sc) for bx, sc in cand if sc >= thr][:tk]
                    for rank, (bx, sc) in enumerate(sel, start=1):
                        cand_of.setdefault(bx, {})[o["name"]] = (sc, rank)
                    yrows.append({"dataset": ds, "frame_id": fi, "timestamp": tstamp,
                                  "object": o["name"], "prompt": o["yolo_prompt"],
                                  "raw_candidates": nraw,
                                  "raw_max_conf": round(max([c for _, c in rawb], default=0.0), 4),
                                  "post_nms_candidates": len(cand),
                                  "post_max_conf": round(max([c for _, c in cand], default=0.0), 4),
                                  "after_score_threshold": len([1 for _, sc in cand if sc >= thr]),
                                  "topk_candidates": len(sel),
                                  "score_threshold": thr, "top_k": tk})

            boxes = list(cand_of.keys())
            if not boxes:
                continue
            crops, keep = [], []
            for bx in boxes:
                c = yi.crop_resize_pad(norm, list(bx))
                if c is not None:
                    crops.append(c); keep.append(bx)
            if not crops:
                continue
            boxes = keep
            cls_all, patch_all = o_n.dinov2_blocks_forward(model, crops, device, BLOCKS)
            masks = yi.segment_boxes(seg, bgr, [list(b) for b in boxes], device)

            gid = {t: group_boxes([list(b) for b in boxes], t) for t in IOUS}

            for bi, bx in enumerate(boxes):
                x1, y1, x2, y2 = bx
                uid = f"{ds}|{fi}|{x1}_{y1}_{x2}_{y2}"
                mask = masks[bi]
                sub = bgr[y1:y2, x1:x2]
                msub = mask[y1:y2, x1:x2] if mask is not None else None
                marea = int(msub.sum()) if msub is not None else 0
                barea = (x2-x1)*(y2-y1)
                ncomp = int(cv2.connectedComponents(msub.astype(np.uint8))[0]-1) if msub is not None else 0
                hm = hists(sub, msub)
                hb = hists(sub, None)
                hbg = hists(sub, ~msub if msub is not None else None)
                H.append(np.concatenate([hm, hb, hbg]))

                qfg = {}
                for b in BLOCKS:
                    qp = patch_all[b][bi]
                    qfg[b] = (yi.masked_query_patches(qp.cpu(), mask, list(bx), pool)[0].to(device)
                              if mask is not None else qp)
                nq = qfg[11].shape[0]

                if a.save_crop_every and fi % a.save_crop_every == 0 and sub.size:
                    cv2.imwrite(os.path.join(cropdir, uid.replace("|", "__") + ".png"), sub)

                brows.append({"uid": uid, "dataset": ds, "frame_id": fi, "timestamp": tstamp,
                              "x1": x1, "y1": y1, "x2": x2, "y2": y2, "bbox_area": barea,
                              "aspect": round((x2-x1)/max(1, y2-y1), 3),
                              "mask_area": marea, "mask_bbox_ratio": round(marea/max(1, barea), 4),
                              "n_components": ncomp, "n_valid_patch": nq,
                              "group_iou30": gid[0.3][bi], "group_iou50": gid[0.5][bi],
                              "group_iou70": gid[0.7][bi],
                              "candidate_for": ";".join(sorted(cand_of[bx].keys()))})

                for o in objs:
                    nm = o["name"]
                    sims = (o["_tcls"] @ cls_all[bi].to(device)).cpu()
                    order = torch.argsort(sims, descending=True)
                    ss = sims[order]
                    t1 = int(order[0])
                    ap = {}
                    for b in BLOCKS:
                        if qfg[b].shape[0] == 0:
                            ap[b] = (0.0, 0.0); continue
                        sim = qfg[b] @ o["_flat"][b].T
                        per = torch.full((qfg[b].shape[0], o["_nview"]), -1.0,
                                         device=device, dtype=sim.dtype)
                        per.scatter_reduce_(1, o["_seg"].unsqueeze(0).expand(qfg[b].shape[0], -1),
                                            sim, reduce="amax")
                        pv = per.mean(0).clamp(0, 1).cpu()
                        ap[b] = (float(pv[t1]), float(pv.max()))
                    cf = cand_of[bx].get(nm)
                    # 렌더 템플릿 HSV 대비 intersection (masked / bbox) — 관찰용 기본 지표
                    T = tpl_h[nm]
                    ci_m = float(np.minimum(T[:, 96:], hm[96:][None]).sum(1).max())
                    ci_b = float(np.minimum(T[:, 96:], hb[96:][None]).sum(1).max())
                    prows.append({"uid": uid, "dataset": ds, "frame_id": fi, "object": nm,
                                  "is_candidate": int(cf is not None),
                                  "yolo_conf": round(cf[0], 4) if cf else 0.0,
                                  "yolo_rank": cf[1] if cf else 0,
                                  "sem_top5": round(float(ss[:5].mean()), 4),
                                  "sem_top1": round(float(ss[0]), 4),
                                  "sem_mean": round(float(sims.mean()), 4),
                                  "cls_top1_view": t1,
                                  "appe11_clstop1": round(ap[11][0], 4),
                                  "appe11_max42": round(ap[11][1], 4),
                                  "appe9_max42": round(ap[9][1], 4),
                                  "appe2_max42": round(ap[2][1], 4),
                                  "hsv_render_masked": round(ci_m, 4),
                                  "hsv_render_bbox": round(ci_b, 4),
                                  "sim_thr": o["similarity_threshold"], "appe_gate": o["appe_gate"]})
            if nfr % 50 == 0:
                print(f"  [{ds}] {nfr} frames, {len(brows)} boxes", flush=True)

        fd = os.path.join(ROOT, "frame_dumps")
        for name, rows in (("boxes", brows), ("pairs", prows), ("yolo", yrows)):
            if not rows:
                continue
            with open(os.path.join(fd, f"{ds}_{name}.csv"), "w", newline="") as f:
                wt = csv.DictWriter(f, fieldnames=list(rows[0].keys())); wt.writeheader(); wt.writerows(rows)
        if H:
            np.savez_compressed(os.path.join(ROOT, "hsv_features", f"{ds}_hsv.npz"),
                                uid=np.array([r["uid"] for r in brows]), hist=np.stack(H))
        print(f"[{ds}] frames={nfr} boxes={len(brows)} pairs={len(prows)} yolo_rows={len(yrows)}", flush=True)


if __name__ == "__main__":
    main()
