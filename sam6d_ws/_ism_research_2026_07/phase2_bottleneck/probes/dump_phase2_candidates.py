#!/usr/bin/env python3
"""dump_phase2_candidates.py — Phase 2 후보 특징 증강 덤프 (READ-ONLY probe).

운영 모듈(yolo_ism, yolo_ism_object_n, ism_hsv)을 import만 하고 수정하지 않는다.
Phase 1C의 정본 후보집합(yolo_localization_research/pipeline/cur/{ds}_pairs.csv,
18060행=1806박스×10객체, 270프레임)을 그대로 사용하고, 각 박스 bbox를 gt_input
프레임에서 재생하여 다음을 **단일 DINOv2 forward(n=[2,9,11])** 로 증강한다:

  Workstream A:  appe block 2 / 9 / 11 (clstop1·max42·mean42)   — 동일 forward, 재forward 0
  Workstream B:  raw 42-view semantic → top1/3/5/all-mean/median/product/geomean
  통합용:        HSV 게이트 점수(Phase 1C 권위 ism_hsv, Dinosaur +9 캐시 반영)

내장 parity 검증: 재계산 sem_top5 ≈ cur sem_top5, appe11_clstop1 ≈ cur appe11_clstop1.
GT visibility(user_reviewed==yes) 조인으로 gt_visible 부여.

산출:
  csv/appearance_block_scores.csv   박스×객체: appe2/9/11 변형 + gt_visible + parity
  csv/semantic_raw_scores.csv       박스×객체: sem 집계 7종 + gt_visible + parity
  csv/phase2_candidates.csv         통합 기저표(yolo_conf/routed/gates/hsv/gt_visible 포함)
  raw/view_sims_phase2.npz          uid|object -> float32[42] 원본 뷰 코사인
  metrics/dump_provenance.json      forward 카운트/parity 통계
"""
import argparse, csv, glob, json, os, sys
import cv2
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
PH2 = os.path.dirname(HERE)                        # phase2_bottleneck
RSRCH = os.path.dirname(PH2)                       # _ism_research_2026_07
REPO = os.path.dirname(RSRCH)                      # sam6d_ws
sys.path.insert(0, REPO)
import yolo_ism as yi                              # noqa: E402
import yolo_ism_object_n as o_n                    # noqa: E402
import ism_hsv                                     # noqa: E402

BLOCKS = [2, 9, 11]
DATASETS = ["sam_105018", "sam_105314", "sam_105652",
            "sam_110104", "sam_110532", "sam_110633"]
CUR = os.path.join(RSRCH, "yolo_localization_research", "pipeline", "cur")
FRAMES = os.path.join(RSRCH, "gt_input", "frames")
GT_CSV = os.path.join(RSRCH, "gt_input", "user_visibility_gt.csv")
OUT = os.path.join(REPO, "outputs", "phase2_appearance_semantic_yolo_bbox")


def load_gt():
    """(ds, frame_id:int) -> set(visible object names), user_reviewed==yes only."""
    gt = {}
    with open(GT_CSV) as f:
        for r in csv.DictReader(f):
            if r.get("user_reviewed", "").strip() != "yes":
                continue
            vo = (r.get("visible_objects") or "").replace(",", ";")
            names = {x.strip() for x in vo.split(";") if x.strip() and x.strip() != "none"}
            gt[(r["dataset_name"], int(r["frame_id"]))] = names
    return gt


def parse_bbox(uid):
    """'sam_105018|0|244_223_403_427' -> (x1,y1,x2,y2)."""
    b = uid.rsplit("|", 1)[1]
    return tuple(int(v) for v in b.split("_"))


def sem_aggregations(sims_sorted, sims_all):
    top1 = float(sims_sorted[0])
    top3 = float(sims_sorted[:3].mean())
    top5 = float(sims_sorted[:5].mean())          # 운영 semantic
    allm = float(sims_all.mean())
    med = float(sims_all.median())
    prod = top5 * allm                            # S5: Top-5 × 전체평균
    # geometric mean은 부호가 유효할 때만(코사인 음수 가능 → 클램프 후)
    geo = float(np.sqrt(max(0.0, top5) * max(0.0, allm)))  # S6
    return {"sem_top1": top1, "sem_top3": top3, "sem_top5": top5,
            "sem_all_mean": allm, "sem_median": med,
            "sem_product": prod, "sem_geomean": geo}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--datasets", nargs="+", default=DATASETS)
    ap.add_argument("--max-frames", type=int, default=0, help="ds별 상한(0=전체) — smoke용")
    a = ap.parse_args()

    gt = load_gt()
    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, False)
    segmentor = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    by_name = {o["name"]: o for o in objs}

    # 객체별 GPU 상주 템플릿(블록별 flat patch + view segment id) + HSV proto
    for o in objs:
        o["_tcls"] = o["tcls"].to(device)
        o["_flat"], o["_seg"] = {}, None
        for b in BLOCKS:
            tp = o["tappe_blocks"].get(b)
            if tp is None:
                raise SystemExit(f"{o['name']}: block {b} 템플릿 캐시 없음")
            o["_flat"][b] = torch.cat(tp, 0).to(device)
            if o["_seg"] is None:
                o["_seg"] = torch.cat([torch.full((t.shape[0],), i, dtype=torch.long)
                                       for i, t in enumerate(tp)]).to(device)
        o["_nview"] = len(o["tappe_blocks"][BLOCKS[0]])
        # HSV proto (Phase 1C 권위; Dinosaur +9 는 캐시에 baked)
        hp = os.path.join(os.path.dirname(o["cls_cache"]), f"{o['name']}_hsv.npz")
        proto, meta = ism_hsv.load_cache(hp, o["template_dir"])
        o["_hsv_proto"] = proto
        o["hsv_gate_threshold"] = float(o.get("hsv_gate_threshold", 0.1214))

    fwd_calls = 0                                  # 단일 forward 증명용
    appe_rows, sem_rows, cand_rows = [], [], []
    view_store = {}
    parity_sem, parity_appe = [], []

    for ds in a.datasets:
        pairs = os.path.join(CUR, f"{ds}_pairs.csv")
        # (frame_id) -> {uid -> {obj -> cur row}}
        byframe = {}
        with open(pairs) as f:
            for r in csv.DictReader(f):
                fid = int(r["frame_id"])
                byframe.setdefault(fid, {}).setdefault(r["uid"], {})[r["object"]] = r
        fids = sorted(byframe)
        if a.max_frames:
            fids = fids[:a.max_frames]
        for fi, fid in enumerate(fids):
            fpath = os.path.join(FRAMES, ds, f"frame_{fid:06d}.png")
            bgr = cv2.imread(fpath)
            if bgr is None:
                print(f"[warn] no frame {fpath}"); continue
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            norm_full = yi.normalize_rgb(rgb)
            vis = gt.get((ds, fid))               # None if frame not GT-reviewed

            uids = list(byframe[fid].keys())
            crops, keep = [], []
            for uid in uids:
                bx = parse_bbox(uid)
                c = yi.crop_resize_pad(norm_full, list(bx))
                if c is not None:
                    crops.append(c); keep.append(uid)
            if not crops:
                continue
            cls_all, patch_all = o_n.dinov2_blocks_forward(model, crops, device, BLOCKS)
            fwd_calls += 1
            masks = yi.segment_boxes(segmentor, bgr, [list(parse_bbox(u)) for u in keep], device)

            for bi, uid in enumerate(keep):
                bx = parse_bbox(uid); x1, y1, x2, y2 = bx
                mask = masks[bi]
                crop_bgr = bgr[y1:y2, x1:x2]
                mask_crop = mask[y1:y2, x1:x2] if mask is not None else None
                # query fg patches per block (블록별, mask fg 필터) — 동일 forward 결과 사용
                qfg = {}
                for b in BLOCKS:
                    qp = patch_all[b][bi]
                    if mask is not None:
                        fg = yi.masked_query_patches(qp.cpu(), mask, list(bx), pool)[0].to(device)
                    else:
                        fg = qp
                    qfg[b] = fg

                for obj, row in byframe[fid][uid].items():
                    o = by_name[obj]
                    sims = (o["_tcls"] @ cls_all[bi].to(device)).cpu()        # [42]
                    order = torch.argsort(sims, descending=True)
                    t_top1 = int(order[0])
                    sa = sem_aggregations(sims[order], sims)
                    view_store[f"{uid}||{obj}"] = sims.numpy().astype(np.float32)

                    appe = {}
                    for b in BLOCKS:
                        if qfg[b].shape[0] == 0:
                            appe[b] = (0.0, 0.0, 0.0); continue
                        sim = qfg[b] @ o["_flat"][b].T
                        per = torch.full((qfg[b].shape[0], o["_nview"]), -1.0,
                                         device=device, dtype=sim.dtype)
                        per.scatter_reduce_(1, o["_seg"].unsqueeze(0).expand(qfg[b].shape[0], -1),
                                            sim, reduce="amax")
                        pv = per.mean(0).clamp(0, 1).cpu()
                        appe[b] = (float(pv[t_top1]), float(pv.max()), float(pv.mean()))

                    # HSV (Phase 1C 권위) — fail-open
                    if o["_hsv_proto"] is not None and mask_crop is not None and mask_crop.sum() >= 1:
                        try:
                            hsv_score = float(ism_hsv.similarity(
                                ism_hsv.query_hist(crop_bgr, mask_crop.astype(np.uint8)),
                                o["_hsv_proto"]))
                        except Exception:
                            hsv_score = 1.0
                    else:
                        hsv_score = 1.0

                    gv = ("" if vis is None else int(obj in vis))
                    yolo_conf = float(row["yolo_conf"]); routed = int(row["routed"])
                    sim_thr = float(row["sim_thr"]); appe_gate = float(row["appe_gate"])
                    # parity vs cur
                    cur_top5 = float(row["sem_top5"]); cur_a11 = float(row["appe11_clstop1"])
                    parity_sem.append(abs(sa["sem_top5"] - cur_top5))
                    parity_appe.append(abs(appe[11][0] - cur_a11))

                    base = {"uid": uid, "dataset": ds, "frame_id": fid, "object": obj,
                            "yolo_conf": round(yolo_conf, 4), "routed": routed,
                            "bbox": f"{x1}_{y1}_{x2}_{y2}", "gt_visible": gv}
                    appe_rows.append({**base,
                        "appe2_clstop1": round(appe[2][0], 4), "appe9_clstop1": round(appe[9][0], 4),
                        "appe11_clstop1": round(appe[11][0], 4),
                        "appe2_max42": round(appe[2][1], 4), "appe9_max42": round(appe[9][1], 4),
                        "appe11_max42": round(appe[11][1], 4),
                        "appe2_mean42": round(appe[2][2], 4), "appe9_mean42": round(appe[9][2], 4),
                        "appe11_mean42": round(appe[11][2], 4),
                        "appe11_clstop1_cur": round(cur_a11, 4),
                        "sim_thr": sim_thr, "appe_gate": appe_gate})
                    sem_rows.append({**base, "n_views": int(sims.numel()),
                        **{k: round(v, 4) for k, v in sa.items()},
                        "sem_top5_cur": round(cur_top5, 4), "sim_thr": sim_thr})
                    cand_rows.append({**base,
                        **{k: round(v, 4) for k, v in sa.items()},
                        "appe2_clstop1": round(appe[2][0], 4), "appe9_clstop1": round(appe[9][0], 4),
                        "appe11_clstop1": round(appe[11][0], 4),
                        "hsv_score": round(hsv_score, 4),
                        "hsv_threshold": round(o["hsv_gate_threshold"], 5),
                        "sim_thr": sim_thr, "appe_gate": appe_gate,
                        "hue_correction_ocv": int(o.get("hsv_hue_correction_ocv", 0))})
        print(f"[{ds}] frames={len(fids)} appe_rows={len(appe_rows)}")

    os.makedirs(os.path.join(OUT, "csv"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "raw"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "metrics"), exist_ok=True)

    def dump(rows, name):
        p = os.path.join(OUT, "csv", name)
        with open(p, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
        return p, len(rows)

    dump(appe_rows, "appearance_block_scores.csv")
    dump(sem_rows, "semantic_raw_scores.csv")
    dump(cand_rows, "phase2_candidates.csv")
    np.savez_compressed(os.path.join(OUT, "raw", "view_sims_phase2.npz"), **view_store)

    prov = {"forward_calls": fwd_calls, "frames_processed": fwd_calls,
            "rows": len(appe_rows), "distinct_view_arrays": len(view_store),
            "blocks_per_forward": BLOCKS,
            "parity_sem_top5_maxabs": float(np.max(parity_sem)) if parity_sem else None,
            "parity_sem_top5_meanabs": float(np.mean(parity_sem)) if parity_sem else None,
            "parity_appe11_maxabs": float(np.max(parity_appe)) if parity_appe else None,
            "parity_appe11_meanabs": float(np.mean(parity_appe)) if parity_appe else None,
            "note": "forward_calls == frames_processed → 프레임당 단일 forward로 block2/9/11 동시 추출"}
    with open(os.path.join(OUT, "metrics", "dump_provenance.json"), "w") as f:
        json.dump(prov, f, indent=2, ensure_ascii=False)
    print(json.dumps(prov, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
