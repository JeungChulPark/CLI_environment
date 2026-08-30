#!/usr/bin/env python3
"""bench_frame_pipeline.py — end-to-end per-frame ISM latency, 3 variants.

READ-ONLY (imports production modules, monkeypatches only inside this process).

  P0  production PEM-bridge / ROS-node path:
        10 per-prompt YOLO passes  +  per-object o_n.recognize()
        (one DINOv2 forward per proposal, one MobileSAM call per object)
  P1  1 shared YOLO pass with multi_label NMS  +  o_n.recognize_frame()
        (one batched DINOv2 forward per frame, one MobileSAM call per frame)
Also records, per frame, the set of accepted object names for each variant so
decision drift between P0 and P1 is visible rather than assumed.
"""
import argparse, csv, glob, json, os, statistics, sys, time

import cv2
import torch

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))  # sam6d_ws
sys.path.insert(0, REPO)
import yolo_ism as yi                       # noqa: E402
import yolo_ism_object_n as o_n             # noqa: E402
from ultralytics import YOLOWorld           # noqa: E402
from ultralytics.utils import nms as ul_nms  # noqa: E402
import ultralytics.models.yolo.detect.predict as _dp  # noqa: E402

_ORIG = ul_nms.non_max_suppression
_MULTI = {"on": False}


def _patched(prediction, *a, **kw):
    if _MULTI["on"]:
        kw["multi_label"] = True
    return _ORIG(prediction, *a, **kw)


ul_nms.non_max_suppression = _patched
_dp.nms.non_max_suppression = _patched


def sync():
    if torch.cuda.is_available():
        torch.cuda.synchronize()


def stat(v):
    v = sorted(v)
    if not v:
        return {}
    return {"n": len(v), "median_ms": round(statistics.median(v), 2),
            "mean_ms": round(statistics.fmean(v), 2), "p90_ms": round(v[int(0.9*(len(v)-1))], 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame-globs", nargs="+", default=[
        os.path.join(REPO, "outputs/rgbd_imu_sdk_bag/SAM_loop1/input/frame_*/rgb.png"),
        os.path.join(REPO, "outputs/pem_inputs/sam_105314/frame_*/rgb.png")])
    ap.add_argument("--max-frames-per-glob", type=int, default=15)
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--out-dir", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results"))
    a = ap.parse_args()

    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, False)
    segmentor = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    prompts, groups = o_n.build_prompt_groups(objs)
    img_sz = int(defaults.get("imgsz", 640))
    conf = min(float(o.get("score_threshold", 0.02)) for o in objs)
    w = defaults.get("weights", "yolov8m-worldv2.pt")

    gpu_flat, gpu_seg = {}, {}
    for o in objs:
        tp = o["tappe"]
        gpu_flat[o["name"]] = torch.cat(tp, 0).to(device)
        gpu_seg[o["name"]] = torch.cat(
            [torch.full((t.shape[0],), i, dtype=torch.long) for i, t in enumerate(tp)]
        ).to(device).unsqueeze(0)

    per_prompt = []
    for p in prompts:
        y = YOLOWorld(w); y.set_classes([p]); per_prompt.append(y)
    shared = YOLOWorld(w); shared.set_classes(prompts)

    frames = []
    for g in a.frame_globs:
        frames += sorted(glob.glob(g))[: a.max_frames_per_glob]
    print(f"[bench] {len(frames)} frames, {len(objs)} objects, {len(prompts)} prompts, dev={device}")

    T = {k: [] for k in ("P0_total", "P0_yolo", "P0_recognize",
                         "P1_total", "P1_yolo", "P1_recognize")}
    counts = {"P0_dino_fwd": [], "P0_sam_calls": [], "P1_dino_fwd": [], "P1_sam_calls": []}
    rows = []

    for fi, fp in enumerate(frames):
        bgr = cv2.imread(fp)
        if bgr is None:
            continue
        warm = fi < a.warmup
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w_ = bgr.shape[:2]

        # ---------------- P0: production path ----------------
        sync(); t0 = time.perf_counter()
        pb0 = {i: [] for i in range(len(prompts))}
        for pk, y in enumerate(per_prompt):
            r = y.predict(bgr, conf=conf, imgsz=img_sz, verbose=False, device=device)
            if len(r) and r[0].boxes is not None and len(r[0].boxes):
                b = r[0].boxes
                for j in range(len(b)):
                    xy = b.xyxy[j].tolist()
                    x1 = max(0, min(int(xy[0]), w_-1)); y1 = max(0, min(int(xy[1]), h-1))
                    x2 = max(x1+1, min(int(xy[2]), w_)); y2 = max(y1+1, min(int(xy[3]), h))
                    pb0[pk].append(([x1, y1, x2, y2], float(b.conf[j])))
        sync(); d_y0 = (time.perf_counter()-t0)*1e3

        sync(); t0 = time.perf_counter()
        acc0, nf0, ns0 = [], 0, 0
        for pi, objs_here in groups.items():
            cand = sorted(pb0.get(pi, []), key=lambda t: t[1], reverse=True)
            for o in objs_here:
                sel = [(b, s) for b, s in cand if s >= float(o.get("score_threshold", 0.0))][
                    : int(o.get("top_k", 3))]
                nf0 += len(sel)
                r = o_n.recognize(o, cand, bgr, rgb, model, device, segmentor, pool)
                if r["box"] is not None:
                    ns0 += 1
                if r["accepted"]:
                    acc0.append(o["name"])
        sync(); d_r0 = (time.perf_counter()-t0)*1e3

        # ---------------- P1: shared multi_label YOLO + batched recognize_frame ----------------
        _MULTI["on"] = True
        sync(); t0 = time.perf_counter()
        res = shared.predict(bgr, conf=conf, imgsz=img_sz, verbose=False, device=device)
        pb1 = {i: [] for i in range(len(prompts))}
        if len(res) and res[0].boxes is not None and len(res[0].boxes):
            b = res[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w_-1)); y1 = max(0, min(int(xy[1]), h-1))
                x2 = max(x1+1, min(int(xy[2]), w_)); y2 = max(y1+1, min(int(xy[3]), h))
                ci = int(b.cls[j])
                if ci in pb1:
                    pb1[ci].append(([x1, y1, x2, y2], float(b.conf[j])))
        sync(); d_y1 = (time.perf_counter()-t0)*1e3
        _MULTI["on"] = False

        sync(); t0 = time.perf_counter()
        norm_full = yi.normalize_rgb(rgb)
        out1 = o_n.recognize_frame(groups, pb1, bgr, rgb, norm_full, model, device, segmentor, pool)
        sync(); d_r1 = (time.perf_counter()-t0)*1e3
        acc1 = [n for n, r in out1.items() if r["accepted"]]
        nf1 = sum(r["num_proposals"] for r in out1.values())
        ns1 = 1 if any(r["box"] is not None for r in out1.values()) else 0

        # (the all-42-template GPU appearance cost is measured per detection in
        #  probe_template_scope.py / bench_patch_sim.py, not re-timed here)

        if warm:
            continue
        T["P0_yolo"].append(d_y0); T["P0_recognize"].append(d_r0); T["P0_total"].append(d_y0+d_r0)
        T["P1_yolo"].append(d_y1); T["P1_recognize"].append(d_r1); T["P1_total"].append(d_y1+d_r1)
        counts["P0_dino_fwd"].append(nf0); counts["P0_sam_calls"].append(ns0)
        counts["P1_dino_fwd"].append(nf1); counts["P1_sam_calls"].append(ns1)
        rows.append({"frame": os.path.basename(os.path.dirname(fp)),
                     "P0_accepted": ";".join(sorted(acc0)), "P1_accepted": ";".join(sorted(acc1)),
                     "same": int(sorted(acc0) == sorted(acc1)),
                     "P0_ms": round(d_y0+d_r0, 2), "P1_ms": round(d_y1+d_r1, 2),
                     "P0_dino_fwd": nf0, "P1_dino_fwd": nf1,
                     "P0_sam_calls": ns0, "P1_sam_calls": ns1})

    summary = {k: stat(v) for k, v in T.items()}
    summary["counts_per_frame"] = {k: {"mean": round(statistics.fmean(v), 2),
                                       "median": statistics.median(v)} for k, v in counts.items()}
    summary["decision_same_frames"] = f"{sum(r['same'] for r in rows)}/{len(rows)}"
    os.makedirs(a.out_dir, exist_ok=True)
    json.dump(summary, open(os.path.join(a.out_dir, "bench_frame_pipeline.json"), "w"), indent=2)
    with open(os.path.join(a.out_dir, "bench_frame_pipeline_frames.csv"), "w", newline="") as f:
        wtr = csv.DictWriter(f, fieldnames=list(rows[0].keys())); wtr.writeheader(); wtr.writerows(rows)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
