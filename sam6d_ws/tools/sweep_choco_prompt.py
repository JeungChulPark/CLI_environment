#!/usr/bin/env python3
"""sweep_choco_prompt.py — choco color+shape prompt sweep (cross-talk aware).

Resumes the multi-object ISM handoff next-action: choco is the only object that
fails to generalize (0%/2% on a couple of bags) because its prompt `carton` is
shape-only and loses boxes to milk's `milk carton`. This sweeps candidate
choco prompts on the weak bags and measures, per (candidate, bag):

  choco%  = frames where choco_high OR choco_low passed the ISM gate
  milk%   = frames where milk passed  (cross-talk guard: must NOT drop)

Only {milk, choco_high, choco_low} are loaded (milk co-present so its
`milk carton` competes for boxes exactly as in the full run). DINOv2 +
MobileSAM are built once and reused; a FRESH YOLOWorld instance is created per
candidate prompt (CLIP text-encoder device conflict on reuse — per handoff).

Read-only: imports helpers from yolo_ism / yolo_ism_object_n, no source edits,
writes nothing except an optional CSV.
"""
import argparse
import csv
import os
import sys

import cv2
import torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

import yolo_ism as yi
import yolo_ism_object_n as o_n

WEAK_BAGS = ["two_table_goback", "two_table_around_goback", "two_table_diagonal2"]
DEFAULT_CANDIDATES = ["carton", "brown carton", "brown box",
                      "chocolate box", "brown package"]
KEEP = {"milk", "choco_hazelnut_high", "choco_hazelnut_low"}


def parse_args():
    p = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--config", default=o_n.DEFAULT_CONFIG)
    p.add_argument("--bags", nargs="+", default=WEAK_BAGS)
    p.add_argument("--candidates", nargs="+", default=DEFAULT_CANDIDATES)
    p.add_argument("--stride", type=int, default=20, help="frame subsample for the sweep")
    p.add_argument("--max-frames", type=int, default=0)
    p.add_argument("--device", default="")
    p.add_argument("--out-csv", default=os.path.join(
        REPO_ROOT, "outputs", "_logs", "choco_prompt_sweep.csv"))
    return p.parse_args()


def run_bag(bag, frames_dir, stride, max_frames, objs, groups, unique_prompts,
            yolo, min_score, model, device, segmentor, pool, topic):
    """Return (n_frames, {name: accepted_count}, choco_any_count, milk_count)."""
    counts = {o["name"]: 0 for o in objs}
    n = 0
    for frame_idx, _name, bgr in yi.resolve_frames(
            os.path.join(REPO_ROOT, "data", "ros2_bag", bag), frames_dir,
            max(1, stride), max_frames, topic):
        n += 1
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res = yolo.predict(bgr, conf=min_score, imgsz=640, verbose=False, device=device)
        prompt_boxes = {i: [] for i in range(len(unique_prompts))}
        if len(res) and res[0].boxes is not None and len(res[0].boxes) > 0:
            b = res[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                cls_i = int(b.cls[j]) if b.cls is not None else 0
                if cls_i in prompt_boxes:
                    prompt_boxes[cls_i].append(([x1, y1, x2, y2], float(b.conf[j])))
        for pi, objs_here in groups.items():
            cand = sorted(prompt_boxes.get(pi, []), key=lambda t: t[1], reverse=True)
            for o in objs_here:
                r = o_n.recognize(o, cand, bgr, rgb, model, device, segmentor, pool)
                if r["accepted"]:
                    counts[o["name"]] += 1
    return n, counts


def main():
    args = parse_args()
    device = args.device or "cuda:0"
    device = device if torch.cuda.is_available() else "cpu"

    defaults, all_objs = o_n.load_config(args.config)
    objs = [o for o in all_objs if o["name"] in KEEP]
    if len(objs) != len(KEEP):
        raise SystemExit(f"[sweep] expected {KEEP}, got {[o['name'] for o in objs]}")

    print(f"[dinov2] loading (once)")
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, rebuild=False)
    seg_w = o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt"))
    print(f"[seg] loading {seg_w} (once)")
    segmentor = yi.build_segmentor(seg_w, device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    min_score = min(float(o.get("score_threshold", 0.02)) for o in objs)
    topic = "/camera/camera/color/image_raw"

    from ultralytics import YOLOWorld
    weights = defaults.get("weights", "yolov8m-worldv2.pt")

    rows = []  # (candidate, bag, n, choco_pct, milk_pct, choco_n, milk_n)
    for cand_prompt in args.candidates:
        # rewrite choco objects' prompt in-memory; milk stays "milk carton"
        for o in objs:
            if o["name"].startswith("choco"):
                o["yolo_prompt"] = cand_prompt
        unique_prompts, groups = o_n.build_prompt_groups(objs)
        try:
            yolo = YOLOWorld(weights)
        except Exception as e:
            print(f"[yolo] {weights} failed ({e}); fallback s")
            yolo = YOLOWorld("yolov8s-worldv2.pt")
        yolo.set_classes(unique_prompts)
        print(f"\n##### choco prompt = '{cand_prompt}'  (classes={unique_prompts}) #####")
        for bag in args.bags:
            frames_dir = os.path.join(REPO_ROOT, "outputs", "yolo_test", bag, "frames")
            n, counts = run_bag(bag, frames_dir, args.stride, args.max_frames, objs,
                                 groups, unique_prompts, yolo, min_score, model,
                                 device, segmentor, pool, topic)
            choco_n = counts["choco_hazelnut_high"] + counts["choco_hazelnut_low"]
            milk_n = counts["milk"]
            cp = 100.0 * choco_n / n if n else 0.0
            mp = 100.0 * milk_n / n if n else 0.0
            rows.append((cand_prompt, bag, n, cp, mp, choco_n, milk_n))
            print(f"  {bag:26s} frames={n:4d}  choco={cp:5.1f}% ({choco_n})  "
                  f"milk={mp:5.1f}% ({milk_n})")
        del yolo
        torch.cuda.empty_cache()

    print("\n===== SWEEP TABLE (choco% / milk%) =====")
    bags = args.bags
    hdr = f"{'candidate':16s} | " + " | ".join(f"{b[:18]:>18s}" for b in bags)
    print(hdr); print("-" * len(hdr))
    by = {}
    for c, b, n, cp, mp, cn, mn in rows:
        by[(c, b)] = (cp, mp)
    for c in args.candidates:
        cells = []
        for b in bags:
            cp, mp = by.get((c, b), (0, 0))
            cells.append(f"{cp:5.1f}/{mp:5.1f}")
        print(f"{c:16s} | " + " | ".join(f"{x:>18s}" for x in cells))

    os.makedirs(os.path.dirname(args.out_csv), exist_ok=True)
    with open(args.out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["candidate", "bag", "frames", "choco_pct", "milk_pct",
                    "choco_n", "milk_n"])
        for r in rows:
            w.writerow([r[0], r[1], r[2], round(r[3], 2), round(r[4], 2), r[5], r[6]])
    print(f"\n[csv] {args.out_csv}")


if __name__ == "__main__":
    main()
