#!/usr/bin/env python3
"""05_montage.py — representative montages per (bag, object) and per bag.

Builds outputs_e2e/evaluation/montage/{bag}/{object}.png from up to N representative
annotated overlays (best/median/worst by score), and a {bag}/_overview.png contact
sheet of the best overlay per object.

Run:
  conda run -n sam6d_ros_humble python tools/e2e_pipeline/05_montage.py
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(__file__))
import pipeline_lib as L  # noqa: E402

EVAL_DIR = os.path.join(L.OUT, "evaluation")
MON_DIR = os.path.join(EVAL_DIR, "montage")
CELL_W = 640  # resize each overlay to this width for the grid


def load_resized(path, w=CELL_W):
    img = cv2.imread(path)
    if img is None:
        return None
    h = int(img.shape[0] * w / img.shape[1])
    return cv2.resize(img, (w, h))


def vstack_pad(imgs):
    if not imgs:
        return None
    w = max(i.shape[1] for i in imgs)
    padded = [cv2.copyMakeBorder(i, 0, 0, 0, w - i.shape[1], cv2.BORDER_CONSTANT, value=(20, 20, 20))
              for i in imgs]
    return np.vstack(padded)


def best_median_worst(rows):
    if not rows:
        return []
    rs = sorted(rows, key=lambda r: float(r["score"]))
    idx = sorted(set([len(rs) - 1, len(rs) // 2, 0]))  # best, median, worst
    return [rs[i] for i in idx]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--per-object", type=int, default=3)
    args = ap.parse_args()

    summary = os.path.join(EVAL_DIR, "pose_quality_summary.csv")
    by_pair = {}
    if os.path.isfile(summary):
        for r in csv.DictReader(open(summary)):
            sc = r["notes"].split("score=")[-1].split()[0] if "score=" in r["notes"] else "0"
            by_pair.setdefault((r["bag_name"], r["object_id"]), []).append({"frame_idx": r["frame_idx"],
                                                                            "score": sc,
                                                                            "img": r["result_image"]})
    n_made = 0
    bags = sorted(set(b for b, _ in by_pair))
    for bag in bags:
        os.makedirs(os.path.join(MON_DIR, bag), exist_ok=True)
        overview = []
        for (b, obj), rows in sorted(by_pair.items()):
            if b != bag:
                continue
            picks = best_median_worst(rows)[:args.per_object]
            cells = []
            for p in picks:
                ip = os.path.join(L.OUT, p["img"])
                im = load_resized(ip)
                if im is not None:
                    cells.append(im)
            grid = vstack_pad(cells)
            if grid is not None:
                cv2.putText(grid, obj, (8, grid.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 255), 2, cv2.LINE_AA)
                cv2.imwrite(os.path.join(MON_DIR, bag, f"{obj}.png"), grid)
                n_made += 1
                if cells:
                    overview.append(cv2.resize(cells[0], (CELL_W // 2, int(cells[0].shape[0] * (CELL_W // 2) / cells[0].shape[1]))))
        if overview:
            # tile overview 2-up rows
            rows_img = []
            for i in range(0, len(overview), 2):
                pair = overview[i:i + 2]
                if len(pair) == 2:
                    h = min(pair[0].shape[0], pair[1].shape[0])
                    pair = [cv2.resize(p, (p.shape[1], h)) for p in pair]
                    rows_img.append(np.hstack(pair))
                else:
                    rows_img.append(pair[0])
            ov = vstack_pad(rows_img)
            if ov is not None:
                cv2.imwrite(os.path.join(MON_DIR, bag, "_overview.png"), ov)
    print(f"[montage] wrote {n_made} object montages under {MON_DIR}")


if __name__ == "__main__":
    main()
