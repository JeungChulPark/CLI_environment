#!/usr/bin/env python3
"""03_postprocess.py — turn raw engine output into the deliverable tree.

For each (bag, object):
  outputs_e2e/{bag}/{object}/frame_{idx}.png   annotated overlay (RGB | pose-render,
                                                with object name + score + status)
  outputs_e2e/{bag}/{object}/pose_results.csv  frame_idx,score,tx,ty,tz,qx,qy,qz,qw,status

PEM-skipped frames (ISM below threshold) get a CSV row with status=ism_below_thresh
and no image (no pose to overlay).

Run:
  conda run -n sam6d_ros_humble python tools/e2e_pipeline/03_postprocess.py --bags two_table_around
"""
import argparse
import csv
import glob
import json
import os
import sys

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(__file__))
import pipeline_lib as L  # noqa: E402

FRAME_ROOT = os.path.join(L.OUT, "_frames")
RAW_ROOT = os.path.join(L.OUT, "_raw")


def rot_to_quat(R):
    """3x3 rotation matrix -> (qx, qy, qz, qw)."""
    R = np.asarray(R, dtype=np.float64)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


def annotate(vis_path, obj, score, status, out_path):
    img = cv2.imread(vis_path)
    if img is None:
        return False
    label = f"{obj}  score={score:.3f}  [{status}]"
    cv2.rectangle(img, (0, 0), (img.shape[1], 26), (0, 0, 0), -1)
    cv2.putText(img, label, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imwrite(out_path, img)
    return True


def process_pair(bag, obj):
    frame_dir = os.path.join(FRAME_ROOT, bag)
    raw = os.path.join(RAW_ROOT, bag, obj)
    out = os.path.join(L.OUT, bag, obj)
    os.makedirs(out, exist_ok=True)
    stems = sorted(os.path.splitext(os.path.basename(p))[0]
                   for p in glob.glob(os.path.join(frame_dir, "rgb", "*.png")))
    rows = []
    n_ok = n_skip = n_img = 0
    for stem in stems:
        pem_json = os.path.join(raw, stem, "detection_pem.json")
        vis_pem = os.path.join(raw, "vis_pem", f"{stem}.png")
        best = None
        if os.path.isfile(pem_json):
            try:
                dets = json.load(open(pem_json))
                if dets:
                    best = max(dets, key=lambda d: d.get("score", -1))
            except Exception:
                best = None
        if best is not None:
            tx, ty, tz = best["t"]
            qx, qy, qz, qw = rot_to_quat(best["R"])
            score = float(best.get("score", 0.0))
            status = "ok"
            rows.append([stem, round(score, 4), round(tx, 2), round(ty, 2), round(tz, 2),
                         round(qx, 6), round(qy, 6), round(qz, 6), round(qw, 6), status])
            if os.path.isfile(vis_pem):
                if annotate(vis_pem, obj, score, status, os.path.join(out, f"frame_{stem}.png")):
                    n_img += 1
            n_ok += 1
        else:
            rows.append([stem, "", "", "", "", "", "", "", "", "ism_below_thresh"])
            n_skip += 1

    with open(os.path.join(out, "pose_results.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame_idx", "score", "tx", "ty", "tz", "qx", "qy", "qz", "qw", "status"])
        w.writerows(rows)
    return {"bag": bag, "obj": obj, "frames": len(stems), "ok": n_ok, "skip": n_skip, "imgs": n_img}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags", default="ALL")
    ap.add_argument("--objects", default="ALL")
    args = ap.parse_args()

    pf_path = os.path.join(L.OUT, "_logs", "preflight.json")
    pf = json.load(open(pf_path)) if os.path.isfile(pf_path) else None

    if args.bags in ("ALL", "PILOT"):
        bags = ([L.PILOT_BAG] if args.bags == "PILOT"
                else (pf["summary"]["bags_ok"] if pf else L.TARGET_BAGS))
    else:
        bags = [b.strip() for b in args.bags.split(",") if b.strip()]
    # only bags that actually have raw output
    bags = [b for b in bags if os.path.isdir(os.path.join(RAW_ROOT, b))]

    objects = (pf["summary"]["objects_ok"] if (pf and args.objects == "ALL")
               else (L.list_objects() if args.objects == "ALL"
                     else [o.strip() for o in args.objects.split(",")]))

    total = []
    for bag in bags:
        for obj in objects:
            if not os.path.isdir(os.path.join(RAW_ROOT, bag, obj)):
                continue
            r = process_pair(bag, obj)
            total.append(r)
            print(f"  {bag}/{obj}: frames={r['frames']} pose_ok={r['ok']} skip={r['skip']} imgs={r['imgs']}",
                  flush=True)
    print(f"[postprocess] {len(total)} (bag,object) pairs processed")


if __name__ == "__main__":
    main()
