#!/usr/bin/env python3
"""Diagnostic: for 0-proposal frames, re-run YOLO-World at a near-zero conf
floor (0.001) and inspect the RAW milk-carton confidence distribution.

Goal: distinguish "weakly suspected but filtered by the 0.05 floor"
(boxes reappear at 0.001 with conf 0.001-0.05) vs "essentially no response"
(still 0 boxes even at 0.001).

Targets two groups, taken from audit_cases.csv:
  - milk_present==yes AND decision==no-object(no-proposal)  [the FN we care about]
  - milk_present==no  AND decision==no-object(no-proposal)  [control: should be ~0]

Does NOT modify yolo_ism.py; reuses yolo_ism.resolve_frames for frame parity.
"""
import csv
import os
import sys
import collections

import torch

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)
import yolo_ism  # noqa: E402

FRAMES_DIR = {"only_milk": "outputs/yolo_test/only_Milk/frames"}
LOWCONF = 0.001
STRIDE = 10


def targets_by_bag():
    """bag -> {frame_idx: group} for 0-proposal frames."""
    out = collections.defaultdict(dict)
    path = os.path.join(REPO, "outputs", "yolo_ism_audit", "audit_cases.csv")
    for r in csv.DictReader(open(path)):
        if r["decision"] != "no-object(no-proposal)":
            continue
        grp = ("milk_present" if r["milk_present"] == "yes"
               else "no_milk" if r["milk_present"] == "no" else "uncertain")
        out[r["bag"]][int(r["frame_idx"])] = grp
    return out


def main():
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    from ultralytics import YOLOWorld
    yolo = YOLOWorld("yolov8m-worldv2.pt")
    yolo.set_classes(["milk carton"])

    tb = targets_by_bag()
    rows = []
    for bag, fmap in tb.items():
        fd = os.path.join(REPO, FRAMES_DIR[bag]) if bag in FRAMES_DIR else ""
        want = set(fmap.keys())
        for frame_idx, name, bgr in yolo_ism.resolve_frames(
                os.path.join("data", "ros2_bag", bag), fd, STRIDE, 0,
                "/camera/camera/color/image_raw"):
            if frame_idx not in want:
                continue
            res = yolo.predict(bgr, conf=LOWCONF, imgsz=640, verbose=False,
                               device=0 if device.startswith("cuda") else "cpu")
            confs = []
            if len(res) and res[0].boxes is not None and len(res[0].boxes) > 0:
                confs = sorted([float(c) for c in res[0].boxes.conf], reverse=True)
            rows.append((bag, frame_idx, fmap[frame_idx],
                         len(confs), confs[0] if confs else 0.0))

    out_csv = os.path.join(REPO, "outputs", "yolo_ism_audit", "diag_zero_proposal.csv")
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["bag", "frame_idx", "group", "n_boxes_at_0.001", "max_conf"])
        w.writerows(rows)

    # ---- distribution ----
    def bucket(c):
        if c < 0.001:
            return "still_zero(<0.001)"
        if c < 0.01:
            return "[0.001,0.01)"
        if c < 0.02:
            return "[0.01,0.02)"
        if c < 0.05:
            return "[0.02,0.05)"
        return ">=0.05(would_pass)"

    for grp in ["milk_present", "no_milk"]:
        sub = [r for r in rows if r[2] == grp]
        if not sub:
            continue
        b = collections.Counter(bucket(r[4]) for r in sub)
        order = ["still_zero(<0.001)", "[0.001,0.01)", "[0.01,0.02)",
                 "[0.02,0.05)", ">=0.05(would_pass)"]
        print(f"\n=== group={grp}  (n={len(sub)} zero-proposal frames) ===")
        for k in order:
            n = b.get(k, 0)
            print(f"  {k:22s}: {n:4d}  ({100*n/len(sub):4.1f}%)")
        nz = sum(1 for r in sub if r[3] > 0)
        print(f"  -> at conf=0.001, {nz}/{len(sub)} frames produce >=1 box "
              f"({100*nz/len(sub):.1f}%)")
    print(f"\nCSV: {out_csv}")


if __name__ == "__main__":
    main()
