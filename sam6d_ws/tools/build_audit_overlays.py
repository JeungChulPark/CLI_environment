#!/usr/bin/env python3
"""Companion to yolo_ism.py for the pseudo-GT audit (does NOT touch yolo_ism logic).

yolo_ism.py only records / draws the SINGLE selected proposal. To audit
proposal_recall ("is Milk inside ANY of the up-to-3 YOLO proposals") we re-run
ONLY YOLO-World (same weights/prompt/conf/imgsz/top-k as yolo_ism) on the exact
same frames (via yolo_ism.resolve_frames) and emit, per bag:

  outputs/yolo_ism/<bag>/proposals.csv          all proposal boxes + scores
  outputs/yolo_ism/<bag>/audit_overlay/<name>.jpg
        composite: every YOLO proposal (yellow, ranked #r score) +
        yolo_ism-selected box (thick green) when decision==milk + decision text

The composite overlay is the single image a VLM auditor inspects per frame:
it shows the raw scene, all candidate boxes (proposal recall) and what the
system decided (selected box + decision).
"""
import argparse
import csv
import os
import sys

import cv2
import numpy as np
import torch

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)
import yolo_ism  # noqa: E402  (reuse resolve_frames for exact frame parity)

OUT_ROOT = os.path.join(REPO, "outputs", "yolo_ism")

BAGS = [
    ("high_texture_around", ""),
    ("high_texture_far_close", ""),
    ("two_table_around", ""),
    ("two_table_around_goback", ""),
    ("two_table_diagonal1", ""),
    ("two_table_diagonal2", ""),
    ("two_table_goback", ""),
    ("only_milk", "outputs/yolo_test/only_Milk/frames"),
    ("milk_nomilk_bag", ""),
]
STRIDE = 10
PROMPT = "milk carton"
WEIGHTS = "yolov8m-worldv2.pt"
CONF = 0.05
TOPK = 3


def load_selected(csv_path):
    """frame_idx -> (decision, selected_box or None) from yolo_ism_results.csv."""
    sel = {}
    if not os.path.isfile(csv_path):
        return sel
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            box = None
            s = row.get("selected_bbox_xyxy", "").strip()
            if s:
                try:
                    box = [int(float(v)) for v in s.split(";")]
                except Exception:
                    box = None
            sel[int(row["frame_idx"])] = (row.get("decision", ""), box)
    return sel


def yolo_proposals(yolo, bgr, device):
    h, w = bgr.shape[:2]
    res = yolo.predict(bgr, conf=CONF, imgsz=640, verbose=False,
                       device=0 if device.startswith("cuda") else "cpu")
    boxes, scores = [], []
    if len(res) and res[0].boxes is not None and len(res[0].boxes) > 0:
        b = res[0].boxes
        for j in torch.argsort(b.conf, descending=True)[:TOPK]:
            xy = b.xyxy[j].tolist()
            x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
            x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
            boxes.append([x1, y1, x2, y2]); scores.append(float(b.conf[j]))
    return boxes, scores


def iou(a, b):
    if a is None or b is None:
        return 0.0
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def draw(bgr, boxes, scores, sel_box, decision):
    out = bgr.copy()
    sel_idx = -1
    if sel_box is not None:
        ious = [iou(b, sel_box) for b in boxes]
        if ious and max(ious) > 0.3:
            sel_idx = int(np.argmax(ious))
    for r, (box, sc) in enumerate(zip(boxes, scores), start=1):
        x1, y1, x2, y2 = box
        col = (0, 215, 255)  # yellow-ish for candidate proposals
        cv2.rectangle(out, (x1, y1), (x2, y2), col, 2)
        cv2.putText(out, f"#{r} {sc:.3f}", (x1 + 2, min(out.shape[0]-4, y2 + 16)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
    if sel_idx >= 0:
        x1, y1, x2, y2 = boxes[sel_idx]
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 220, 0), 3)  # selected = green
        cv2.putText(out, "SELECTED", (x1 + 2, max(12, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 2, cv2.LINE_AA)
    txt = f"decision={decision or 'n/a'}  proposals={len(boxes)}"
    cv2.rectangle(out, (0, 0), (min(out.shape[1], 470), 22), (0, 0, 0), -1)
    cv2.putText(out, txt, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 1, cv2.LINE_AA)
    return out


def main():
    global CONF
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="", help="comma list of bag names (default all)")
    ap.add_argument("--out-root", default=OUT_ROOT)
    ap.add_argument("--conf", type=float, default=CONF)
    args = ap.parse_args()
    want = set(s for s in args.only.split(",") if s) if args.only else None
    out_root = args.out_root if os.path.isabs(args.out_root) \
        else os.path.join(REPO, args.out_root)
    CONF = args.conf

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    from ultralytics import YOLOWorld
    try:
        yolo = YOLOWorld(WEIGHTS)
    except Exception as e:
        print(f"[yolo] {WEIGHTS} failed ({e}); fallback yolov8s-worldv2.pt")
        yolo = YOLOWorld("yolov8s-worldv2.pt")
    yolo.set_classes([PROMPT])

    for bag, frames_dir in BAGS:
        if want and bag not in want:
            continue
        out_dir = os.path.join(out_root, bag)
        if not os.path.isdir(out_dir):
            print(f"[skip] {bag}: no output dir (inference missing)")
            continue
        ov_dir = os.path.join(out_dir, "audit_overlay")
        os.makedirs(ov_dir, exist_ok=True)
        sel = load_selected(os.path.join(out_dir, "yolo_ism_results.csv"))

        fd = os.path.join(REPO, frames_dir) if frames_dir else ""
        prop_rows = []
        n = 0
        for frame_idx, name, bgr in yolo_ism.resolve_frames(
                os.path.join("data", "ros2_bag", bag), fd, STRIDE, 0,
                "/camera/camera/color/image_raw"):
            boxes, scores = yolo_proposals(yolo, bgr, device)
            decision, sel_box = sel.get(frame_idx, ("", None))
            sel_idx = -1
            if sel_box is not None:
                ious = [iou(b, sel_box) for b in boxes]
                if ious and max(ious) > 0.3:
                    sel_idx = int(np.argmax(ious))
            for r, (box, sc) in enumerate(zip(boxes, scores), start=1):
                prop_rows.append([bag, frame_idx, name, r, *box, round(sc, 4),
                                  int(r - 1 == sel_idx)])
            ov = draw(bgr, boxes, scores, sel_box, decision)
            cv2.imwrite(os.path.join(ov_dir, f"{name}.jpg"), ov)
            n += 1

        with open(os.path.join(out_dir, "proposals.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["bag", "frame_idx", "name", "prop_rank",
                        "x1", "y1", "x2", "y2", "yolo_score", "is_selected"])
            w.writerows(prop_rows)
        print(f"[ok] {bag}: {n} frames, {len(prop_rows)} proposals, overlays -> {ov_dir}")


if __name__ == "__main__":
    main()
