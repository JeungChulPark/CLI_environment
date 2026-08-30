#!/usr/bin/env python3
"""viz_phase1b.py — before/after for HSV SHADOW. READ-ONLY. Right panel is a SIMULATION
('would reject'), NOT an active decision. Real bag frames + real shadow_decisions.csv values.

Case 1 (the win): a cross-fire FP that Phase 1A accepted -> HSV color mismatch -> would reject.
Case 2 (the cost): a REAL Dinosaur TP that HSV would ALSO reject (green render/real domain gap).
"""
import csv, os, sys
from pathlib import Path

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE)
CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
OUTDIR = os.path.expanduser(
    "~/temp_ws/CLI_environment/sam6d_ws/_bmad-output/implementation-artifacts/phase1b-visualizations")
os.makedirs(OUTDIR, exist_ok=True)
GREEN, RED, ORANGE, GRAY, BLACK = (0, 200, 0), (0, 0, 255), (0, 140, 255), (150, 150, 150), (0, 0, 0)

DEC = {(r["dataset"], int(r["frame_id"]), r["object"]): r
       for r in csv.DictReader(open(os.path.join(HERE, "shadow_decisions.csv")))}

# (dataset, frame, object, kind, out_name)
CASES = [
    ("sam_105652", 175, "Rabbit", "fp_win", "shadow_crossfire_fp_would_reject.png"),
    ("sam_110633", 225, "Dinosaur", "tp_cost", "shadow_dinosaur_tp_casualty.png"),
]


def get_frame(ds, fidx):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    ts = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(os.path.join(CONV, ds))], default_typestore=ts) as rd:
        conns = [c for c in rd.connections if c.topic == "/camera/camera/color/image_raw"]
        i = -1
        for conn, t, raw in rd.messages(connections=conns):
            i += 1
            if i < fidx:
                continue
            m = rd.deserialize(raw, conn.msgtype)
            b = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
            return cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else b.copy()
    return None


def txt(img, s, org, col, sc=0.5, th=1):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, sc, BLACK, th + 2, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, sc, col, th, cv2.LINE_AA)


def panel(frame, box, obj, side, kind, hsv, thr, sem, appe):
    img = frame.copy()
    x1, y1, x2, y2 = box
    if side == "left":
        col = RED if kind == "fp_win" else GREEN
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
        tag = f"{obj}: ACCEPTED (FP)" if kind == "fp_win" else f"{obj}: DETECTED (TP)"
        title = "Phase 1A (HSV off): accepted"
    else:
        col = ORANGE
        for xx in range(x1, x2, 12):
            cv2.line(img, (xx, y1), (min(xx + 6, x2), y1), col, 2)
            cv2.line(img, (xx, y2), (min(xx + 6, x2), y2), col, 2)
        for yy in range(y1, y2, 12):
            cv2.line(img, (x1, yy), (x1, min(yy + 6, y2)), col, 2)
            cv2.line(img, (x2, yy), (x2, min(yy + 6, y2)), col, 2)
        tag = f"{obj}: SHADOW would REJECT"
        title = "Phase 1B SHADOW (simulated, not active)"
    txt(img, tag, (max(2, x1), max(14, y1 - 6)), col, 0.5, 1)
    H, W = img.shape[:2]
    head = np.full((30, W, 3), 245, np.uint8); txt(head, title, (8, 21), BLACK, 0.58, 2)
    cap = np.full((104, W, 3), 245, np.uint8)
    if side == "left":
        if kind == "fp_win":
            L = [(f"sem={sem}>=0.35 appe={appe}>=0.55 PASS -> accepted", BLACK),
                 (f"but this box is NOT a {obj} (doll-prompt cross-fire)", RED),
                 (f"low YOLO threshold let a wrong-color candidate through -> FP", RED)]
        else:
            L = [(f"sem={sem}>=0.35 appe={appe}>=0.55 PASS -> accepted", BLACK),
                 (f"this IS a real {obj} -> correct detection (TP)", (0, 130, 0)),
                 (f"HSV active would wrongly drop it (see right)", ORANGE)]
    else:
        good = kind == "fp_win"
        L = [(f"HSV color score = {hsv}  <  threshold {thr}", (0, 130, 0) if good else RED),
             (("color does NOT match {}'s render templates".format(obj)) if good
              else "render green != real green (domain gap)", (0, 130, 0) if good else RED),
             (("=> active gate would REMOVE this FP (correct)" if good
               else "=> active gate would REMOVE a real TP (casualty)"),
              (0, 130, 0) if good else RED)]
    for i, (s, c) in enumerate(L):
        txt(cap, s, (8, 22 + 26 * i), c, 0.48, 1)
    return np.vstack([head, img, cap])


for ds, fr, obj, kind, name in CASES:
    r = DEC.get((ds, fr, obj))
    frame = get_frame(ds, fr)
    if r is None or frame is None:
        print(f"[skip] {ds} f{fr} {obj}")
        continue
    box = (int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"]))
    left = panel(frame, box, obj, "left", kind, r["hsv_score"], r["t_hsv"], r["sem"], r["appe"])
    right = panel(frame, box, obj, "right", kind, r["hsv_score"], r["t_hsv"], r["sem"], r["appe"])
    gap = np.full((left.shape[0], 8, 3), 255, np.uint8)
    combo = np.hstack([left, gap, right])
    strip = np.full((28, combo.shape[1], 3), 255, np.uint8)
    vis = "visible" if r["visible"] == "1" else "NOT visible"
    txt(strip, f"{ds} frame {fr}  |  GT: {obj} {vis}  |  box {box}  |  RIGHT = SHADOW SIMULATION (not active)",
        (8, 19), BLACK, 0.5, 1)
    out = np.vstack([strip, combo])
    p = os.path.join(OUTDIR, name)
    cv2.imwrite(p, out)
    print(f"wrote {p}  ({out.shape[1]}x{out.shape[0]}) hsv={r['hsv_score']} vis={r['visible']} b1_accept={r['b1_accept']}")
