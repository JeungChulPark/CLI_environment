#!/usr/bin/env python3
"""viz_phase1a.py — before/after (B0 vs B1) for real Rabbit recovery frames. READ-ONLY.

Left  panel  = B0 (per-object guard): the Rabbit candidate is REMOVED at the score_threshold
               stage (conf < guard) -> object ends up FN. Box drawn ORANGE = "Removed by threshold".
Right panel  = B1 (uniform 0.02):     the SAME candidate survives (conf >= 0.02), passes the
               downstream semantic + appearance gates -> Rabbit detected (TP). Box GREEN.

Frames/boxes come straight from the frozen cur dump + b0b1 eval (no synthesis). In-image text
is English (OpenCV cannot render Hangul); Korean narrative is in README.md.
"""
import csv, os
from pathlib import Path

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE)
PIPE = os.path.join(RSRCH, "yolo_localization_research", "pipeline", "cur")
CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
OUTDIR = os.path.expanduser(
    "~/temp_ws/CLI_environment/sam6d_ws/_bmad-output/implementation-artifacts/phase1a-visualizations")
os.makedirs(OUTDIR, exist_ok=True)

GREEN, ORANGE, RED, WHITE, BLACK = (0, 200, 0), (0, 140, 255), (0, 0, 255), (255, 255, 255), (0, 0, 0)
GUARD = {"Rabbit": 0.30, "Bear": 0.35, "Dinosaur": 0.30}

# (dataset, frame, object, out_name)
CASES = [
    ("sam_110633", 72, "Rabbit", "before_after_rabbit_recovered.png"),
    ("sam_110104", 382, "Rabbit", "before_after_additional_recovery.png"),
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


def selected_box(ds, fr, obj):
    rows = [r for r in csv.DictReader(open(os.path.join(PIPE, f"{ds}_pairs.csv")))
            if int(r["frame_id"]) == fr and r["object"] == obj
            and int(r["routed"]) == 1 and float(r["yolo_conf"]) >= 0.02]
    rows.sort(key=lambda r: -float(r["yolo_conf"]))
    top = rows[:3]
    if not top:
        return None
    b = max(top, key=lambda r: float(r["sem_top5"]))
    x1, y1, x2, y2 = (int(v) for v in b["uid"].split("|")[2].split("_"))
    return dict(box=(x1, y1, x2, y2), conf=float(b["yolo_conf"]),
               sem=float(b["sem_top5"]), appe=float(b["appe11_clstop1"]),
               simthr=float(b["sim_thr"]), appegate=float(b["appe_gate"]))


def putlines(img, x, y, lines, scale=0.5, thick=1):
    for i, (txt, col) in enumerate(lines):
        yy = y + int(22 * scale / 0.5) * i
        cv2.putText(img, txt, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, scale, BLACK, thick + 2, cv2.LINE_AA)
        cv2.putText(img, txt, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, scale, col, thick, cv2.LINE_AA)


def panel(frame, info, obj, mode):
    """mode 'B0' -> removed(orange)/FN ; 'B1' -> detected(green)/TP"""
    img = frame.copy()
    x1, y1, x2, y2 = info["box"]
    guard = GUARD[obj]
    if mode == "B0":
        col = ORANGE
        # dashed rectangle to signal "candidate was killed"
        for xx in range(x1, x2, 12):
            cv2.line(img, (xx, y1), (min(xx + 6, x2), y1), col, 2)
            cv2.line(img, (xx, y2), (min(xx + 6, x2), y2), col, 2)
        for yy in range(y1, y2, 12):
            cv2.line(img, (x1, yy), (x1, min(yy + 6, y2)), col, 2)
            cv2.line(img, (x2, yy), (x2, min(yy + 6, y2)), col, 2)
        tag = f"{obj}: REMOVED by threshold -> FN"
    else:
        col = GREEN
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
        tag = f"{obj}: DETECTED -> TP"
    cv2.putText(img, tag, (max(2, x1), max(14, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, BLACK, 3, cv2.LINE_AA)
    cv2.putText(img, tag, (max(2, x1), max(14, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, col, 1, cv2.LINE_AA)

    # header + caption bars
    H, W = img.shape[:2]
    head = np.full((34, W, 3), 245, np.uint8)
    title = "Before: per-object guard" if mode == "B0" else "After: uniform threshold 0.02"
    cv2.putText(head, title, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.62, BLACK, 2, cv2.LINE_AA)
    cap = np.full((96, W, 3), 245, np.uint8)
    if mode == "B0":
        lines = [(f"conf={info['conf']:.4f}  <  guard={guard:.2f}  -> dropped at candidate stage", RED),
                 (f"downstream ISM never sees it  ->  {obj} = MISS (FN)", RED),
                 (f"applied score_threshold = {guard:.2f}", BLACK)]
    else:
        lines = [(f"conf={info['conf']:.4f}  >=  0.02  -> candidate kept", (0, 130, 0)),
                 (f"sem={info['sem']:.3f}>={info['simthr']:.2f}  appe={info['appe']:.3f}>={info['appegate']:.2f} PASS", (0, 130, 0)),
                 (f"applied score_threshold = 0.02  ->  {obj} = DETECTED (TP)", BLACK)]
    putlines(cap, 8, 22, lines, 0.48, 1)
    return np.vstack([head, img, cap])


for ds, fr, obj, name in CASES:
    frame = get_frame(ds, fr)
    info = selected_box(ds, fr, obj)
    if frame is None or info is None:
        print(f"[skip] {ds} f{fr}: frame/box missing")
        continue
    left = panel(frame, info, obj, "B0")
    right = panel(frame, info, obj, "B1")
    gap = np.full((left.shape[0], 8, 3), 255, np.uint8)
    combo = np.hstack([left, gap, right])
    # top strip with the case id
    strip = np.full((30, combo.shape[1], 3), 255, np.uint8)
    cv2.putText(strip, f"{ds}  frame {fr}   |   GT: {obj} visible   |   box {info['box']}",
                (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, BLACK, 1, cv2.LINE_AA)
    # legend
    leg = np.full((26, combo.shape[1], 3), 255, np.uint8)
    cv2.rectangle(leg, (8, 6), (26, 20), GREEN, -1); cv2.putText(leg, "detected (TP)", (32, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, BLACK, 1, cv2.LINE_AA)
    cv2.rectangle(leg, (170, 6), (188, 20), ORANGE, -1); cv2.putText(leg, "removed by threshold (FN)", (194, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, BLACK, 1, cv2.LINE_AA)
    out = np.vstack([strip, combo, leg])
    path = os.path.join(OUTDIR, name)
    cv2.imwrite(path, out)
    print(f"wrote {path}  ({out.shape[1]}x{out.shape[0]})  conf={info['conf']:.4f} sem={info['sem']:.3f} appe={info['appe']:.3f}")
