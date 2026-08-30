#!/usr/bin/env python3
"""crop_montage.py — real Dinosaur crops: HSV-rejected vs HSV-passing, + a render template.
Confirms whether rejected 'Dinosaur' boxes are actually on the green dinosaur. READ-ONLY.
"""
import csv, os, sys
from pathlib import Path
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
RES_ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(RES_ROOT)
REPO = os.path.dirname(RSRCH)
CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
VIZ = os.path.join(RES_ROOT, "visualizations")
os.makedirs(VIZ, exist_ok=True)
DEC = os.path.join(RSRCH, "phase1b_validation", "shadow_decisions.csv")


def frame(ds, fidx):
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


rows = [r for r in csv.DictReader(open(DEC)) if r["object"] == "Dinosaur" and r["visible"] == "1"
        and r["b1_accept"] == "1"]
rej = sorted([r for r in rows if r["would_hsv_reject"] == "1"],
             key=lambda r: (int(r["x2"]) - int(r["x1"])) * (int(r["y2"]) - int(r["y1"])), reverse=True)[:5]
pas = sorted([r for r in rows if r["would_hsv_reject"] == "0"],
             key=lambda r: (int(r["x2"]) - int(r["x1"])) * (int(r["y2"]) - int(r["y1"])), reverse=True)[:5]


def crop(r):
    f = frame(r["dataset"], int(r["frame_id"]))
    if f is None:
        return None
    x1, y1, x2, y2 = int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"])
    c = f[y1:y2, x1:x2]
    return cv2.resize(c, (120, 120)) if c.size else None


def strip(items, label, col):
    tiles = []
    for r in items:
        c = crop(r)
        if c is None:
            continue
        hsv = cv2.cvtColor(c, cv2.COLOR_BGR2HSV).reshape(-1, 3)
        hdeg = float(np.median(hsv[:, 0])) * 2
        cv2.putText(c, f"H{hdeg:.0f} s{r['hsv_score']}", (2, 115),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(c, f"H{hdeg:.0f} s{r['hsv_score']}", (2, 115),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1, cv2.LINE_AA)
        tiles.append(c)
    if not tiles:
        return None
    band = np.full((22, 120 * len(tiles) + 4 * (len(tiles) - 1), 3), 245, np.uint8)
    cv2.putText(band, label, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    row = tiles[0]
    for t in tiles[1:]:
        row = np.hstack([row, np.full((120, 4, 3), 255, np.uint8), t])
    return np.vstack([band, row])


# render template tile for reference
tdir = os.path.join(REPO, "template", "Dinosaur", "templates")
rimg = cv2.imread(os.path.join(tdir, "rgb_0.png"))
rmask = cv2.imread(os.path.join(tdir, "mask_0.png"), cv2.IMREAD_GRAYSCALE)
ys, xs = np.where(rmask > 0)
rt = cv2.resize(rimg[ys.min():ys.max(), xs.min():xs.max()], (120, 120))
hdeg = float(np.median(cv2.cvtColor(rt, cv2.COLOR_BGR2HSV).reshape(-1, 3)[:, 0])) * 2
cv2.putText(rt, f"H{hdeg:.0f}", (2, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1, cv2.LINE_AA)

s_rej = strip(rej, "REAL 'Dinosaur' HSV-REJECTED (would_reject=1)  -- brown/orange?", (0, 0, 255))
s_pas = strip(pas, "REAL Dinosaur HSV-PASSED (TP)  -- green", (0, 130, 0))
refband = np.full((22, 124, 3), 245, np.uint8)
cv2.putText(refband, "render", (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
ref = np.vstack([refband, np.hstack([rt, np.full((120, 4, 3), 255, np.uint8)])])

W = max(s_rej.shape[1], s_pas.shape[1], ref.shape[1])
def pad(a):
    return np.hstack([a, np.full((a.shape[0], W - a.shape[1], 3), 255, np.uint8)]) if a.shape[1] < W else a
out = np.vstack([pad(ref), np.full((6, W, 3), 255, np.uint8),
                 pad(s_pas), np.full((6, W, 3), 255, np.uint8), pad(s_rej)])
cv2.imwrite(os.path.join(VIZ, "dinosaur_reject_vs_pass_crops.png"), out)
print(f"wrote {VIZ}/dinosaur_reject_vs_pass_crops.png  ({out.shape[1]}x{out.shape[0]})")
print(f"rejected crops: {len(rej)}  passing crops: {len(pas)}")
