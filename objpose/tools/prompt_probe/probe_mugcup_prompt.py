#!/usr/bin/env python3
"""probe_mugcup_prompt.py — compare YOLO-World prompts for Mugcup_high on a converted SAM session (2026-09-29).

The live pipeline runs every enabled object's prompt in ONE YOLO-World call (multi-label NMS), so each candidate
Mugcup prompt is tested together with the other 7 prompts unchanged. A Mugcup box counts as a hit when it is mostly
pink (the mug is pastel pink; HSV hue >= 140 or <= 8, saturation 25..170, value >= 90), otherwise as a false box.
No ground truth is used: this measures where the detector looks, the ISM/PEM still decide acceptance.
"""
import argparse, json, sys
from pathlib import Path
import cv2, numpy as np, yaml

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose" / "pc"))
sys.path.insert(0, str(REPO / "sam6d_realtime"))
from conv_session import ConvSession                     # noqa: E402
from yolo_ism_object_n import multi_label_nms           # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--session", required=True)
ap.add_argument("--every", type=int, default=4)
ap.add_argument("--conf", type=float, default=0.02)
ap.add_argument("--prompts", nargs="+", required=True)
ap.add_argument("--out", required=True)
a = ap.parse_args()

cfg = yaml.safe_load(open(REPO / "sam6d_realtime/configs/yolo_ism_objects.yaml"))
others = [o["yolo_prompt"] for o in cfg["objects"] if o.get("enabled", True) and o["name"] not in ("Mugcup_high", "scenes")]
from ultralytics import YOLOWorld
WEIGHTS = str(REPO / "sam6d_realtime" / cfg["defaults"].get("weights", "yolov8m-worldv2.pt"))
sess = ConvSession(a.session)
idx = list(range(0, len(sess), a.every))
frames = [sess.read(i, with_depth=False).color_bgr for i in idx]

def pink_frac(img, box):
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    crop = img[max(y1, 0):y2, max(x1, 0):x2]
    if crop.size == 0:
        return 0.0
    h, s, v = cv2.split(cv2.cvtColor(crop, cv2.COLOR_BGR2HSV))
    m = ((h >= 140) | (h <= 8)) & (s >= 25) & (s <= 170) & (v >= 90)
    return float(m.mean())

res_all = {}
for p in a.prompts:
    classes = others + [p]
    yolo = YOLOWorld(WEIGHTS)          # fresh model: set_classes after a CUDA predict mixes devices
    yolo.set_classes(classes)
    hits, false, confs, hit_frames = 0, 0, [], []
    for fi, img in zip(idx, frames):
        with multi_label_nms(True):
            r = yolo.predict(img, conf=a.conf, imgsz=640, verbose=False)[0]
        got = False
        for b in r.boxes:
            if int(b.cls) != len(classes) - 1:
                continue
            box = b.xyxy[0].tolist(); c = float(b.conf)
            if pink_frac(img, box) >= 0.25:
                hits += 1; confs.append(c); got = True
            else:
                false += 1
        if got:
            hit_frames.append(fi)
    res_all[p] = {"frames": len(idx), "frames_with_pink_box": len(hit_frames), "pink_boxes": hits, "other_boxes": false,
                  "pink_conf_median": round(float(np.median(confs)), 3) if confs else None,
                  "pink_conf_max": round(max(confs), 3) if confs else None}
    print(f"{p!r:32s} pink-box frames {len(hit_frames):4d}/{len(idx)}  pink {hits:4d}  other {false:4d}  "
          f"conf med {res_all[p]['pink_conf_median']} max {res_all[p]['pink_conf_max']}", flush=True)
Path(a.out).write_text(json.dumps({"session": a.session, "every": a.every, "conf": a.conf, "results": res_all}, indent=1))
