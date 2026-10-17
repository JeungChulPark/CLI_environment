#!/usr/bin/env python3
"""yolo_examples.py — YOLO-World 가 못 잡는 객체마다, 그 객체가 잘 보이는(visib 최대) 프레임 한 장에서
장면 프롬프트로 YOLO-World 를 돌려 박스를 그린다. 초록 = GT 박스, 주황 = YOLO-World 박스(프롬프트·신뢰도)."""
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[3]
SAM6D = REPO / "sam6d_realtime"
YCBV = Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video/test")
out = Path(sys.argv[1]).resolve()
WANT = ["ycbv_pudding_box", "ycbv_gelatin_box", "ycbv_potted_meat_can", "ycbv_foam_brick", "ycbv_large_clamp", "ycbv_scissors"]
sys.path.insert(0, str(SAM6D)); os.chdir(SAM6D)
import yolo_ism_object_n as o_n
import ultralytics.nn.text_model as _tm
_tm.WEIGHTS_DIR = SAM6D / "assets"
from ultralytics import YOLOWorld

table = json.loads((REPO / "objpose/rt/bop/ycbv_objects.json").read_text())["objects"]
by_id = {o["obj_id"]: o["name"] for o in table}
id_of = {v: k for k, v in by_id.items()}
defaults, all_objs = o_n.load_config(str(SAM6D / "configs/yolo_ism_ycbv.yaml"))
prompt = {o["name"]: o["yolo_prompt"] for o in all_objs}
tiles = []
for nm in WANT:
    best = None
    for sc in sorted(YCBV.iterdir()):
        gt = json.loads((sc / "scene_gt.json").read_text()); info = json.loads((sc / "scene_gt_info.json").read_text())
        for f, rows in gt.items():
            for j, r in enumerate(rows):
                if r["obj_id"] == id_of[nm] and int(f) % 10 == 1:
                    v = info[f][j]["visib_fract"]
                    if best is None or v > best[0]:
                        best = (v, sc, f, info[f][j]["bbox_visib"], sorted({by_id[x["obj_id"]] for x in rows}))
    v, sc, f, bb, names = best
    prompts = list(dict.fromkeys(prompt[n] for n in names))
    yolo = YOLOWorld(o_n._abspath(defaults.get("weights", "yolov8m-worldv2.pt")))
    yolo.set_classes(prompts)
    img = cv2.imread(str(sc / "rgb" / f"{int(f):06d}.png"))
    with o_n.multi_label_nms(True):
        r = yolo.predict(img, conf=0.02, imgsz=640, verbose=False, device="cuda:0")
    b = r[0].boxes
    x, y, w, h = bb
    cv2.rectangle(img, (x, y), (x + w, y + h), (60, 200, 60), 3)
    for j in range(len(b)):
        x1, y1, x2, y2 = map(int, b.xyxy[j].tolist()); lab = f"{prompts[int(b.cls[j])]} {float(b.conf[j]):.2f}"
        cv2.rectangle(img, (x1, y1), (x2, y2), (30, 138, 224), 2)
        cv2.putText(img, lab, (x1, max(40, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, lab, (x1, max(40, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.rectangle(img, (0, 0), (640, 28), (255, 255, 255), -1)
    cv2.putText(img, f"{sc.name} #{int(f)}  green = GT {nm.replace('ycbv_', '')}  (prompt: '{prompt[nm]}')",
                (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 1, cv2.LINE_AA)
    tiles.append(img)
    print(nm, sc.name, f, round(v, 2), len(b), "boxes")
sp = lambda s: np.full(s, 255, np.uint8)
rows = [np.concatenate([tiles[r * 3], sp((480, 8, 3)), tiles[r * 3 + 1], sp((480, 8, 3)), tiles[r * 3 + 2]], 1) for r in range(2)]
grid = np.concatenate([rows[0], sp((8, rows[0].shape[1], 3)), rows[1]], 0)
cv2.imwrite(str(out / "yolo_failures.png"), cv2.resize(grid, None, fx=0.75, fy=0.75, interpolation=cv2.INTER_AREA))
