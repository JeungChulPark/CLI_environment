#!/usr/bin/env python3
"""make_object_reference.py — 객체 이름 참조 시트를 만든다 (READ-ONLY).

사용자가 `visible_objects` 에 적을 이름을 오타 없이 고르려면 이름 목록만으로는 부족하다.
각 이름이 실제로 어떤 물건인지 **라벨된 실사 crop** 과 **렌더 템플릿**을 나란히 보여준다.

산출: gt_input/OBJECT_REFERENCE.jpg
"""
import csv, os
from collections import defaultdict

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
REPO = os.path.dirname(RSRCH)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
GT = os.path.join(RSRCH, "gt_input")

NAMES = [
    ("Bear", "brown bear doll", "Bear"),
    ("Rabbit", "white rabbit doll", "Rabbit"),
    ("Dinosaur", "green dinosaur doll", "Dinosaur"),
    ("milk", "milk carton", "milk"),
    ("choco_hazelnut_high", "maroon box", "choco_hazelnut_color_high"),
    ("Febreze_high", "febreze spray bottle", "Febreze_high"),
    ("Mugcup_high", "mug cup", "Mugcup_color_high"),
    ("saffron", "white jug", "saffron"),
    ("Sauce_high", "orange bottle", "Sauce_high"),
    ("Sikhye_high", "yellow can", "Sikhye_high"),
]
CELL, NREAL = 200, 3

labels = defaultdict(list)
for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv"))):
    labels[r["true_class"]].append(r["uid"])

rows_img = []
for name, prompt, tdir in NAMES:
    cells = []
    # 렌더 템플릿 1장 — view 0 은 전부 top-down 이라 병뚜껑/원반처럼 보인다.
    # mask 면적이 가장 큰 view(대개 측면)를 골라 사람이 알아보기 쉽게 한다.
    td = os.path.join(REPO, "template", tdir, "templates")
    best, bi = -1, 0
    for i in range(42):
        mp = os.path.join(td, f"mask_{i}.png")
        m = cv2.imread(mp, cv2.IMREAD_GRAYSCALE)
        if m is not None and (m > 0).sum() > best:
            best, bi = (m > 0).sum(), i
    im = cv2.imread(os.path.join(td, f"rgb_{bi}.png"))
    cells.append(cv2.resize(im, (CELL, CELL)) if im is not None
                 else np.full((CELL, CELL, 3), 60, np.uint8))
    # 실사 crop
    n = 0
    for uid in labels.get(name, []):
        cp = os.path.join(OBS, "frame_dumps", "crops", uid.replace("|", "__") + ".png")
        im = cv2.imread(cp)
        if im is None or min(im.shape[:2]) < 20:
            continue
        cells.append(cv2.resize(im, (CELL, CELL)))
        n += 1
        if n >= NREAL:
            break
    while len(cells) < 1 + NREAL:
        cells.append(np.full((CELL, CELL, 3), 60, np.uint8))

    band = np.full((CELL, 330, 3), 25, np.uint8)
    cv2.putText(band, name, (8, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(band, f"({prompt})", (8, 108), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (170, 220, 170), 1, cv2.LINE_AA)
    cv2.putText(band, "render  |  real crops from the 6 bags", (8, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                (140, 140, 140), 1, cv2.LINE_AA)
    rows_img.append(np.hstack([band] + cells))

sheet = np.vstack(rows_img)
hdr = np.full((60, sheet.shape[1], 3), 15, np.uint8)
cv2.putText(hdr, "Allowed names for visible_objects  (copy EXACTLY - case and underscores)", (10, 38),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
out = os.path.join(GT, "OBJECT_REFERENCE.jpg")
cv2.imwrite(out, np.vstack([hdr, sheet]), [cv2.IMWRITE_JPEG_QUALITY, 93])
print(f"-> {out}  ({sheet.shape[1]}x{sheet.shape[0]+60})")
