#!/usr/bin/env python3
"""plot_stages.py — diag_ycbv_stages.py 결과(stages.json)로 객체별 '어디서 떨어졌나' 막대와 YOLO 실패 예시 이미지를 만든다."""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

d = Path(sys.argv[1])
YCBV = Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video/test")
rep = json.loads((d / "stages.json").read_text())
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "axes.unicode_minus": False, "font.size": 12,
                     "axes.spines.top": False, "axes.spines.right": False})

objs = sorted(rep["objects"].items(), key=lambda kv: kv[1]["s4"] / kv[1]["gt"])
seg = [("최종 포즈 맞음", "#3a9d5d"), ("포즈 단계에서 탈락", "#9ecae1"), ("ISM 검증에서 탈락", "#f2c14e"),
       ("YOLO: 박스는 냈으나 다른 이름", "#e08a1e"), ("YOLO: 박스 자체가 없음", "#b8342a")]
fig, ax = plt.subplots(figsize=(13, 9))
for i, (nm, c) in enumerate(objs):
    g = c["gt"]
    s1, s2, s3, s4 = (c[k] / g * 100 for k in ("s1", "s2", "s3", "s4"))
    s4 = min(s4, s3)
    vals = [s4, s3 - s4, s2 - s3, s1 - s2, 100 - s1]
    left = 0
    for (lab, col), v in zip(seg, vals):
        ax.barh(i, v, left=left, color=col, edgecolor="white", linewidth=1.5, label=lab if i == 0 else None)
        left += v
    ax.text(101, i, f"{g}", va="center", fontsize=10, color="#555")
ax.set_yticks(range(len(objs)), [nm.replace("ycbv_", "") + f"  ({c['prompt']})" for nm, c in objs], fontsize=11)
ax.set_xlim(0, 108); ax.set_xlabel("화면에 보이는 GT 인스턴스 중 비율 [%]  (오른쪽 숫자 = 인스턴스 수)")
ax.legend(ncol=3, fontsize=11, loc="upper center", bbox_to_anchor=(0.45, 1.09), frameon=False)
ax.set_title("YCB-Video 21개 객체 — 인식 단계별로 어디서 떨어졌나 (10프레임마다 1장, 괄호 = YOLO-World 프롬프트)", fontsize=14, pad=42)
fig.tight_layout(); fig.savefig(d / "stages_by_object.png", dpi=150); plt.close(fig)

# YOLO 실패 예시: 5개 객체 × 1장
want = ["ycbv_pudding_box", "ycbv_gelatin_box", "ycbv_potted_meat_can", "ycbv_foam_brick", "ycbv_large_clamp"]
tiles = []
for nm in want:
    cands = [s for s in rep["samples"] if s["obj"] == nm]
    if not cands:
        continue
    s = cands[len(cands) // 2]
    img = cv2.imread(str(YCBV / s["scene"] / "rgb" / f"{s['k'] + 1:06d}.png"))
    for bb, lab, cf in s["boxes"]:
        cv2.rectangle(img, tuple(bb[:2]), tuple(bb[2:]), (30, 138, 224), 2)
        cv2.putText(img, f"{lab} {cf:.2f}", (bb[0], max(12, bb[1] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, f"{lab} {cf:.2f}", (bb[0], max(12, bb[1] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    g = [int(v) for v in s["gt_box"]]
    cv2.rectangle(img, tuple(g[:2]), tuple(g[2:]), (60, 200, 60), 3)
    cv2.rectangle(img, (0, 0), (640, 28), (255, 255, 255), -1)
    cv2.putText(img, f"{s['scene']}  GT: {nm.replace('ycbv_', '')} (green)   orange = YOLO-World boxes",
                (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (20, 20, 20), 1, cv2.LINE_AA)
    tiles.append(img)
while len(tiles) < 6:
    tiles.append(np.full_like(tiles[0], 255))
rows = [np.concatenate([tiles[r * 3], np.full((480, 8, 3), 255, np.uint8), tiles[r * 3 + 1],
                        np.full((480, 8, 3), 255, np.uint8), tiles[r * 3 + 2]], 1) for r in range(2)]
grid = np.concatenate([rows[0], np.full((8, rows[0].shape[1], 3), 255, np.uint8), rows[1]], 0)
cv2.imwrite(str(d / "yolo_failures.png"), cv2.resize(grid, None, fx=0.75, fy=0.75, interpolation=cv2.INTER_AREA))
print("ok")
