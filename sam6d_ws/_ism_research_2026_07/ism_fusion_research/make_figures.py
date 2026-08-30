#!/usr/bin/env python3
"""make_figures.py — 보고서용 시각 자료 (READ-ONLY).

1. alpha_tradeoff.png     α 스윕의 FP/FN 상충과 AUROC
2. hsv_methods.png        HSV prototype 방식별 객체 AUROC
3. hue_correction.png     Hue 보정 전/후 렌더 vs 실사 색분포
"""
import csv, os
from collections import defaultdict

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
RES = os.path.join(HERE, "results")
FIG = os.path.join(HERE, "figures")
os.makedirs(FIG, exist_ok=True)
plt.rcParams["axes.grid"] = True
plt.rcParams["grid.alpha"] = 0.3
plt.rcParams["font.family"] = "DejaVu Sans"

# ------------------------------------------------------------------ 1. α
R = list(csv.DictReader(open(os.path.join(RES, "semantic_alpha_sweep.csv"))))
a = [float(r["alpha"]) for r in R]
fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
ax[0].plot(a, [int(r["FP"]) for r in R], "o-", color="tab:red", label="FP")
ax[0].plot(a, [int(r["FN"]) for r in R], "s-", color="tab:blue", label="FN")
ax[0].plot(a, [int(r["FP"]) + int(r["FN"]) for r in R], "^--", color="k", label="FP+FN")
ax[0].axvspan(0.25, 0.45, color="tab:green", alpha=0.15)
ax[0].text(0.35, 61, "stable plateau\nα=0.25–0.45", ha="center", fontsize=8, color="tab:green")
ax[0].set_xlabel("α   (1.0 = Top-5 only,  0.0 = 42-view mean only)")
ax[0].set_ylabel("held-out error count (6 folds pooled)")
ax[0].set_title("Decision-level FP / FN vs α"); ax[0].legend(fontsize=9)

ax[1].plot(a, [float(r["mean_auroc"]) for r in R], "o-", color="tab:purple", label="mean AUROC")
ax[1].plot(a, [float(r["min_object_auroc"]) for r in R], "s-", color="tab:orange",
           label="worst-object AUROC")
ax2 = ax[1].twinx()
ax2.plot(a, [float(r["f1"]) for r in R], "^--", color="tab:gray", label="F1")
ax2.set_ylabel("F1", color="tab:gray")
ax[1].axvspan(0.25, 0.45, color="tab:green", alpha=0.15)
ax[1].set_xlabel("α"); ax[1].set_ylabel("AUROC")
ax[1].set_title("Ranking quality vs α"); ax[1].legend(fontsize=9, loc="center right")
fig.suptitle("Common semantic score: Top-5 / 42-view-mean fusion (LODO held-out)", fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "alpha_tradeoff.png"), dpi=130)
plt.close(fig); print("figures/alpha_tradeoff.png")

# ------------------------------------------------------------------ 2. HSV 방식
G = list(csv.DictReader(open(os.path.join(RES, "hsv_global_calibration.csv"))))
per = defaultdict(lambda: defaultdict(list))
for r in G:
    per[r["object"]][r["method"]].append(float(r["auroc"]))
M = ["render42", "render42_hue_global", "render42_hue_perobj",
     "render42_hs_global", "render42_hs_global_lowsat", "ply_view42_hue_global"]
LB = ["render42\n(current)", "global hue", "per-object hue",
      "global hue+sat", "global hue+sat\n(low-sat exempt)", "PLY 42-view\n+global hue"]
objs = sorted(per)
x = np.arange(len(objs)); w = 0.13
fig, ax = plt.subplots(figsize=(15, 5.4))
for i, (m, lb) in enumerate(zip(M, LB)):
    v = [np.mean(per[o][m]) if per[o][m] else np.nan for o in objs]
    ax.bar(x + (i - 2.5) * w, v, w, label=lb)
ax.axhline(0.5, color="k", ls=":", lw=1)
ax.set_xticks(x); ax.set_xticklabels(objs, rotation=30, ha="right")
ax.set_ylabel("LODO held-out AUROC"); ax.set_ylim(0.3, 1.03)
ax.set_title("HSV prototype construction — per-object discriminability "
             "(real crops used only as evaluation reference)")
ax.legend(fontsize=8, ncol=3)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "hsv_methods.png"), dpi=130)
plt.close(fig); print("figures/hsv_methods.png")

# ------------------------------------------------------------------ 3. Hue 보정
Z = np.load(os.path.join(HERE, "ply_hsv", "prototype_colors.npz"))
labels = {r["uid"]: r["true_class"]
          for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}
DS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
realH = defaultdict(list)
for ds in DS:
    z = np.load(os.path.join(OBS, "hsv_features", f"{ds}_hsv.npz"))
    for u, h in zip(z["uid"], z["hist"]):
        c = labels.get(str(u))
        if c:
            realH[c].append(h[0:32])          # masked H32


def hue32(rgb, shift=0, gain=1.0):
    img = np.asarray(rgb, np.uint8).reshape(-1, 1, 3)[:, :, ::-1]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.int16)
    if shift:
        hsv[..., 0] = (hsv[..., 0] + int(round(shift / 2))) % 180
    if gain != 1.0:
        hsv[..., 1] = np.clip(hsv[..., 1] * gain, 0, 255)
    hsv = hsv.astype(np.uint8)
    h = cv2.calcHist([hsv], [0], None, [32], [0, 180]).flatten()
    return h / max(h.sum(), 1e-12)


SHOW = ["Bear", "Sauce_high", "saffron", "Sikhye_high", "Febreze_high", "Rabbit"]
fig, axes = plt.subplots(2, 3, figsize=(15, 7))
xs = np.arange(32) * (180 / 32)
for ax_, o in zip(axes.ravel(), SHOW):
    k = f"{o}__render42"
    if k not in Z.files or o not in realH:
        continue
    col = Z[k].reshape(-1, 3)
    ax_.plot(xs, np.mean(np.stack(realH[o]), 0), color="tab:green", lw=2.2,
             label=f"real crops (n={len(realH[o])})")
    ax_.plot(xs, hue32(col), color="tab:blue", lw=1.8, label="render (current)")
    ax_.plot(xs, hue32(col, -6, 1.3), color="tab:red", lw=1.8, ls="--",
             label="render + global (-6 deg, sat x1.3)")
    ax_.set_title(o, fontsize=10); ax_.set_xlabel("Hue (OpenCV 0-180)")
    ax_.set_ylabel("normalized count"); ax_.legend(fontsize=7)
fig.suptitle("Global colour calibration of render templates vs real crops "
             "(Rabbit is achromatic — hue carries no information there)", fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "hue_correction.png"), dpi=130)
plt.close(fig); print("figures/hue_correction.png")
