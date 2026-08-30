#!/usr/bin/env python3
"""make_figures.py — 보고서용 시각 자료 생성 (READ-ONLY).

1. semantic_rank_curves.png     순위별 AUROC 곡선 (top5 유리 / mean 유리 객체 대비)
2. semantic_view_curves_<obj>.png  대표 객체의 42-view 유사도 곡선 (pos vs neg)
3. render_old_vs_high_<obj>.png    기존 렌더 vs high PLY 렌더 나란히 + 차분
4. hsv_render_vs_real_<obj>.png    렌더/PLY정점색/실사 crop 의 Hue 분포
5. gt_seed_propagation.png         최소 GT seed 와 인접 프레임 전파 예시
"""
import csv, os
from collections import defaultdict

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
REPO = os.path.dirname(RSRCH)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
RES = os.path.join(ROOT, "results")
FIG = os.path.join(ROOT, "figures")
os.makedirs(FIG, exist_ok=True)
plt.rcParams["font.family"] = "DejaVu Sans"
plt.rcParams["axes.grid"] = True
plt.rcParams["grid.alpha"] = 0.3

# ---------------------------------------------------------------- 1. 순위별 AUROC
rank = defaultdict(dict)
for r in csv.DictReader(open(os.path.join(RES, "semantic_rank_auroc.csv"))):
    rank[r["object"]][int(r["rank"])] = float(r["auroc"])
dec = {r["object"]: r for r in csv.DictReader(open(os.path.join(RES, "semantic_tail_decomposition.csv")))}

fig, ax = plt.subplots(1, 2, figsize=(13, 5))
better = [o for o in dec if float(dec[o]["mean_minus_top5"]) > 0.005]
worse = [o for o in dec if float(dec[o]["mean_minus_top5"]) < -0.005]
for grp, a, title in ((better, ax[0], "mean improves (tail carries signal)"),
                      (worse, ax[1], "mean degrades (tail is noise)")):
    for o in sorted(grp):
        ks = sorted(rank[o])
        a.plot(ks, [rank[o][k] for k in ks], marker="o", ms=3, lw=1.4, label=o)
    a.axhline(0.5, color="k", ls=":", lw=1)
    a.axvspan(1, 5, color="tab:orange", alpha=0.12)
    a.text(3, 0.02, "top-5", ha="center", fontsize=9, color="tab:orange")
    a.set_xlabel("similarity rank k  (1 = best matching template view)")
    a.set_ylabel("AUROC of the k-th ranked similarity")
    a.set_title(title); a.set_ylim(0, 1.05); a.legend(fontsize=8)
fig.suptitle("Where the discriminative signal lives across the 42 template views", fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "semantic_rank_curves.png"), dpi=130)
plt.close(fig)
print("figures/semantic_rank_curves.png")

# ---------------------------------------------------------------- 2. 42-view 곡선
rows = list(csv.DictReader(open(os.path.join(RES, "semantic_view_similarity.csv"))))
by = defaultdict(lambda: {"pos": [], "neg": []})
for r in rows:
    s = np.sort(np.array([float(x) for x in r["sims"].split(";")]))[::-1]
    by[r["object"]]["pos" if int(r["is_positive"]) else "neg"].append(s)

SHOW = ["Dinosaur", "choco_hazelnut_high", "saffron", "Febreze_high", "Sauce_high", "Bear"]
fig, axes = plt.subplots(2, 3, figsize=(15, 7.5), sharex=True)
for a, o in zip(axes.ravel(), SHOW):
    P, N = np.stack(by[o]["pos"]), np.stack(by[o]["neg"])
    x = np.arange(1, P.shape[1] + 1)
    for M, c, lab in ((P, "tab:blue", f"positive (n={len(P)})"),
                      (N, "tab:red", f"negative (n={len(N)})")):
        m, lo, hi = M.mean(0), np.percentile(M, 25, 0), np.percentile(M, 75, 0)
        a.plot(x, m, color=c, lw=1.8, label=lab)
        a.fill_between(x, lo, hi, color=c, alpha=0.18)
    a.axvspan(1, 5, color="tab:orange", alpha=0.12)
    d = dec[o]
    a.set_title(f"{o}\ntop5 {d['auroc_top5']} / tail {d['auroc_tail6_42']} / mean {d['auroc_mean']}",
                fontsize=9)
    a.legend(fontsize=7)
    a.set_xlabel("rank"); a.set_ylabel("CLS cosine")
fig.suptitle("42-view CLS similarity curves — sorted descending (shaded = IQR)", fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "semantic_view_curves.png"), dpi=130)
plt.close(fig)
print("figures/semantic_view_curves.png")

# ---------------------------------------------------------------- 3. 렌더 비교
TPL_DIR = {"Bear": "Bear", "saffron": "saffron", "Febreze_high": "Febreze_high",
           "Mugcup_high": "Mugcup_color_high"}
HIGH_DIR = {"Bear": "Bear_high", "saffron": "saffron_high",
            "Febreze_high": "Febreze_high", "Mugcup_high": "Mugcup_high"}
VIEWS = [0, 7, 14, 21, 28, 35]
for obj in TPL_DIR:
    old = os.path.join(REPO, "template", TPL_DIR[obj], "templates")
    new = os.path.join(ROOT, "high_ply_templates", HIGH_DIR[obj], "templates")
    if not os.path.isdir(new):
        continue
    fig, axes = plt.subplots(3, len(VIEWS), figsize=(2.1 * len(VIEWS), 6.6))
    for j, v in enumerate(VIEWS):
        a = cv2.imread(os.path.join(old, f"rgb_{v}.png"))
        b = cv2.imread(os.path.join(new, f"rgb_{v}.png"))
        if a is None or b is None:
            continue
        d = np.abs(a.astype(int) - b.astype(int)).sum(2).astype(np.uint8)
        for i, (im, t) in enumerate(((a[:, :, ::-1], "operational"),
                                     (b[:, :, ::-1], "*_high.ply"),
                                     (d, f"|diff| max={d.max()}"))):
            ax = axes[i, j]
            ax.imshow(im, cmap=None if i < 2 else "inferno",
                      vmin=None if i < 2 else 0, vmax=None if i < 2 else 30)
            ax.set_xticks([]); ax.set_yticks([]); ax.grid(False)
            if j == 0:
                ax.set_ylabel(t, fontsize=9)
            if i == 0:
                ax.set_title(f"view {v}", fontsize=8)
    fig.suptitle(f"{obj}: operational render vs *_high.ply render "
                 f"(diff scaled 0–30 of 255)", fontsize=11)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, f"render_old_vs_high_{obj}.png"), dpi=120)
    plt.close(fig)
    print(f"figures/render_old_vs_high_{obj}.png")

# ---------------------------------------------------------------- 4. Hue 분포
labels = {r["uid"]: r["true_class"]
          for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
realH = defaultdict(list)
for ds in DATASETS:
    z = np.load(os.path.join(OBS, "hsv_features", f"{ds}_hsv.npz"))
    for u, h in zip(z["uid"], z["hist"]):
        c = labels.get(str(u))
        if c:
            realH[c].append(h[0:32])              # masked H32
zt = np.load(os.path.join(OBS, "hsv_features", "template_render_hsv.npz"))

SHOW2 = ["Bear", "Sauce_high", "saffron", "Sikhye_high", "milk", "Febreze_high"]
fig, axes = plt.subplots(2, 3, figsize=(15, 7))
for a, o in zip(axes.ravel(), SHOW2):
    if o not in realH or o not in zt.files:
        continue
    x = np.arange(32) * (180 / 32)
    a.plot(x, np.mean(np.stack(realH[o]), 0), color="tab:green", lw=2,
           label=f"real crops (n={len(realH[o])})")
    a.plot(x, zt[o][:, 0:32].mean(0), color="tab:blue", lw=2, label="operational render")
    hi = os.path.join(ROOT, "high_ply_templates", HIGH_DIR.get(o, "_"), "templates")
    if os.path.isdir(hi):
        hs = []
        for i in range(42):
            im = cv2.imread(os.path.join(hi, f"rgb_{i}.png"))
            mk = cv2.imread(os.path.join(hi, f"mask_{i}.png"), 0)
            if im is None or mk is None:
                continue
            hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
            h = cv2.calcHist([hsv], [0], (mk > 0).astype(np.uint8) * 255, [32], [0, 180]).flatten()
            hs.append(h / max(h.sum(), 1e-12))
        if hs:
            a.plot(x, np.mean(hs, 0), color="tab:red", lw=1.6, ls="--", label="*_high.ply render")
    a.set_title(o, fontsize=10); a.set_xlabel("Hue (0–180, OpenCV)")
    a.set_ylabel("normalized count"); a.legend(fontsize=7)
fig.suptitle("Masked hue distribution: real crops vs renders", fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "hsv_render_vs_real.png"), dpi=130)
plt.close(fig)
print("figures/hsv_render_vs_real.png")

# ---------------------------------------------------------------- 5. GT seed / 전파
pstat = list(csv.DictReader(open(os.path.join(RES, "gt_propagation_stats.csv"))))
plan = [r for r in csv.DictReader(open(os.path.join(RES, "gt_cost_by_plan.csv")))
        if r["dataset"] != "합계"]
fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
ious = [float(r["iou_consecutive_median"]) for r in pstat if r["iou_consecutive_median"]]
ax[0].hist(ious, bins=20, color="tab:blue", alpha=0.8)
ax[0].axvline(0.5, color="r", ls="--", label="IoU 0.5")
ax[0].axvline(np.median(ious), color="k", ls="-",
              label=f"median {np.median(ious):.2f}")
ax[0].set_xlabel("median IoU between consecutive accepted boxes")
ax[0].set_ylabel("# (dataset, object)")
ax[0].set_title("Adjacent-frame box propagation viability"); ax[0].legend(fontsize=8)

names = ["A every10", "A every25", "A every50", "B state", "C track", "D uncertain"]
keys = ["planA_every10", "planA_every25", "planA_every50",
        "planB_state_changes", "planC_track_starts", "planD_uncertain_frames"]
tot = [sum(int(r[k]) for r in plan) for k in keys]
b = ax[1].bar(names, tot, color=["tab:green"] * 3 + ["tab:red", "tab:orange", "tab:red"])
ax[1].axhline(2596, color="k", ls=":", label="all 2,596 frames")
for r, v in zip(b, tot):
    ax[1].text(r.get_x() + r.get_width() / 2, v, f"{v:,}", ha="center", va="bottom", fontsize=9)
ax[1].set_ylabel("frames a human must inspect (6 datasets)")
ax[1].set_title("Manual cost by seed-GT plan"); ax[1].legend(fontsize=8)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "gt_seed_propagation.png"), dpi=130)
plt.close(fig)
print("figures/gt_seed_propagation.png")
