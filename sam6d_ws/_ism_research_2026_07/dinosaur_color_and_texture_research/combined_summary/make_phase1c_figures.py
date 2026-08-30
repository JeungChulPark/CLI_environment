#!/usr/bin/env python3
"""make_phase1c_figures.py — Phase 1C figures (Korean labels). READ-ONLY.
Outputs to outputs/phase1c_hsv_dinosaur_color_correction/figures/ + reproduces texture metrics.
"""
import csv, json, os, sys
from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
RSRCH = os.path.join(REPO, "_ism_research_2026_07")
sys.path.insert(0, REPO)
import ism_hsv, yolo_ism_object_n as o_n   # noqa

# Korean font if available
for fp in ["/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
           "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"]:
    if os.path.isfile(fp):
        font_manager.fontManager.addfont(fp)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp).get_name()
        break
plt.rcParams["axes.unicode_minus"] = False

FIG = os.path.join(REPO, "outputs", "phase1c_hsv_dinosaur_color_correction", "figures")
CSVO = os.path.join(REPO, "outputs", "phase1c_hsv_dinosaur_color_correction", "csv")
PIPE = os.path.join(RSRCH, "yolo_localization_research", "pipeline", "cur")
GT = os.path.join(RSRCH, "gt_input")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
MASKED = slice(0, 224); HS = slice(96, 224); T = 0.1214


def save(fig, sub, name):
    d = os.path.join(FIG, sub); os.makedirs(d, exist_ok=True)
    fig.savefig(os.path.join(d, name + ".png"), dpi=120, bbox_inches="tight")
    try:
        fig.savefig(os.path.join(d, name + ".svg"), bbox_inches="tight")
    except Exception:
        pass
    plt.close(fig); print("wrote", sub + "/" + name)


def auroc(s, y):
    s = np.asarray(s, float); y = np.asarray(y); P = int(y.sum()); N = len(y) - P
    if P == 0 or N == 0:
        return float("nan")
    r = np.argsort(np.argsort(s)) + 1.0
    u, inv, c = np.unique(s, return_inverse=True, return_counts=True)
    sm = np.zeros(len(c)); np.add.at(sm, inv, r); r = (sm / c)[inv]
    return float((r[y == 1].sum() - P * (P + 1) / 2) / (P * N))

# ---------- data: HSV scores per candidate (config C) ----------
defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
tdir = {o["name"]: o["template_dir"] for o in objs}
PROTO = {o["name"]: (ism_hsv.load_cache(os.path.join(os.path.dirname(o["cls_cache"]),
        f"{o['name']}_hsv.npz"), o["template_dir"])[0]) for o in objs}
gt = {}
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] == "yes":
        gt[(r["dataset_name"], int(r["frame_id"]))] = set(
            t.strip() for t in r["visible_objects"].split(";") if t.strip())
hsv_tp, hsv_fp = [], []
for ds in DATASETS:
    z = np.load(os.path.join(PIPE, f"{ds}_hsv.npz"))
    hq = {str(u): h[MASKED][HS].astype(np.float64) for u, h in zip(z["uid"], z["hist"])}
    rows = defaultdict(list)
    for r in csv.DictReader(open(os.path.join(PIPE, f"{ds}_pairs.csv"))):
        if (ds, int(r["frame_id"])) not in gt:
            continue
        rows[(ds, int(r["frame_id"]), r["object"])].append(r)
    for (dd, ff, o), cs in rows.items():
        v = [c for c in cs if int(c["routed"]) == 1 and float(c["yolo_conf"]) >= 0.02]
        v.sort(key=lambda c: -float(c["yolo_conf"])); v = v[:3]
        if not v:
            continue
        b = max(v, key=lambda c: float(c["sem_top5"]))
        if not ((float(b["sem_top5"]) >= float(b["sim_thr"])) and (float(b["appe11_clstop1"]) >= float(b["appe_gate"]))):
            continue
        q = hq.get(b["uid"])
        s = ism_hsv.similarity(q, PROTO[o]) if q is not None else 1.0
        (hsv_tp if o in gt[(dd, ff)] else hsv_fp).append(s)
hsv_tp, hsv_fp = np.array(hsv_tp), np.array(hsv_fp)

# ===== FIG 1: HSV score distribution =====
fig, ax = plt.subplots(figsize=(8, 4.5))
bins = np.linspace(0, 1, 40)
ax.hist(hsv_tp, bins, alpha=0.6, color="#2a9d8f", label=f"실제 객체 TP (n={len(hsv_tp)})", hatch="//")
ax.hist(hsv_fp, bins, alpha=0.6, color="#d1495b", label=f"오검출 FP (n={len(hsv_fp)})", hatch="\\\\")
ax.axvline(T, color="black", ls="--", lw=2, label=f"threshold {T} (이상이면 통과)")
ax.set_title("HSV 색상 유사도 분포 — 통과 기준 score ≥ 0.1214 (높을수록 색이 비슷)")
ax.set_xlabel("HSV 색상 유사도 score (0~1)"); ax.set_ylabel("후보 수"); ax.legend()
ax.text(0.02, 0.95, "출처: 사람 GT 935 · Phase 1C 운영 캐시", transform=ax.transAxes,
        fontsize=8, va="top", color="gray")
save(fig, "hsv", "hsv_score_distribution")

# ===== FIG 2: Dinosaur before/after (+9 correction) =====
fig, ax = plt.subplots(1, 2, figsize=(11, 4.3))
ax[0].bar(["보정 전\n(hc=0)", "보정 후\n(+9 OpenCV = +18°)"], [33, 39],
          color=["#adb5bd", "#2a9d8f"], hatch=["", "//"])
ax[0].set_ylim(0, 45); ax[0].set_ylabel("Dinosaur TP (사람 GT)")
ax[0].set_title("Dinosaur 색 보정: TP 33 → 39 (+6), 새 FP 0")
for i, v in enumerate([33, 39]):
    ax[0].text(i, v + 0.5, str(v), ha="center", fontweight="bold")
# hue distribution render vs +9 vs real
labels = ["렌더\n템플릿", "렌더+9°\n(보정)", "실제\nDinosaur"]
hue = [78, 96, 100]; err = [8, 8, 13]
ax[1].bar(labels, hue, yerr=err, color=["#9acd32", "#4caf50", "#1b7a3d"], capsize=6,
          hatch=["", "//", ".."])
ax[1].set_ylabel("Hue (0~360°)"); ax[1].set_ylim(50, 120)
ax[1].set_title("보정으로 렌더 Hue를 실제 Dinosaur쪽으로 이동")
for i, h in enumerate(hue):
    ax[1].text(i, h + err[i] + 1, f"{h}°", ha="center")
save(fig, "dinosaur", "dinosaur_before_after")

# ===== texture data from earlier extraction =====
TX = os.path.join(RSRCH, "dinosaur_color_and_texture_research", "research_b_texture", "results",
                  "texture_all_candidates.csv")
trows = list(csv.DictReader(open(TX)))
# choco human-GT
ch = [r for r in trows if r["object"] == "choco_hazelnut_high"]
cy = np.array([int(r["visible"]) for r in ch]); cs = np.array([float(r["texture_p2"]) for r in ch])
au_choco = auroc(cs, cy)
# rescue
rej = [r for r in trows if r["would_hsv_reject"] == "1"]
rt = np.array([float(r["texture_p2"]) for r in rej if r["visible"] == "1"])
rf = np.array([float(r["texture_p2"]) for r in rej if r["visible"] == "0"])
au_res = auroc(np.r_[rt, rf], np.r_[np.ones(len(rt)), np.zeros(len(rf))])

# ===== FIG 3: texture negative verifier (ROC + tradeoff) =====
fig, ax = plt.subplots(1, 2, figsize=(11, 4.4))
th = np.unique(np.r_[cs, [0, 1]]); tpr = []; fpr = []
for t in np.sort(th):
    tpr.append((cs[cy == 1] >= t).mean()); fpr.append((cs[cy == 0] >= t).mean())
ax[0].plot(fpr, tpr, "-o", ms=3, color="#264653")
ax[0].plot([0, 1], [0, 1], ":", color="gray")
ax[0].set_title(f"Choco vs 갈색박스 ROC (사람 GT) — AUROC {au_choco:.3f}")
ax[0].set_xlabel("FPR (갈색박스를 통과시킴)"); ax[0].set_ylabel("TPR (진짜 choco 유지)")
ax[0].text(0.3, 0.1, f"n choco={int(cy.sum())} / FP={len(cy)-int(cy.sum())}", fontsize=9)
grid = np.linspace(0.5, 0.72, 30)
keep = [(cs[cy == 1] >= t).sum() for t in grid]; rm = [(cs[cy == 0] < t).sum() for t in grid]
ax[1].plot(grid, keep, "-s", ms=3, color="#2a9d8f", label="choco TP 유지")
ax2 = ax[1].twinx(); ax2.plot(grid, rm, "-^", ms=3, color="#d1495b", label="FP 제거")
ax[1].set_xlabel("texture threshold"); ax[1].set_ylabel("choco TP 유지 수", color="#2a9d8f")
ax2.set_ylabel("FP 제거 수", color="#d1495b")
ax[1].set_title("깨끗한 임계점 없음 (분포 겹침) → 운영 미채택")
save(fig, "texture_negative", "texture_negative_roc_tradeoff")

# ===== FIG 4: texture rescue (overlap + readmission) =====
fig, ax = plt.subplots(1, 2, figsize=(11, 4.4))
bins = np.linspace(0.3, 0.9, 30)
ax[0].hist(rt, bins, alpha=0.6, color="#2a9d8f", label=f"진짜 TP casualty (n={len(rt)}, 평균 {rt.mean():.3f})", hatch="//")
ax[0].hist(rf, bins, alpha=0.6, color="#d1495b", label=f"HSV가 옳게 지운 FP (n={len(rf)}, 평균 {rf.mean():.3f})", hatch="\\\\")
ax[0].set_title(f"texture는 두 집단을 구분 못함 — AUROC {au_res:.3f} (우연)")
ax[0].set_xlabel("texture score"); ax[0].set_ylabel("후보 수"); ax[0].legend(fontsize=8)
grid = np.linspace(0.5, 0.75, 30)
rec_tp = [(rt >= t).sum() for t in grid]; read_fp = [(rf >= t).sum() for t in grid]
ax[1].plot(grid, rec_tp, "-s", ms=3, color="#2a9d8f", label="TP 복구")
ax[1].plot(grid, read_fp, "-^", ms=3, color="#d1495b", label="FP 재유입")
ax[1].axvline(0.62, color="black", ls="--", lw=1)
ax[1].text(0.62, max(read_fp) * 0.8, "0.62:\nTP 29 / FP 194", fontsize=8)
ax[1].set_xlabel("texture threshold"); ax[1].set_ylabel("후보 수")
ax[1].set_title("복구 임계에서 FP 대량 재유입 → rescue 기각")
ax[1].legend()
save(fig, "texture_rescue", "texture_rescue_overlap_readmission")

# ===== FIG 5: beginner summary =====
fig, ax = plt.subplots(figsize=(9, 4.8))
cfgs = ["기존\n(HSV 없음)", "Phase 1C\n(HSV+Dino보정)"]
TP = [648, 622]; FP = [334, 86]; F1 = [0.676, 0.757]
x = np.arange(2); w = 0.25
ax.bar(x - w, TP, w, label="TP (맞게 검출)", color="#2a9d8f", hatch="//")
ax.bar(x, FP, w, label="FP (오검출)", color="#d1495b", hatch="\\\\")
ax.set_xticks(x); ax.set_xticklabels(cfgs); ax.set_ylabel("건수")
for i in range(2):
    ax.text(i - w, TP[i] + 6, TP[i], ha="center", fontsize=9)
    ax.text(i, FP[i] + 6, FP[i], ha="center", fontsize=9)
    ax.text(i + w, 20, f"F1\n{F1[i]}", ha="center", fontsize=9, color="#264653", fontweight="bold")
ax.legend(loc="upper right")
ax.set_title("Phase 1C 요약 — HSV로 오검출 334→86, Dinosaur 색보정으로 TP +6 회복")
fig.text(0.5, -0.02, "HSV는 색이 다른 오검출(다른 인형·갈색박스)을 대량 제거. 일부 진짜 Dinosaur도 색차로 제거되던 것을\n"
         "Dinosaur 렌더에만 +9° 보정해 6개 회복(FP 증가 0). texture는 진짜/오검출이 너무 비슷해 대량 복구엔 못 씀.",
         ha="center", fontsize=8.5, color="gray")
save(fig, "summary", "phase1c_beginner_summary")

# reproduce metrics json for texture
json.dump({"choco_negative_verifier_AUROC_humanGT": round(au_choco, 3),
           "choco_TP": int(cy.sum()), "choco_FP": int(len(cy) - cy.sum()),
           "rescue_AUROC": round(au_res, 3), "rescue_TP_mean": round(float(rt.mean()), 3),
           "rescue_FP_mean": round(float(rf.mean()), 3),
           "rescue_TP_total": len(rt), "rescue_FP_total": len(rf)},
          open(os.path.join(CSVO, "..", "metrics", "texture_reproduction.json"), "w"), indent=2)
print(f"\ntexture repro: choco neg AUROC {au_choco:.3f}, rescue AUROC {au_res:.3f}")
