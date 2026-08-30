#!/usr/bin/env python3
"""make_phase22_figures.py — Phase 2.2 "training-free gate" study figures. READ-ONLY.
Reads outputs/phase22_training_free_gate/{metrics,csv} and writes PNG(dpi=120)+SVG
into outputs/phase22_training_free_gate/figures/ (NOT figures_visual/).
Does not modify operational code / GT / existing outputs. Guards every read: skip+warn.
"""
import csv
import json
import os
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
BASE = os.path.join(REPO, "outputs", "phase22_training_free_gate")
METRICS = os.path.join(BASE, "metrics")
CSVD = os.path.join(BASE, "csv")
FIG = os.path.join(BASE, "figures")
os.makedirs(FIG, exist_ok=True)

SKIPPED = []

# ---- Korean font (fallback English) ----
FONT_USED = "matplotlib default (no CJK font found)"
KO_OK = False
for fp in ["/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
           "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"]:
    if os.path.isfile(fp):
        font_manager.fontManager.addfont(fp)
        name = font_manager.FontProperties(fname=fp).get_name()
        plt.rcParams["font.family"] = name
        FONT_USED = f"{name} ({fp})"
        KO_OK = True
        break
plt.rcParams["axes.unicode_minus"] = False


def L(ko, en):
    """Korean if a CJK font is loaded, else English fallback."""
    return ko if KO_OK else en


def save(fig, name):
    fig.savefig(os.path.join(FIG, name + ".png"), dpi=120, bbox_inches="tight")
    try:
        fig.savefig(os.path.join(FIG, name + ".svg"), bbox_inches="tight")
    except Exception as e:  # noqa
        print("  [warn] svg failed for", name, ":", e)
    plt.close(fig)
    print("wrote", name + ".png / .svg")


def load_json(path):
    if not os.path.isfile(path):
        SKIPPED.append(path)
        print("[warn] missing, skip:", path)
        return None
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:  # noqa
        SKIPPED.append(path + f" (read error: {e})")
        print("[warn] read error, skip:", path, e)
        return None


def load_csv(path):
    if not os.path.isfile(path):
        SKIPPED.append(path)
        print("[warn] missing, skip:", path)
        return None
    try:
        with open(path, encoding="utf-8") as f:
            return list(csv.DictReader(f))
    except Exception as e:  # noqa
        SKIPPED.append(path + f" (read error: {e})")
        print("[warn] read error, skip:", path, e)
        return None


# ---- load data ----
tf = load_json(os.path.join(METRICS, "tf_metrics.json"))
comp = load_csv(os.path.join(CSVD, "method_comparison.csv"))
by_class = load_csv(os.path.join(CSVD, "method_by_class.csv"))
by_bag = load_csv(os.path.join(CSVD, "method_by_bag.csv"))  # noqa (available)

BASE_FP = 86
BASE_F1 = 0.7572
BASE_TP = 622

# palette
C_TP = "#2a9d8f"
C_FP = "#d1495b"
C_F1 = "#264653"
C_NEU = "#adb5bd"

# preferred method order (skip baseline duplicate P0/P4 in some charts)
ORDER = ["P0_phase1c", "P1_rerank_top2", "P1_rerank_top3",
         "P7_one_borderline_rescue", "P2_rank_consensus",
         "P3_multiblock_consensus", "P4_view_stability", "P6_pareto_select"]
SHORT = {
    "P0_phase1c": "P0\nphase1c(base)",
    "P1_rerank_top2": "P1\nrerank_top2",
    "P1_rerank_top3": "P1\nrerank_top3",
    "P7_one_borderline_rescue": "P7\n1borderline\nrescue",
    "P2_rank_consensus": "P2\nrank\nconsensus",
    "P3_multiblock_consensus": "P3\nmultiblock\nconsensus",
    "P4_view_stability": "P4\nview_stab",
    "P6_pareto_select": "P6\npareto_sel",
}

# =========================================================================
# FIG 1: grouped bar TP / FP / F1 over all methods
# =========================================================================
if tf is not None:
    m = tf.get("methods", {})
    methods = [k for k in ORDER if k in m]
    if methods:
        TP = [m[k]["TP"] for k in methods]
        FP = [m[k]["FP"] for k in methods]
        F1 = [m[k]["f1"] for k in methods]
        x = np.arange(len(methods))
        w = 0.38

        fig, (axc, axf) = plt.subplots(
            2, 1, figsize=(12, 8), sharex=True,
            gridspec_kw={"height_ratios": [2.2, 1]})

        axc.bar(x - w / 2, TP, w, label=L("TP (맞게 검출)", "TP (correct)"),
                color=C_TP, hatch="//")
        axc.bar(x + w / 2, FP, w, label=L("FP (오검출)", "FP (false pos)"),
                color=C_FP, hatch="\\\\")
        axc.axhline(BASE_FP, color="black", ls="--", lw=1.6,
                    label=L(f"기준 FP={BASE_FP}", f"baseline FP={BASE_FP}"))
        for i in range(len(methods)):
            axc.text(x[i] - w / 2, TP[i] + 4, str(TP[i]), ha="center", fontsize=8)
            axc.text(x[i] + w / 2, FP[i] + 4, str(FP[i]), ha="center", fontsize=8,
                     color=C_FP, fontweight="bold" if FP[i] > BASE_FP else "normal")
        axc.set_ylabel(L("건수 (TP / FP)", "count (TP / FP)"))
        axc.legend(loc="upper left", ncol=3, fontsize=9)
        axc.set_ylim(0, max(TP) * 1.18)
        axc.set_title(L(
            "Training-free 규칙 비교 — FP≤86을 지키면서 TP를 늘리는 방법은 없음\n"
            "(TP를 크게 올린 방법은 모두 FP가 기준선 86을 넘김)",
            "Training-free rules — NO method keeps FP<=86 while raising TP\n"
            "(every TP gain pushes FP above the 86 baseline)"),
            fontsize=12)

        bars = axf.bar(x, F1, 0.55, color=C_F1)
        axf.axhline(BASE_F1, color="black", ls="--", lw=1.6,
                    label=L(f"기준 F1={BASE_F1}", f"baseline F1={BASE_F1}"))
        for i in range(len(methods)):
            up = F1[i] >= BASE_F1
            axf.text(x[i], F1[i] + 0.004, f"{F1[i]:.4f}", ha="center", fontsize=8,
                     color=C_TP if up else C_FP, fontweight="bold")
        axf.set_ylim(min(F1) - 0.02, max(F1 + [BASE_F1]) + 0.02)
        axf.set_ylabel("F1")
        axf.legend(loc="upper left", fontsize=9)
        axf.set_xticks(x)
        axf.set_xticklabels([SHORT[k] for k in methods], fontsize=8)
        fig.text(0.5, -0.01, L(
            "출처: outputs/phase22_training_free_gate/metrics/tf_metrics.json · 사람 GT 935",
            "source: outputs/phase22_training_free_gate/metrics/tf_metrics.json · human GT 935"),
            ha="center", fontsize=8, color="gray")
        save(fig, "method_tp_fp_f1")

# =========================================================================
# FIG 2: TP_recovered vs FP_readmitted scatter with y=x reference
# =========================================================================
if tf is not None:
    m = tf.get("methods", {})
    # exclude no-op / negative-recovery entries for clarity but keep informative ones
    methods = [k for k in ORDER if k in m and (m[k]["TP_recovered"] != 0 or m[k]["FP_readmitted"] != 0)]
    if methods:
        xr = np.array([m[k]["TP_recovered"] for k in methods], float)
        yr = np.array([m[k]["FP_readmitted"] for k in methods], float)

        fig, ax = plt.subplots(figsize=(9, 8))
        lim = max(xr.max(), yr.max()) * 1.15 + 5
        lo = min(xr.min(), yr.min(), 0) - 5
        ax.plot([lo, lim], [lo, lim], ":", color="gray", lw=1.5,
                label=L("y=x (위쪽=나쁜 거래: 되살린 TP보다 재유입 FP가 많음)",
                        "y=x (above line = bad trade: more FP than TP)"))
        ax.fill_between([lo, lim], [lo, lim], lim, color=C_FP, alpha=0.06)
        ax.fill_between([lo, lim], lo, [lo, lim], color=C_TP, alpha=0.06)

        colors = [C_FP if y > x else C_TP for x, y in zip(xr, yr)]
        ax.scatter(xr, yr, s=140, c=colors, edgecolor="black", zorder=3)
        ann = {
            "P7_one_borderline_rescue": "P7 one_borderline_rescue (+28 / +14)",
            "P2_rank_consensus": "P2 rank_consensus (+78 / +143)",
            "P3_multiblock_consensus": "P3 multiblock (+73 / +133)",
            "P1_rerank_top3": "P1 rerank_top3 (+5 / +6)",
            "P1_rerank_top2": "P1 rerank_top2 (-21 / +1)",
            "P6_pareto_select": "P6 pareto_select (+5 / +6)",
        }
        for k, xx, yy in zip(methods, xr, yr):
            label = ann.get(k, k)
            ax.annotate(label, (xx, yy), xytext=(8, 8),
                        textcoords="offset points", fontsize=9, fontweight="bold")
        ax.set_xlabel(L("되살린 TP 개수 (TP_recovered)", "TP recovered"))
        ax.set_ylabel(L("재유입된 FP 개수 (FP_readmitted)", "FP readmitted"))
        ax.set_xlim(lo, lim)
        ax.set_ylim(lo, lim)
        ax.legend(loc="upper left", fontsize=9)
        ax.set_title(L(
            "TP 복구 vs FP 재유입 — 큰 복구(P2/P3)는 FP가 ~2배로 폭증\n"
            "가장 우호적 거래는 P7(+28/+14), rerank_top3(+5/+6)뿐",
            "TP recovered vs FP readmitted — large recovery (P2/P3) doubles FP\n"
            "best trades: P7(+28/+14), rerank_top3(+5/+6)"),
            fontsize=12)
        save(fig, "tp_recovered_vs_fp_readmitted")

# =========================================================================
# FIG 3: recovered root-cause distribution (stacked bar per method)
# =========================================================================
if tf is not None:
    dist = tf.get("recovered_rc_distribution", {})
    if dist:
        methods = [k for k in ORDER if k in dist]
        rcs = ["RC8_semantic", "RC9_appearance", "RC10_hsv", "RC11_selection"]
        rc_color = {"RC8_semantic": "#e9c46a", "RC9_appearance": "#2a9d8f",
                    "RC10_hsv": "#8ecae6", "RC11_selection": "#f4a261"}
        rc_label = {
            "RC8_semantic": L("RC8 의미(semantic) 경계", "RC8 semantic borderline"),
            "RC9_appearance": L("RC9 외형(appearance) 경계", "RC9 appearance borderline"),
            "RC10_hsv": L("RC10 색(hsv) 경계", "RC10 hsv borderline"),
            "RC11_selection": L("RC11 선택(selection) 오류", "RC11 selection error"),
        }
        x = np.arange(len(methods))
        fig, ax = plt.subplots(figsize=(11, 6))
        bottom = np.zeros(len(methods))
        for rc in rcs:
            vals = np.array([dist[k].get(rc, 0) for k in methods], float)
            if vals.sum() == 0:
                continue
            ax.bar(x, vals, 0.6, bottom=bottom, label=rc_label[rc],
                   color=rc_color[rc], edgecolor="white")
            for i, v in enumerate(vals):
                if v > 0:
                    ax.text(x[i], bottom[i] + v / 2, int(v), ha="center",
                            va="center", fontsize=8, fontweight="bold")
            bottom += vals
        ax.set_xticks(x)
        ax.set_xticklabels([SHORT[k] for k in methods], fontsize=8)
        ax.set_ylabel(L("되살린 FN 개수 (근본원인별)", "recovered FN (by root cause)"))
        ax.legend(fontsize=9)
        ax.set_title(L(
            "복구된 FN의 근본원인 분포 — P7은 외형/의미 경계건을 되살림,\n"
            "rerank 계열은 오직 RC11(선택) FN만 안전하게 복구",
            "Recovered FN by root cause — P7 rescues appearance/semantic borderline,\n"
            "rerank family recovers only RC11 (selection) FN"),
            fontsize=12)
        save(fig, "recovered_rc_distribution")

# =========================================================================
# FIG 4: per-class ΔTP / ΔFP vs P0 for P7 and rerank_top3
# =========================================================================
if by_class is not None:
    # index: (method, class) -> row
    idx = {(r["method"], r["class"]): r for r in by_class}
    classes = []
    for r in by_class:
        if r["method"] == "P0_phase1c" and r["class"] not in classes:
            classes.append(r["class"])
    targets = ["P7_one_borderline_rescue", "P1_rerank_top3"]
    tlabel = {"P7_one_borderline_rescue": "P7 one_borderline_rescue",
              "P1_rerank_top3": "P1 rerank_top3"}
    have = [t for t in targets if any((t, c) in idx for c in classes)]
    if classes and have:
        fig, axes = plt.subplots(len(have), 1, figsize=(12, 4.2 * len(have)),
                                 squeeze=False)
        x = np.arange(len(classes))
        w = 0.38
        for ai, t in enumerate(have):
            ax = axes[ai][0]
            dTP, dFP = [], []
            for c in classes:
                b = idx.get(("P0_phase1c", c))
                mm = idx.get((t, c))
                if b is None or mm is None:
                    dTP.append(0); dFP.append(0); continue
                dTP.append(int(mm["TP"]) - int(b["TP"]))
                dFP.append(int(mm["FP"]) - int(b["FP"]))
            ax.axhline(0, color="black", lw=0.8)
            ax.bar(x - w / 2, dTP, w, label=L("ΔTP (많을수록 좋음)", "ΔTP (higher better)"),
                   color=C_TP, hatch="//")
            ax.bar(x + w / 2, dFP, w, label=L("ΔFP (적을수록 좋음)", "ΔFP (lower better)"),
                   color=C_FP, hatch="\\\\")
            for i in range(len(classes)):
                if dTP[i]:
                    ax.text(x[i] - w / 2, dTP[i] + (0.3 if dTP[i] >= 0 else -0.6),
                            f"{dTP[i]:+d}", ha="center", fontsize=8)
                if dFP[i]:
                    ax.text(x[i] + w / 2, dFP[i] + (0.3 if dFP[i] >= 0 else -0.6),
                            f"{dFP[i]:+d}", ha="center", fontsize=8, fontweight="bold")
            # highlight choco for P7
            if t == "P7_one_borderline_rescue" and "choco_hazelnut_high" in classes:
                ci = classes.index("choco_hazelnut_high")
                ax.axvspan(ci - 0.5, ci + 0.5, color="#ffd166", alpha=0.25, zorder=0)
                ax.annotate(
                    L("choco_hazelnut_high: FP +12 (P7 새 FP 14개 중 12개)",
                      "choco_hazelnut_high: FP +12 (12 of P7's 14 new FP)"),
                    (ci + w / 2, dFP[ci]), xytext=(0, 24),
                    textcoords="offset points", ha="center", fontsize=9,
                    fontweight="bold", color="#b8860b",
                    arrowprops=dict(arrowstyle="->", color="#b8860b"))
            ax.set_xticks(x)
            ax.set_xticklabels(classes, rotation=30, ha="right", fontsize=8)
            ax.set_ylabel(L("P0 대비 변화량", "Δ vs P0"))
            ax.legend(loc="upper left", fontsize=9)
            ax.set_title(tlabel[t] + L("  — 클래스별 P0 대비 ΔTP / ΔFP",
                                       "  — per-class ΔTP / ΔFP vs P0"))
        fig.suptitle(L(
            "클래스별 이득/손해 — P7의 추가 FP는 choco_hazelnut_high에 집중(14개 중 12개)",
            "Per-class gain/cost — P7's extra FP concentrates in choco_hazelnut_high (12 of 14)"),
            fontsize=12, y=1.005)
        fig.tight_layout()
        save(fig, "method_by_class_delta")

# =========================================================================
# FIG 5: beginner one-page Korean summary
# =========================================================================
fig = plt.figure(figsize=(11, 8.5))
fig.patch.set_facecolor("white")
ax = fig.add_axes([0, 0, 1, 1]); ax.axis("off")

title = L("Phase 2.2 — Training-free 게이트 연구 요약",
          "Phase 2.2 — Training-free gate study summary")
ax.text(0.5, 0.955, title, ha="center", fontsize=19, fontweight="bold", color=C_F1)

concl = L(
    "결론: training-free 규칙만으로 FP≤86을 지키면서 TP를 늘리는 방법은 없음",
    "Conclusion: no training-free rule keeps FP<=86 while raising TP")
ax.text(0.5, 0.90, concl, ha="center", fontsize=13.5, color=C_FP, fontweight="bold")

lines = [
    L("• 최선(권장): one_borderline_rescue — TP +28 / FP +14, F1 0.7572 → 0.7715",
      "• Best: one_borderline_rescue — TP +28 / FP +14, F1 0.7572 → 0.7715"),
    L("    - 10개 중 8개 class에서 개선, 새 FP는 대부분 choco_hazelnut_high(14개 중 12개)",
      "    - improves 8 of 10 classes; new FP mostly choco_hazelnut_high (12 of 14)"),
    L("• 안전한 소폭 복구: rerank_top3 — selection(RC11) FN 5개만 복구 (TP +5 / FP +6)",
      "• Safe small win: rerank_top3 — recovers 5 selection(RC11) FN (TP +5 / FP +6)"),
    L("• 큰 복구(rank_consensus / multiblock_consensus)는 FP 폭증 → 기각",
      "• Large recovery (rank/multiblock consensus) explodes FP -> rejected"),
    L("    - rank_consensus: TP +78 / FP +143,  multiblock: TP +73 / FP +133",
      "    - rank_consensus: TP +78 / FP +143,  multiblock: TP +73 / FP +133"),
    L("• 남은 222개의 feature-limited FN은 training-free로 넘지 못함",
      "• The remaining 222 feature-limited FN cannot be crossed training-free"),
    L("    → 더 강한 feature(학습형 검증기 등)가 필요",
      "    → a stronger feature (e.g. learned verifier) is required"),
]
y = 0.83
for ln in lines:
    strong = ln.strip().startswith("•")
    ax.text(0.07, y, ln, ha="left", fontsize=12,
            color="#222" if strong else "#555",
            fontweight="bold" if strong else "normal")
    y -= 0.052

# small embedded bar: baseline vs P7 vs rerank_top3
axb = fig.add_axes([0.10, 0.10, 0.52, 0.30])
labels = [L("기존\nP0", "P0"), "P7\nrescue", "P1\nrerank_top3"]
tpv = [BASE_TP, 650, 627]
fpv = [BASE_FP, 100, 92]
xx = np.arange(3); w = 0.32
axb.bar(xx - w / 2, tpv, w, color=C_TP, hatch="//", label="TP")
axb.bar(xx + w / 2, fpv, w, color=C_FP, hatch="\\\\", label="FP")
axb.axhline(BASE_FP, color="black", ls="--", lw=1.2)
axb.text(2.35, BASE_FP + 4, L("기준 FP=86", "base FP=86"), fontsize=8, color="black")
for i in range(3):
    axb.text(xx[i] - w / 2, tpv[i] + 6, tpv[i], ha="center", fontsize=8)
    axb.text(xx[i] + w / 2, fpv[i] + 6, fpv[i], ha="center", fontsize=8, color=C_FP)
axb.set_xticks(xx); axb.set_xticklabels(labels, fontsize=9)
axb.set_ylim(0, max(tpv) * 1.2)
axb.legend(fontsize=9, loc="upper left")
axb.set_title(L("TP는 오르지만 FP도 기준선(86)을 넘음",
                "TP rises but FP crosses the 86 baseline"), fontsize=10)

# ops note box
ops = L(
    "운영: 기본 OFF로 구현. 활성화는 별도 승인 후에만.",
    "Ops: implement default OFF; enable only with separate approval.")
ax.text(0.66, 0.30, ops, ha="left", fontsize=12.5, color=C_F1, fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.6", fc="#f1faee", ec=C_F1, lw=1.5))
ax.text(0.5, 0.03, L(
    "출처: outputs/phase22_training_free_gate/ (metrics/tf_metrics.json, csv/*) · 사람 GT 935",
    "source: outputs/phase22_training_free_gate/ (metrics + csv) · human GT 935"),
    ha="center", fontsize=8.5, color="gray")
save(fig, "beginner_summary")

# ---- report ----
print("\n=== DONE ===")
print("figures dir:", FIG)
print("generated:", sorted(f for f in os.listdir(FIG) if f.endswith((".png", ".svg"))))
print("font used:", FONT_USED)
print("skipped (missing/errored):", SKIPPED if SKIPPED else "none")
