#!/usr/bin/env python3
"""make_phase21_figures.py — Phase 2.1 FN root-cause audit figures. READ-ONLY.

Consumes precomputed metrics/CSV under outputs/phase21_fn_root_cause_audit/ and
renders PNG(dpi=120)+SVG into .../figures/. Does NOT recompute anything, does NOT
touch operational code / GT / existing outputs. Every source read is guarded; a
missing source skips its figure with a warning instead of crashing.

Style copied from
  _ism_research_2026_07/dinosaur_color_and_texture_research/combined_summary/make_phase1c_figures.py
(Agg backend, Korean font w/ English fallback, unicode_minus=False, save()->png+svg).
"""
import json
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
AUDIT = os.path.join(REPO, "outputs", "phase21_fn_root_cause_audit")
MET = os.path.join(AUDIT, "metrics")
FIG = os.path.join(AUDIT, "figures")
os.makedirs(FIG, exist_ok=True)

# ---------- Korean font (fallback to English via L()) ----------
KO_OK = False
for fp in ["/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
           "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
           "/usr/share/fonts/truetype/nanum/NanumSquareR.ttf"]:
    if os.path.isfile(fp):
        font_manager.fontManager.addfont(fp)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp).get_name()
        KO_OK = True
        FONT_USED = fp
        break
if not KO_OK:
    FONT_USED = "(none — DejaVu Sans, English fallback)"
plt.rcParams["axes.unicode_minus"] = False


def L(ko, en):
    """Korean label if a CJK font loaded, else English fallback."""
    return ko if KO_OK else en


WRITTEN = []
SKIPPED = []


def load_json(name):
    """Guarded JSON load; returns None (and records a skip reason) if missing/bad."""
    p = os.path.join(MET, name)
    if not os.path.isfile(p):
        print(f"[WARN] missing source: {p}")
        return None
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        print(f"[WARN] could not parse {p}: {e}")
        return None


def save(fig, name):
    d = FIG
    os.makedirs(d, exist_ok=True)
    png = os.path.join(d, name + ".png")
    fig.savefig(png, dpi=120, bbox_inches="tight")
    try:
        fig.savefig(os.path.join(d, name + ".svg"), bbox_inches="tight")
    except Exception:
        pass
    plt.close(fig)
    WRITTEN.append(name)
    print("wrote", name)


def skip(name, why):
    print(f"[SKIP] {name}: {why}")
    SKIPPED.append((name, why))


# palette
C_PROP = "#e67e22"    # proposal = orange
C_SEM = "#c0392b"     # gate: semantic  (red)
C_APPE = "#e74c3c"    # gate: appearance(lighter red)
C_HSV = "#f1948a"     # gate: hsv       (lightest red)
C_GATE = "#c0392b"    # gate aggregate
C_SEL = "#2980b9"     # selection = blue
C_TP = "#2a9d8f"      # true positive / good
C_FP = "#d1495b"      # false positive / bad

rc = load_json("root_cause_metrics.json")
partition = load_json("final_causal_partition.json")
cf = load_json("counterfactual_metrics.json")
prop = load_json("proposal_metrics.json")


# ============================================================ FIG 1: funnel
if rc is not None:
    pc = rc.get("primary_causes", {})
    d_prop = pc.get("RC1_no_candidate", 0)
    d_sem = pc.get("RC8_semantic", 0)
    d_appe = pc.get("RC9_appearance", 0)
    d_hsv = pc.get("RC10_hsv", 0)
    d_sel = pc.get("RC11_selection", 0)
    start = 935
    tp = start - d_prop - d_sem - d_appe - d_hsv - d_sel  # = 622
    stages = [
        (L("GT 가시 셀", "GT-visible cells"), start, None, None),
        (L("후보 존재", "has candidate"), start - d_prop, d_prop, C_PROP),
        (L("semantic 통과", "pass semantic"), start - d_prop - d_sem, d_sem, C_SEM),
        (L("appearance 통과", "pass appearance"),
         start - d_prop - d_sem - d_appe, d_appe, C_APPE),
        (L("HSV 통과", "pass HSV"),
         start - d_prop - d_sem - d_appe - d_hsv, d_hsv, C_HSV),
        (L("TP (최종 검출)", "TP (final)"), tp, d_sel, C_SEL),
    ]
    fig, ax = plt.subplots(figsize=(10, 5.2))
    ys = np.arange(len(stages))[::-1]
    maxv = start
    for y, (lab, val, drop, dcol) in zip(ys, stages):
        left = (maxv - val) / 2.0
        ax.barh(y, val, left=left, height=0.62, color="#3a5a78",
                edgecolor="white", zorder=3)
        ax.text(maxv / 2.0, y, f"{lab}\n{val}", ha="center", va="center",
                color="white", fontsize=10, fontweight="bold", zorder=4)
        if drop:
            ax.annotate(f"-{drop}", xy=(maxv * 0.985, y + 0.5), ha="right",
                        va="center", fontsize=10, color=dcol, fontweight="bold")
    ax.set_yticks([])
    ax.set_xlim(0, maxv)
    ax.set_xlabel(L("셀 수", "cell count"))
    ax.set_title(L(f"FN 깔때기: 935 가시 → 622 TP (게이트가 대부분 탈락시킴)",
                   f"FN funnel: 935 visible -> 622 TP (gates drop most)"),
                 fontsize=12, fontweight="bold")
    ax.text(0.01, -0.14, L(
        "주황=proposal(-38) · 빨강계열=gate(semantic-141/appe-106/hsv-23) · 파랑=selection(-5)",
        "orange=proposal(-38) · reds=gate(sem-141/appe-106/hsv-23) · blue=selection(-5)"),
        transform=ax.transAxes, fontsize=8.5, color="gray")
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    save(fig, "fn_funnel")
else:
    skip("fn_funnel", "root_cause_metrics.json missing")


# ============================================================ FIG 2: RC bars
if rc is not None:
    pc = rc.get("primary_causes", {})
    order = [("RC1_no_candidate", C_PROP, L("proposal 누락", "proposal miss")),
             ("RC8_semantic", C_SEM, L("gate·semantic", "gate·semantic")),
             ("RC9_appearance", C_APPE, L("gate·appearance", "gate·appearance")),
             ("RC10_hsv", C_HSV, L("gate·HSV", "gate·HSV")),
             ("RC11_selection", C_SEL, L("selection", "selection"))]
    total = sum(pc.values()) or 313
    keys = [k for k, _, _ in order]
    vals = [pc.get(k, 0) for k in keys]
    cols = [c for _, c, _ in order]
    labs = [f"{k.split('_')[0]}\n{lab}" for (k, _, lab) in order]
    fig, ax = plt.subplots(figsize=(9, 5))
    x = np.arange(len(keys))
    bars = ax.bar(x, vals, color=cols, edgecolor="black", linewidth=0.5)
    for xi, v in zip(x, vals):
        ax.text(xi, v + 1.5, f"{v}\n({100 * v / total:.0f}%)", ha="center",
                va="bottom", fontsize=9, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(labs, fontsize=9)
    ax.set_ylabel(L("FN 개수", "FN count"))
    ax.set_ylim(0, max(vals) * 1.22)
    ax.set_title(L(f"1차 근본원인별 FN 분포 (총 {total}) — gate가 지배",
                   f"Primary root-cause FN counts (total {total}) — gate dominates"),
                 fontsize=12, fontweight="bold")
    from matplotlib.patches import Patch
    leg = [Patch(color=C_PROP, label=L("proposal", "proposal")),
           Patch(color=C_GATE, label=L("gate (semantic/appe/hsv)", "gate")),
           Patch(color=C_SEL, label=L("selection", "selection"))]
    ax.legend(handles=leg, loc="upper right")
    save(fig, "fn_root_cause_bars")
else:
    skip("fn_root_cause_bars", "root_cause_metrics.json missing")


# ============================================================ FIG 3: partition
if partition is not None:
    fp_ = partition.get("final_causal_partition", {})
    prop_n = fp_.get("proposal", 0)
    gate_n = fp_.get("gate", 0)
    sel_n = fp_.get("selection", 0)
    tot = prop_n + gate_n + sel_n or 313
    fig, ax = plt.subplots(figsize=(8.2, 6))
    sizes = [gate_n, sel_n, prop_n]
    cols = [C_GATE, C_SEL, C_PROP]
    labs = [L(f"gate\n{gate_n} ({100*gate_n/tot:.1f}%)", f"gate\n{gate_n} ({100*gate_n/tot:.1f}%)"),
            L(f"selection\n{sel_n} ({100*sel_n/tot:.1f}%)", f"selection\n{sel_n} ({100*sel_n/tot:.1f}%)"),
            L(f"proposal\n{prop_n} ({100*prop_n/tot:.1f}%)", f"proposal\n{prop_n} ({100*prop_n/tot:.1f}%)")]
    wedges, txts, atxts = ax.pie(
        sizes, labels=labs, colors=cols, autopct="", startangle=90,
        counterclock=False, wedgeprops=dict(edgecolor="white", linewidth=2),
        textprops=dict(fontsize=11, fontweight="bold"))
    ax.set_title(L(
        f"FN {tot} 병목: gate {100*gate_n/tot:.0f}% > selection {100*sel_n/tot:.0f}% > proposal {100*prop_n/tot:.0f}%",
        f"FN {tot} bottleneck: gate {100*gate_n/tot:.0f}% > selection {100*sel_n/tot:.0f}% > proposal {100*prop_n/tot:.0f}%"),
        fontsize=13, fontweight="bold", pad=18)
    ax.text(0, -1.32, L("best-ROI oracle 재분류 후 최종 인과 분할",
                        "final causal partition after best-ROI oracle reclass"),
            ha="center", fontsize=9, color="gray")
    save(fig, "final_causal_partition_pie_or_bar")
else:
    skip("final_causal_partition_pie_or_bar", "final_causal_partition.json missing")


# ============================================================ FIG 4: counterfactuals
if cf is not None:
    cfs = cf.get("counterfactuals", {})
    order = ["CF0_actual", "CF4_semantic_bypass", "CF4_appearance_bypass",
             "CF4_hsv_bypass", "CF4_sem+appe_bypass", "CF4_appe+hsv_bypass",
             "CF4_all_gates_bypass"]
    order = [k for k in order if k in cfs]
    short = {"CF0_actual": L("CF0\n실제", "CF0\nactual"),
             "CF4_semantic_bypass": L("sem\nbypass", "sem\nbypass"),
             "CF4_appearance_bypass": L("appe\nbypass", "appe\nbypass"),
             "CF4_hsv_bypass": L("hsv\nbypass", "hsv\nbypass"),
             "CF4_sem+appe_bypass": L("sem+appe", "sem+appe"),
             "CF4_appe+hsv_bypass": L("appe+hsv", "appe+hsv"),
             "CF4_all_gates_bypass": L("all gates\nbypass", "all gates\nbypass")}
    TP = [cfs[k]["TP"] for k in order]
    FP = [cfs[k]["FP"] for k in order]
    FN = [cfs[k]["FN"] for k in order]
    fig, ax = plt.subplots(figsize=(12, 5.6))
    x = np.arange(len(order))
    w = 0.26
    ax.bar(x - w, TP, w, label=L("TP (맞음)", "TP"), color=C_TP)
    ax.bar(x, FP, w, label=L("FP (오검출)", "FP"), color=C_FP)
    ax.bar(x + w, FN, w, label=L("FN (놓침)", "FN"), color="#7f8c8d")
    for xi, (t, f, n) in zip(x, zip(TP, FP, FN)):
        ax.text(xi - w, t + 8, str(t), ha="center", fontsize=7.5)
        ax.text(xi, f + 8, str(f), ha="center", fontsize=7.5,
                color=(C_FP if f > 400 else "black"),
                fontweight=("bold" if f > 400 else "normal"))
        ax.text(xi + w, n + 8, str(n), ha="center", fontsize=7.5)
    # annotate FN_recovered vs FP_readmitted per single-gate bypass
    for xi, k in zip(x, order):
        rec = cfs[k].get("FN_recovered", 0)
        rea = cfs[k].get("FP_readmitted", 0)
        if k == "CF0_actual":
            continue
        ax.annotate(L(f"복구 {rec}\n재유입 {rea}", f"rec {rec}\nread {rea}"),
                    xy=(xi, -0.14), xycoords=("data", "axes fraction"),
                    ha="center", va="top", fontsize=7, color="#555")
    ax.set_xticks(x)
    ax.set_xticklabels([short[k] for k in order], fontsize=9)
    ax.set_ylabel(L("건수", "count"))
    ax.set_ylim(0, max(FP) * 1.14)
    ax.set_title(L(
        "반사실 게이트 우회 — recall↑ 이지만 FP 폭증 (all-gates: TP897 / FP1244)",
        "Counterfactual gate bypass — recall up but FP explodes (all-gates: TP897 / FP1244)"),
        fontsize=12, fontweight="bold")
    ax.legend(loc="upper left")
    ax.margins(y=0.02)
    save(fig, "counterfactual_bars")
else:
    skip("counterfactual_bars", "counterfactual_metrics.json missing")


# ============================================================ FIG 5: best-ROI oracle
if cf is not None and cf.get("oracle"):
    orc = cf["oracle"]
    total_g = orc.get("gate_fn_evaluated", 0)
    sel_causal = orc.get("usable_roi_exists(selection_causal)", 0)
    gate_lim = orc.get("gate_feature_limited", 0)
    fig, ax = plt.subplots(figsize=(8, 5.6))
    sizes = [gate_lim, sel_causal]
    cols = [C_GATE, C_SEL]
    labs = [L(f"gate/feature 한계\n{gate_lim} ({100*gate_lim/total_g:.0f}%)",
              f"gate/feature-limited\n{gate_lim} ({100*gate_lim/total_g:.0f}%)"),
            L(f"selection 인과\n(더 좋은 ROI 존재)\n{sel_causal} ({100*sel_causal/total_g:.0f}%)",
              f"selection-causal\n(usable ROI exists)\n{sel_causal} ({100*sel_causal/total_g:.0f}%)")]
    ax.pie(sizes, labels=labs, colors=cols, startangle=90, counterclock=False,
           wedgeprops=dict(width=0.42, edgecolor="white", linewidth=2),
           textprops=dict(fontsize=10.5, fontweight="bold"))
    ax.text(0, 0, L(f"gate/selection\nFN {total_g}", f"gate/selection\nFN {total_g}"),
            ha="center", va="center", fontsize=12, fontweight="bold")
    ax.set_title(L(f"더 좋은 proposal로도 {gate_lim}/{total_g}는 gate가 막음",
                   f"Even with better proposals, {gate_lim}/{total_g} are blocked by gate"),
                 fontsize=12.5, fontweight="bold", pad=16)
    save(fig, "best_roi_oracle")
else:
    skip("best_roi_oracle", "counterfactual_metrics.json oracle missing")


# ============================================================ FIG 6: proposal recall defs
if prop is not None:
    modes = prop.get("bbox_existence_recall_by_mode", {})
    want = ["shared960", "perprompt960", "perprompt1280", "ext960", "shared960_agn"]
    want = [m for m in want if m in modes]
    vals = [modes[m]["recall"] for m in want]
    hits = [modes[m].get("hit") for m in want]
    poss = [modes[m].get("pos") for m in want]
    cols = ["#95a5a6" if m != "perprompt960" else C_TP for m in want]
    fig, ax = plt.subplots(figsize=(9.5, 5.2))
    x = np.arange(len(want))
    ax.bar(x, vals, color=cols, edgecolor="black", linewidth=0.5)
    for xi, (v, h, p) in zip(x, zip(vals, hits, poss)):
        lab = f"{v:.3f}"
        if h is not None and p is not None:
            lab += f"\n{h}/{p}"
        ax.text(xi, v + 0.012, lab, ha="center", va="bottom", fontsize=9,
                fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(want, fontsize=9, rotation=12)
    ax.set_ylabel(L("BBox 존재 recall", "BBox-existence recall"))
    ax.set_ylim(0, 1.08)
    ax.set_title(L(
        "모드별 BBox-존재 recall — 0.959는 존재 recall이지 IoU가 아님",
        "BBox-existence recall by mode — 0.959 is existence, not IoU"),
        fontsize=12, fontweight="bold")
    ax.text(0.5, -0.24, L(
        "IoU@0.5 / IoU@0.7 recall = 측정불가 (GT bbox 없음). 존재 recall은 셀당 conf≥0.02 후보 1개 이상 기준.",
        "IoU@0.5 / IoU@0.7 recall = not measurable (no GT bbox). Existence = >=1 conf>=0.02 candidate per cell."),
        transform=ax.transAxes, ha="center", fontsize=8.5, color="gray")
    save(fig, "proposal_recall_definitions")
else:
    skip("proposal_recall_definitions", "proposal_metrics.json missing")


# ============================================================ FIG 7: cause by class stacked
if partition is not None and partition.get("by_class"):
    bc = partition["by_class"]
    rows = []
    for cls, d in bc.items():
        p = d.get("proposal", 0)
        g = d.get("gate", 0)
        s = d.get("selection", 0)
        rows.append((cls, p, g, s, p + g + s))
    rows.sort(key=lambda r: -r[4])
    names = [r[0] for r in rows]
    P = np.array([r[1] for r in rows])
    G = np.array([r[2] for r in rows])
    S = np.array([r[3] for r in rows])
    fig, ax = plt.subplots(figsize=(11, 5.6))
    x = np.arange(len(names))
    ax.bar(x, G, color=C_GATE, label=L("gate", "gate"))
    ax.bar(x, S, bottom=G, color=C_SEL, label=L("selection", "selection"))
    ax.bar(x, P, bottom=G + S, color=C_PROP, label=L("proposal", "proposal"))
    for xi, r in zip(x, rows):
        ax.text(xi, r[4] + 0.5, str(r[4]), ha="center", va="bottom",
                fontsize=8.5, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=30, ha="right", fontsize=8.5)
    ax.set_ylabel(L("FN 개수", "FN count"))
    ax.set_title(L(
        "클래스별 FN 인과 구성 — 인형(Dinosaur/Bear)은 gate 지배, choco/milk는 proposal 비중↑",
        "FN cause by class — dolls (Dinosaur/Bear) gate-dominated, choco/milk more proposal"),
        fontsize=11.5, fontweight="bold")
    ax.legend(loc="upper right")
    save(fig, "fn_cause_by_class_stacked")
else:
    skip("fn_cause_by_class_stacked", "final_causal_partition.json by_class missing")


# ============================================================ FIG 8: beginner summary
if partition is not None:
    fp_ = partition.get("final_causal_partition", {})
    prop_n = fp_.get("proposal", 0)
    gate_n = fp_.get("gate", 0)
    sel_n = fp_.get("selection", 0)
    tot = prop_n + gate_n + sel_n or 313
    fig = plt.figure(figsize=(11, 7))
    fig.patch.set_facecolor("white")
    ax = fig.add_axes([0, 0, 1, 1]); ax.axis("off")
    ax.text(0.5, 0.95, L("SAM-6D ISM · FN 근본원인 감사 — 한눈 요약",
                         "SAM-6D ISM · FN Root-Cause Audit — One-Page Summary"),
            ha="center", va="top", fontsize=18, fontweight="bold")
    ax.text(0.5, 0.885, L(f"GT 가시 935셀 · FN 총 {tot}건",
                          f"935 GT-visible cells · {tot} total FN"),
            ha="center", va="top", fontsize=12, color="#555")

    boxes = [
        (0.06, C_PROP, L(f"① proposal 누락 = {prop_n}개 ({100*prop_n/tot:.0f}%)",
                         f"(1) proposal miss = {prop_n} ({100*prop_n/tot:.0f}%)"),
         L("YOLO가 박스를 아예 만들지 못한 FN.",
           "FN where YOLO made no box at all.")),
        (0.06, C_SEL, L(f"② selection 실패 = {sel_n}개 ({100*sel_n/tot:.0f}%)",
                        f"(2) selection miss = {sel_n} ({100*sel_n/tot:.0f}%)"),
         L("좋은 박스가 있었지만 선택 단계가 놓친 FN.",
           "A usable box existed but selection picked the wrong one.")),
        (0.06, C_GATE, L(f"③ gate 차단 = {gate_n}개 ({100*gate_n/tot:.0f}%)",
                         f"(3) gate block = {gate_n} ({100*gate_n/tot:.0f}%)"),
         L("좋은 박스가 있어도(혹은 어떤 박스로도) semantic/appearance/HSV 게이트가 막은 FN.",
           "Even with a good box (or any box), sem/appe/HSV gates rejected it.")),
    ]
    y = 0.80
    for _, col, title, body in boxes:
        ax.add_patch(plt.Rectangle((0.05, y - 0.135), 0.90, 0.125,
                     transform=ax.transAxes, facecolor=col, alpha=0.12,
                     edgecolor=col, linewidth=1.6))
        ax.add_patch(plt.Rectangle((0.05, y - 0.135), 0.012, 0.125,
                     transform=ax.transAxes, facecolor=col, edgecolor=col))
        ax.text(0.08, y - 0.03, title, fontsize=14, fontweight="bold",
                color=col, va="top")
        ax.text(0.08, y - 0.075, body, fontsize=11, color="#222", va="top")
        y -= 0.155

    ax.text(0.5, 0.315, L(
        "결론: 병목은 proposal이 아니라 gate (precision–recall 트레이드오프).",
        "Conclusion: the bottleneck is the gate, not proposal (precision-recall tradeoff)."),
        ha="center", fontsize=14, fontweight="bold", color="#c0392b")
    ax.text(0.5, 0.245, L(
        "게이트를 열면 recall은 오르지만 (all-gates 우회 시 recall 0.96) FP가 폭증한다 (FP 86→1244).",
        "Opening gates raises recall (0.96 with all bypassed) but FP explodes (86->1244)."),
        ha="center", fontsize=11, color="#333")
    ax.text(0.5, 0.185, L(
        "이전의 'proposal 문제' 결론은 shared-pass / IoU 정의 차이에서 온 착시였다.",
        "The earlier 'proposal problem' conclusion came from a shared-pass / IoU definition mismatch."),
        ha="center", fontsize=11, color="#333")
    ax.text(0.5, 0.11, L(
        "존재 recall(perprompt960)=0.959 이므로 박스는 대부분 존재한다 → 남은 문제는 게이트 통과.",
        "BBox-existence recall (perprompt960)=0.959, so boxes mostly exist -> remaining problem is passing the gate."),
        ha="center", fontsize=10.5, color="#666")
    ax.text(0.5, 0.03, L(
        "출처: outputs/phase21_fn_root_cause_audit (root_cause / final_causal_partition / counterfactual / proposal metrics)",
        "Source: outputs/phase21_fn_root_cause_audit metrics"),
        ha="center", fontsize=8, color="gray")
    save(fig, "beginner_summary")
else:
    skip("beginner_summary", "final_causal_partition.json missing")


# ============================================================ report
print("\n" + "=" * 60)
print(f"font used: {FONT_USED}  (korean_labels={KO_OK})")
print(f"output dir: {FIG}")
print(f"generated {len(WRITTEN)} figures (png+svg):")
for n in WRITTEN:
    print(f"  - {n}.png / {n}.svg")
if SKIPPED:
    print(f"skipped {len(SKIPPED)}:")
    for n, why in SKIPPED:
        print(f"  - {n}: {why}")
else:
    print("skipped: none")
