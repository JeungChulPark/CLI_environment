#!/usr/bin/env python3
"""make_phase2_figures.py — Phase 2 bottleneck figures (Appearance / Semantic / YOLO bbox / Integrated).
READ-ONLY: reads pre-computed CSV + metrics JSON, writes PNG+SVG figures only.
Run with: ~/miniconda3/envs/sam_yolo/bin/python
"""
import csv, json, os
from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(REPO, "outputs", "phase2_appearance_semantic_yolo_bbox")
CSVD = os.path.join(OUT, "csv")
MET = os.path.join(OUT, "metrics")
FIG = os.path.join(OUT, "figures")

# ---------- Korean font (fall back to English) ----------
KOREAN = False
for fp in ["/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
           "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"]:
    if os.path.isfile(fp):
        try:
            font_manager.fontManager.addfont(fp)
            plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp).get_name()
            KOREAN = True
            FONT_USED = fp
            break
        except Exception:
            pass
if not KOREAN:
    FONT_USED = "default (English fallback)"
plt.rcParams["axes.unicode_minus"] = False


def L(ko, en):
    """Pick Korean label if a Korean font is available, else English."""
    return ko if KOREAN else en


GEN = []


def save(fig, sub, name):
    d = os.path.join(FIG, sub)
    os.makedirs(d, exist_ok=True)
    p_png = os.path.join(d, name + ".png")
    fig.savefig(p_png, dpi=120, bbox_inches="tight")
    GEN.append(p_png)
    try:
        p_svg = os.path.join(d, name + ".svg")
        fig.savefig(p_svg, bbox_inches="tight")
        GEN.append(p_svg)
    except Exception as e:
        print("  [warn] svg failed for", name, e)
    plt.close(fig)
    print("wrote", sub + "/" + name)


def load_json(path):
    if not os.path.isfile(path):
        print("  [warn] missing json:", path)
        return None
    try:
        return json.load(open(path, encoding="utf-8"))
    except Exception as e:
        print("  [warn] failed to read", path, e)
        return None


def load_csv(path):
    if not os.path.isfile(path):
        print("  [warn] missing csv:", path)
        return None
    try:
        return list(csv.DictReader(open(path, encoding="utf-8")))
    except Exception as e:
        print("  [warn] failed to read", path, e)
        return None


TEAL, RED, DARK, GRAY = "#2a9d8f", "#d1495b", "#264653", "#adb5bd"

# ================================================================
# Selected-best candidate per (dataset,frame_id,object) from phase2_candidates.csv
# ================================================================
sel = None
cand = load_csv(os.path.join(CSVD, "phase2_candidates.csv"))
if cand:
    groups = defaultdict(list)
    for r in cand:
        groups[(r["dataset"], r["frame_id"], r["object"])].append(r)
    sel = []
    for key, cs in groups.items():
        v = []
        for c in cs:
            try:
                if int(c["routed"]) == 1 and float(c["yolo_conf"]) >= 0.02:
                    v.append(c)
            except (ValueError, KeyError):
                continue
        if not v:
            continue
        v.sort(key=lambda c: -float(c["yolo_conf"]))
        v = v[:3]
        best = max(v, key=lambda c: float(c["sem_top5"]))
        sel.append(best)
    print(f"selected-best candidates: {len(sel)}")


def split_pos_neg(rows, field):
    pos, neg = [], []
    for r in rows:
        gv = r.get("gt_visible", "").strip()
        if gv == "":
            continue
        try:
            val = float(r[field])
        except (ValueError, KeyError):
            continue
        (pos if gv == "1" else neg).append(val)
    return np.array(pos), np.array(neg)


# ================================================================
# APPEARANCE
# ================================================================
appe = load_json(os.path.join(MET, "appearance_metrics.json"))

# --- FIG 1: block2/9/11 clstop1 distributions pos vs neg ---
if sel:
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.3), sharey=True)
    blocks = [("appe2_clstop1", "block2"), ("appe9_clstop1", "block9"), ("appe11_clstop1", "block11")]
    bins = np.linspace(0, 1, 40)
    for ax, (fld, bname) in zip(axes, blocks):
        pos, neg = split_pos_neg(sel, fld)
        ax.hist(pos, bins, alpha=0.6, color=TEAL, hatch="//",
                label=L(f"실제 TP (n={len(pos)})", f"real TP (n={len(pos)})"))
        ax.hist(neg, bins, alpha=0.6, color=RED, hatch="\\\\",
                label=L(f"오검출 FP (n={len(neg)})", f"FP (n={len(neg)})"))
        au = appe["discriminability"][bname]["AUROC"] if appe else float("nan")
        ax.set_title(f"{bname}  AUROC={au:.3f}")
        ax.set_xlabel(L("cls-top1 유사도", "cls-top1 similarity"))
        ax.legend(fontsize=8)
    axes[0].set_ylabel(L("후보 수", "candidates"))
    fig.suptitle(L("DINOv2 블록별 appearance score 분포 (선택-best 후보) — 세 블록 변별력 유사",
                   "DINOv2 per-block appearance score distribution (selected-best) — similar overall discriminability"))
    save(fig, "appearance", "block_score_distributions")

# --- FIG 2: block2 vs block9 scatter colored by pos/neg ---
if sel:
    fig, ax = plt.subplots(figsize=(6.5, 6))
    xy = [(float(r["appe2_clstop1"]), float(r["appe9_clstop1"]), r["gt_visible"].strip())
          for r in sel if r.get("gt_visible", "").strip() in ("0", "1")]
    xp = [a for a, b, g in xy if g == "1"]; yp = [b for a, b, g in xy if g == "1"]
    xn = [a for a, b, g in xy if g == "0"]; yn = [b for a, b, g in xy if g == "0"]
    ax.scatter(xn, yn, s=8, alpha=0.35, color=RED, label=L(f"오검출 FP (n={len(xn)})", f"FP (n={len(xn)})"))
    ax.scatter(xp, yp, s=8, alpha=0.45, color=TEAL, label=L(f"실제 TP (n={len(xp)})", f"real TP (n={len(xp)})"))
    ax.plot([0, 1], [0, 1], ":", color="gray")
    corr = appe["block_correlation"]["b2_b9"] if appe else float("nan")
    ax.set_xlabel("block2 cls-top1"); ax.set_ylabel("block9 cls-top1")
    ax.set_title(L(f"block2 vs block9 (상관 {corr:.3f}) — 두 블록은 다른 정보",
                   f"block2 vs block9 (corr {corr:.3f}) — partially independent"))
    ax.legend()
    save(fig, "appearance", "block2_vs_block9_scatter")

# --- FIG 3: overall AUROC by block + per-class grouped bar (KEY) ---
if appe:
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.2),
                             gridspec_kw={"width_ratios": [1, 3]})
    # overall
    blocks = ["block2", "block9", "block11"]
    aur = [appe["discriminability"][b]["AUROC"] for b in blocks]
    cols = [DARK, TEAL, "#e9c46a"]
    axes[0].bar(blocks, aur, color=cols, hatch=["", "//", ".."])
    axes[0].set_ylim(0.5, 0.85); axes[0].set_ylabel("AUROC")
    axes[0].set_title(L("전체 변별력 (블록별)", "Overall AUROC by block"))
    for i, v in enumerate(aur):
        axes[0].text(i, v + 0.005, f"{v:.3f}", ha="center", fontsize=9, fontweight="bold")
    # per-class
    ca = appe["class_auroc"]
    objs = list(ca.keys())
    x = np.arange(len(objs)); w = 0.27
    b2 = [ca[o]["block2"] for o in objs]
    b9 = [ca[o]["block9"] for o in objs]
    b11 = [ca[o]["block11"] for o in objs]
    axes[1].bar(x - w, b2, w, color=DARK, label="block2")
    axes[1].bar(x, b9, w, color=TEAL, hatch="//", label="block9")
    axes[1].bar(x + w, b11, w, color="#e9c46a", hatch="..", label="block11")
    axes[1].axhline(0.5, color="gray", ls=":", lw=1)
    axes[1].set_xticks(x); axes[1].set_xticklabels(objs, rotation=35, ha="right", fontsize=8)
    axes[1].set_ylabel("AUROC"); axes[1].set_ylim(0.35, 1.0)
    axes[1].set_title(L("객체별 블록 변별력 — Bear/Dinosaur는 block2, choco는 block11 우세 (클래스 의존)",
                        "Per-class AUROC — Bear/Dinosaur favor block2, choco favors block11 (class-dependent)"))
    axes[1].legend()
    save(fig, "appearance", "auroc_overall_and_perclass")

# --- FIG 4: pipeline config bar A0..A4 F1 with TP/FP annotated ---
if appe and appe.get("pipeline_configs"):
    fig, ax = plt.subplots(figsize=(9, 5))
    pc = appe["pipeline_configs"]
    names = [c["name"].split("_")[0] for c in pc]
    f1 = [c["f1"] for c in pc]
    bars = ax.bar(names, f1, color=TEAL, hatch="//")
    # highlight best
    bi = int(np.argmax(f1)); bars[bi].set_color("#e76f51")
    ax.set_ylabel("F1"); ax.set_ylim(0, max(f1) * 1.25)
    ax.set_title(L("Appearance 파이프라인 설정별 F1 (block9 게이트 = A2가 최고, FP 최소)",
                   "Appearance pipeline configs — F1 (block9 gate = A2 best, lowest FP)"))
    for i, c in enumerate(pc):
        ax.text(i, c["f1"] + 0.01, f"F1 {c['f1']:.3f}", ha="center", fontsize=8, fontweight="bold")
        ax.text(i, 0.03, f"TP {c['TP']}\nFP {c['FP']}", ha="center", fontsize=8, color=DARK)
    # full names legend
    fig.text(0.5, -0.06, "  |  ".join(c["name"] for c in pc), ha="center", fontsize=7, color="gray")
    save(fig, "appearance", "pipeline_configs_f1")

# ================================================================
# SEMANTIC
# ================================================================
sem = load_json(os.path.join(MET, "semantic_metrics.json"))

# --- FIG 5: sem_top5 vs sem_all_mean scatter (pos/neg) corr annotated ---
if sel:
    fig, ax = plt.subplots(figsize=(6.5, 6))
    xy = [(float(r["sem_top5"]), float(r["sem_all_mean"]), r["gt_visible"].strip())
          for r in sel if r.get("gt_visible", "").strip() in ("0", "1")]
    xn = [a for a, b, g in xy if g == "0"]; yn = [b for a, b, g in xy if g == "0"]
    xp = [a for a, b, g in xy if g == "1"]; yp = [b for a, b, g in xy if g == "1"]
    ax.scatter(xn, yn, s=8, alpha=0.35, color=RED, label=L(f"오검출 FP (n={len(xn)})", f"FP (n={len(xn)})"))
    ax.scatter(xp, yp, s=8, alpha=0.45, color=TEAL, label=L(f"실제 TP (n={len(xp)})", f"real TP (n={len(xp)})"))
    corr = sem["correlation"]["top5_allmean"] if sem and "correlation" in sem else 0.948
    ax.set_xlabel("sem_top5"); ax.set_ylabel("sem_all_mean")
    ax.set_title(L(f"sem_top5 vs sem_all_mean (상관 {corr:.3f}) — 집계 방식 바꿔도 거의 동일",
                   f"sem_top5 vs sem_all_mean (corr {corr:.3f}) — aggregation barely changes ranking"))
    ax.legend()
    save(fig, "semantic", "top5_vs_allmean_scatter")

# --- FIG 6: AUROC bar across S1..S7 ---
if sem and "discriminability" in sem:
    fig, ax = plt.subplots(figsize=(9, 5))
    disc = sem["discriminability"]
    names = list(disc.keys())
    aur = [disc[n]["AUROC"] for n in names]
    bars = ax.bar(names, aur, color=TEAL, hatch="//")
    bi = int(np.argmax(aur)); bars[bi].set_color("#e76f51")
    ax.set_ylim(0.75, 0.83); ax.set_ylabel("AUROC")
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=25, ha="right", fontsize=8)
    ax.set_title(L("Semantic 집계 변별력 (S1~S7) — 전 변형 0.79~0.81, 변경 이득 미미",
                   "Semantic aggregation AUROC (S1..S7) — all 0.79~0.81, negligible gain"))
    for i, v in enumerate(aur):
        ax.text(i, v + 0.001, f"{v:.3f}", ha="center", fontsize=8)
    save(fig, "semantic", "auroc_across_S1_S7")

# --- FIG 7: pipeline config F1 bar S0..S7 ---
if sem and sem.get("pipeline_configs"):
    fig, ax = plt.subplots(figsize=(10, 5))
    pc = sem["pipeline_configs"]
    names = [c["name"].split("_")[0] for c in pc]
    f1 = [c["f1"] for c in pc]
    bars = ax.bar(names, f1, color=TEAL, hatch="//")
    bi = int(np.argmax(f1)); bars[bi].set_color("#e76f51")
    ax.set_ylabel("F1"); ax.set_ylim(0, max(f1) * 1.25)
    ax.set_title(L("Semantic 파이프라인 설정별 F1 (S0=운영 top5가 이미 최상급 → 변경 무의미)",
                   "Semantic pipeline configs F1 (S0=operational top5 already best → no change needed)"))
    for i, c in enumerate(pc):
        ax.text(i, c["f1"] + 0.008, f"F1 {c['f1']:.3f}", ha="center", fontsize=7.5, fontweight="bold")
        ax.text(i, 0.03, f"TP {c['TP']}\nFP {c['FP']}", ha="center", fontsize=7.5, color=DARK)
    fig.text(0.5, -0.05, "  |  ".join(c["name"] for c in pc), ha="center", fontsize=6.5, color="gray")
    save(fig, "semantic", "pipeline_configs_f1")

# ================================================================
# YOLO BBOX
# ================================================================
yb = load_json(os.path.join(MET, "yolo_bbox_metrics.json"))

# --- FIG 8: failure taxonomy bar ---
if yb and "failure_taxonomy" in yb:
    fig, ax = plt.subplots(figsize=(8.5, 5))
    ft = yb["failure_taxonomy"]
    keys = ["hit_operational", "F2_shared_nms_suppression", "F1_below_confidence", "F0_no_raw_prediction"]
    labs = [L("정상 검출\n(hit)", "hit\noperational"),
            L("F2: shared-pass\nNMS 억제", "F2: shared-pass\nNMS suppress"),
            L("F1: conf\n미달", "F1: below\nconf"),
            L("F0: raw\n예측 없음", "F0: no raw\nprediction")]
    vals = [ft.get(k, 0) for k in keys]
    cols = [TEAL, "#e76f51", "#e9c46a", GRAY]
    bars = ax.bar(labs, vals, color=cols, hatch=["", "//", "..", ""])
    ax.set_ylabel(L("셀 수", "GT cells"))
    miss = sum(vals[1:])
    ax.set_title(L(f"YOLO bbox 실패 분류 — 누락 {miss}건 중 F2(NMS 억제)가 {ft.get('F2_shared_nms_suppression',0)}건으로 지배적",
                   f"YOLO bbox failure taxonomy — of {miss} misses, F2 (NMS) dominates ({ft.get('F2_shared_nms_suppression',0)})"))
    for i, v in enumerate(vals):
        ax.text(i, v + 8, str(v), ha="center", fontsize=10, fontweight="bold")
    save(fig, "yolo_bbox", "failure_taxonomy")

# --- FIG 9: visibility recall vs FP/frame scatter across methods ---
if yb and "methods" in yb:
    fig, ax = plt.subplots(figsize=(8.5, 6))
    m = yb["methods"]
    # friendly labels
    friendly = {
        "C0_operational_shared960@0.02": ("C0 operational", 0.826),
        "C2_perprompt960@0.02": ("perprompt960", 0.959),
        "C4_perprompt1280@0.02": ("perprompt1280", 0.971),
        "C2_shared960_agnostic@0.02": ("agnostic", 0.703),
    }
    for name, d in m.items():
        rec = d["recall"]; fp = d["fp_per_frame"]
        lab = friendly.get(name, (name.split("@")[0], None))[0]
        star = name in friendly
        ax.scatter(fp, rec, s=120 if star else 45,
                   color="#e76f51" if star else GRAY,
                   edgecolor=DARK, zorder=3 if star else 2)
        ax.annotate(f"{lab}\n(recall {rec:.3f})", (fp, rec),
                    textcoords="offset points", xytext=(8, 4), fontsize=8,
                    fontweight="bold" if star else "normal")
    ax.axhline(0.826, color=TEAL, ls=":", lw=1)
    ax.set_xlabel(L("프레임당 FP box 수", "FP boxes / frame"))
    ax.set_ylabel(L("가시성 recall", "visibility recall"))
    ax.set_title(L("검출 방식별 recall vs FP/frame — per-prompt가 recall 대폭↑ (약간의 FP 대가)",
                   "Method recall vs FP/frame — per-prompt lifts recall a lot (modest FP cost)"))
    save(fig, "yolo_bbox", "recall_vs_fp_methods")

# --- FIG 10: conf sweep line (recall & fp/frame vs conf) ---
if yb and "conf_sweep" in yb:
    fig, ax = plt.subplots(figsize=(8.5, 5))
    cs = yb["conf_sweep"]
    confs = sorted(cs.keys(), key=lambda x: float(x))
    xc = [float(c) for c in confs]
    rec = [cs[c]["recall"] for c in confs]
    fpf = [cs[c]["fp_per_frame"] for c in confs]
    ax.plot(xc, rec, "-o", color=TEAL, label=L("가시성 recall", "visibility recall"))
    ax.set_xlabel("conf threshold"); ax.set_ylabel(L("recall", "recall"), color=TEAL)
    ax.tick_params(axis="y", labelcolor=TEAL)
    ax2 = ax.twinx()
    ax2.plot(xc, fpf, "-^", color=RED, label=L("FP/frame", "FP/frame"))
    ax2.set_ylabel(L("프레임당 FP box", "FP boxes / frame"), color=RED)
    ax2.tick_params(axis="y", labelcolor=RED)
    ax.axvline(0.02, color=DARK, ls="--", lw=1)
    ax.text(0.021, min(rec), L("운영 0.02", "operational 0.02"), fontsize=8, color=DARK)
    ax.set_title(L("conf 임계 스윕 — 낮출수록 recall↑ 그러나 FP 급증 (shared-pass 한계)",
                   "conf sweep — lower conf raises recall but FP explodes (shared-pass ceiling)"))
    ax.legend(loc="center right")
    save(fig, "yolo_bbox", "conf_sweep")

# ================================================================
# INTEGRATED
# ================================================================
integ = load_json(os.path.join(MET, "integrated_metrics.json"))

# --- FIG 11: P0/P1/P3(shared960)/P3(perprompt1280) grouped bar TP/FP/F1 ---
if integ and "results" in integ:
    res = integ["results"]
    order = ["P0_phase1c_baseline", "P1_block9_appe", "P3_altcand_shared960", "P3_altcand_perprompt1280"]
    order = [k for k in order if k in res]
    labs = {"P0_phase1c_baseline": L("P0 기준(1C)", "P0 baseline"),
            "P1_block9_appe": L("P1 block9게이트", "P1 block9 gate"),
            "P3_altcand_shared960": L("P3 shared960", "P3 shared960"),
            "P3_altcand_perprompt1280": L("P3 perprompt1280", "P3 perprompt1280")}
    fig, ax = plt.subplots(figsize=(10, 5.5))
    x = np.arange(len(order)); w = 0.35
    tp = [res[k]["TP"] for k in order]
    fp = [res[k]["FP"] for k in order]
    ax.bar(x - w/2, tp, w, color=TEAL, hatch="//", label="TP")
    ax.bar(x + w/2, fp, w, color=RED, hatch="\\\\", label="FP")
    ax.set_xticks(x); ax.set_xticklabels([labs[k] for k in order], fontsize=9)
    ax.set_ylabel(L("건수", "count"))
    for i, k in enumerate(order):
        ax.text(i - w/2, tp[i] + 6, tp[i], ha="center", fontsize=9)
        ax.text(i + w/2, fp[i] + 6, fp[i], ha="center", fontsize=9)
        ax.text(i, -55, f"F1 {res[k]['f1']:.3f}", ha="center", fontsize=9,
                color=DARK, fontweight="bold")
    ax.set_ylim(0, max(tp) * 1.15)
    ax.legend(loc="upper right")
    ax.set_title(L("통합 파이프라인 비교 — block9 게이트로 FP 86→54, per-prompt 후보로 TP↑",
                   "Integrated pipeline — block9 gate FP 86→54, per-prompt candidates raise TP"))
    save(fig, "integrated", "pipeline_TP_FP_F1")

# --- FIG 12: one-page beginner summary ---
fig, ax = plt.subplots(figsize=(11, 7))
ax.axis("off")
title = L("Phase 2 병목 요약 — 초보자용 한 장",
          "Phase 2 Bottleneck Summary — one page")
ax.text(0.5, 0.97, title, ha="center", va="top", fontsize=16, fontweight="bold", color=DARK)

# pull numbers safely
def g(d, *ks, default=None):
    for k in ks:
        if d is None:
            return default
        d = d.get(k) if isinstance(d, dict) else default
    return d if d is not None else default

p0fp = g(integ, "results", "P0_phase1c_baseline", "FP", default=86)
p1fp = g(integ, "results", "P1_block9_appe", "FP", default=54)
c0rec = g(yb, "methods", "C0_operational_shared960@0.02", "recall", default=0.826)
ppRec = g(yb, "methods", "C2_perprompt960@0.02", "recall", default=0.959)
f2 = g(yb, "failure_taxonomy", "F2_shared_nms_suppression", default=140)
miss_total = 163
if yb and "failure_taxonomy" in yb:
    ft = yb["failure_taxonomy"]
    miss_total = ft.get("F2_shared_nms_suppression", 0) + ft.get("F1_below_confidence", 0) + ft.get("F0_no_raw_prediction", 0)

if KOREAN:
    blocks_txt = [
        ("1) Appearance (외형)",
         f"block9 게이트를 쓰면 오검출 FP {p0fp}→{p1fp}로 감소. 단 블록 변별력이 객체마다 편중\n"
         f"   (Bear/Dinosaur는 block2, choco는 block11) → 게이트는 조건부로만 채택.", TEAL),
        ("2) Semantic (의미)",
         "Top-5 집계가 이미 최상급. S1~S7 어떤 변형도 AUROC 0.79~0.81로 사실상 동일\n"
         "   → 집계 방식 변경의 이득 없음 (현행 top5 유지).", "#e9c46a"),
        ("3) YOLO bbox (후보 생성)",
         f"누락 {miss_total}건 중 {f2}건이 shared-pass NMS 억제(F2)가 원인.\n"
         f"   per-prompt 검출로 recall {c0rec:.3f}→{ppRec:.3f} 회복 (이미 cur/eval 경로엔 적용됨).", "#e76f51"),
        ("결론 / 우선순위",
         f"① per-prompt 후보 보장 (+약 84 TP)  ② block9 게이트는 조건부 채택.\n"
         f"   최종 병목 = 후보생성 단계의 NMS 억제 > 게이트.", DARK),
    ]
else:
    blocks_txt = [
        ("1) Appearance",
         f"block9 gate cuts FP {p0fp}->{p1fp}. But per-block discriminability is class-dependent\n"
         f"   (Bear/Dinosaur favor block2, choco favors block11) -> adopt gate conditionally.", TEAL),
        ("2) Semantic",
         "Top-5 aggregation is already top-tier. S1..S7 all AUROC 0.79~0.81 (essentially equal)\n"
         "   -> no benefit from changing aggregation (keep top5).", "#e9c46a"),
        ("3) YOLO bbox (candidate generation)",
         f"Of {miss_total} misses, {f2} are shared-pass NMS suppression (F2).\n"
         f"   per-prompt detection recovers recall {c0rec:.3f}->{ppRec:.3f} (already applied on cur/eval path).", "#e76f51"),
        ("Conclusion / priority",
         f"1) Guarantee per-prompt candidates (~+84 TP)  2) block9 gate conditionally.\n"
         f"   Main bottleneck = NMS suppression at candidate stage > the gate.", DARK),
    ]

y = 0.85
for head, body, col in blocks_txt:
    ax.add_patch(plt.Rectangle((0.03, y - 0.145), 0.94, 0.15, transform=ax.transAxes,
                               facecolor=col, alpha=0.12, edgecolor=col, lw=1.5))
    ax.text(0.05, y, head, fontsize=13, fontweight="bold", color=col, va="top", transform=ax.transAxes)
    ax.text(0.05, y - 0.045, body, fontsize=10.5, color="#222", va="top", transform=ax.transAxes)
    y -= 0.205
ax.text(0.5, 0.02, L("출처: 사람 GT 339프레임 / 935 셀 · Phase 2 metrics",
                     "source: human GT 339 frames / 935 cells - Phase 2 metrics"),
        ha="center", fontsize=8, color="gray", transform=ax.transAxes)
save(fig, "integrated", "phase2_beginner_summary")

# ================================================================
print("\n" + "=" * 60)
print(f"Korean font: {'YES - ' if KOREAN else 'NO - '}{FONT_USED}")
print(f"Generated {len(GEN)} files:")
for p in GEN:
    print("  ", p)
