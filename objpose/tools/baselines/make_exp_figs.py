#!/usr/bin/env python3
"""make_exp_figs.py — figures for the thesis comparison slides, from eval_cmp.py outputs.

  ar_time_<ds>.png   BOP AR vs time per image: published methods, SAM-6D re-run here, proposed
  rt_compare.png     YCB-V real-time: processed frames / correct-when-answered / on-screen recall
  perobj_ycbv.png    per-object recall (MSSD) proposed vs original re-run
"""
from __future__ import annotations

import glob
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm

REPO = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
OUT = REPO / "objpose/output/cmp_260924"
FIG = Path(sys.argv[1]) if len(sys.argv) > 1 else OUT / "figs"
FIG.mkdir(parents=True, exist_ok=True)
fm.fontManager.addfont("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "axes.unicode_minus": False, "axes.spines.top": False,
                     "axes.spines.right": False, "font.size": 10})
NAVY, ORANGE, GREY = "#18334b", "#d9822b", "#9a9a9a"

pub = json.loads((HERE / "published_bop.json").read_text())["rows"]
OFF = {"Co-op (5 hyp)": (-4, 4, "right"), "FreeZe (SAM-6D masks, refined)": (4, -9, "left"),
       "FoundationPose": (4, -8, "left"), "FreeZeV2.1 (BOP24 1st)": (4, 2, "left"),
       "MegaPose + refiner (5 hyp)": (4, -9, "left"), "GigaPose + MegaPose (5 hyp)": (-4, 4, "right"),
       "FoundPose + MegaPose (5 hyp)": (4, 2, "left"), "GenFlow-MultiHypo16": (-4, 4, "right"),
       "MegaPose + Teaser++": (-4, -9, "right")}
sc = json.loads((OUT / "bop_scores.json").read_text())


def score(prefix, ds):
    for k, v in sc.items():
        if k.startswith(prefix) and k.endswith(f"_{ds}-test.csv") and v.get("ar") is not None:
            return v
    return None


def ar_time(ds):
    fig, ax = plt.subplots(figsize=(5.2, 3.3), dpi=220)
    for r in pub:
        if r["method"].startswith("SAM-6D"):
            continue
        filled = r["input"] == "RGB-D"
        ax.scatter(r["time"], r[ds], s=28, facecolor=GREY if filled else "white", edgecolor=GREY, zorder=2)
        dx, dy, ha = OFF.get(r["method"], (4, -3, "left"))
        ax.annotate(r["method"].split(" (")[0].replace(" + ", "+"), (r["time"], r[ds]), xytext=(dx, dy),
                    textcoords="offset points", fontsize=6.5, color="#6f6f6f", ha=ha)
    for r in pub:
        if r["method"].startswith("SAM-6D"):
            ax.scatter(r["time"], r[ds], s=36, color="#5b7fa3", zorder=3)
            up = "SAM)" in r["method"]
            ax.annotate(r["method"] + " 논문", (r["time"], r[ds]), xytext=(-4, 6) if up else (-5, -12),
                        textcoords="offset points", fontsize=7, color="#5b7fa3", ha="right")
    o = score("orig-orig", ds)
    if o:
        ax.scatter(o["time"], 100 * o["ar"], s=40, color=ORANGE, marker="s", zorder=4)
        ax.annotate("SAM-6D (FastSAM) 재실행", (o["time"], 100 * o["ar"]), xytext=(6, -3), textcoords="offset points",
                    fontsize=7.5, color=ORANGE)
    p = score("proposed-proposed", ds)
    if p:
        ax.scatter(p["time"], 100 * p["ar"], s=90, color=NAVY, marker="*", zorder=5)
        ax.annotate("제안 방법", (p["time"], 100 * p["ar"]), xytext=(6, -3), textcoords="offset points",
                    fontsize=8.5, color=NAVY, weight="bold")
    ax.set_xscale("log"); ax.set_xlabel("이미지당 처리 시간 [s] (로그)"); ax.set_ylabel("BOP AR [%]")
    ax.set_xlim(0.2, 300); ax.grid(alpha=0.25)
    fig.tight_layout(); fig.savefig(FIG / f"ar_time_{ds}.png", facecolor="white"); plt.close(fig)


def rt():
    r = json.loads((OUT / "rt_scores.json").read_text())
    keys = [("proc_rate", "처리한 프레임"), ("adds_cond", "답이 맞는 비율\n(ADD-S)"), ("recall_proc", "처리 프레임\n재현율"),
            ("screen_hold", "화면 재현율\n결과 유지"), ("screen_slam", "화면 재현율\nSLAM 보정")]
    x = np.arange(len(keys)); w = 0.36
    fig, ax = plt.subplots(figsize=(6.2, 2.9), dpi=220)
    for k, (m, col, lab) in enumerate([("orig", ORANGE, "원본 SAM-6D"), ("proposed", NAVY, "제안 방법")]):
        v = [100 * r[m][kk] for kk, _ in keys]
        b = ax.bar(x + (k - 0.5) * w, v, w, color=col, label=lab)
        for xi, vi in zip(x + (k - 0.5) * w, v):
            ax.text(xi, vi + 1.5, f"{vi:.1f}", ha="center", fontsize=7.5)
    ax.set_xticks(x); ax.set_xticklabels([l for _, l in keys], fontsize=8.5); ax.set_ylim(0, 112); ax.set_ylabel("%")
    ax.legend(frameon=False, fontsize=8.5, loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=2)
    fig.tight_layout(); fig.savefig(FIG / "rt_compare.png", facecolor="white"); plt.close(fig)


def perobj():
    names = {o["obj_id"]: o["name"][5:] for o in json.loads((REPO / "objpose/rt/bop/ycbv_objects.json").read_text())["objects"]}
    def rec(prefix):
        f = glob.glob(str(HERE / f"bop_eval/{prefix}*_ycbv-test/error=mssd_ntop=-1/scores_th=0.200_min-visib=-1.000.json"))
        return {int(k): v for k, v in json.loads(Path(f[0]).read_text())["obj_recalls"].items()} if f else None
    p, o = rec("proposed-proposedycbvbop19"), rec("orig-origycbvbop19")
    if not p or not o:
        return
    ids = sorted(p, key=lambda i: -(o[i]))
    y = np.arange(len(ids))
    fig, ax = plt.subplots(figsize=(5.0, 4.4), dpi=220)
    ax.barh(y + 0.2, [100 * o[i] for i in ids], 0.4, color=ORANGE, label="원본 SAM-6D 재실행")
    ax.barh(y - 0.2, [100 * p[i] for i in ids], 0.4, color=NAVY, label="제안 방법")
    ax.set_yticks(y); ax.set_yticklabels([names[i] for i in ids], fontsize=7.5); ax.invert_yaxis()
    ax.set_xlabel("MSSD 재현율 (임계 0.2d) [%]"); ax.legend(frameon=False, fontsize=8, loc="lower right")
    fig.tight_layout(); fig.savefig(FIG / "perobj_ycbv.png", facecolor="white"); plt.close(fig)


if __name__ == "__main__":
    for ds in ("ycbv", "lmo"):
        ar_time(ds)
    if (OUT / "rt_scores.json").exists():
        rt()
    perobj()
    print("figs ->", FIG)
