#!/usr/bin/env python3
"""new_score_why.py — WHY does the product help on some pairs and not others?

A product of channels is a SUM IN LOG SPACE:

    log(A*B) = log A + log B

so ranking by A*B is ranking by the sum of log-evidence. For a head-to-head
between the true object `s` and a rival `r` on the SAME crop, define the
per-channel log-ratio

    L_c = log( score_c(crop, s) / score_c(crop, r) )

The product wins that crop iff  sum_c L_c > 0.  Everything about "when does
multiplying help" follows from the distribution of L_c:

  * mean(L_c) > 0   -> channel c pushes toward the right answer
  * |mean(L_c)|     -> how hard it pushes  (its VOTE WEIGHT in the sum)
  * corr(L_a, L_b)  -> whether two channels are making the same mistake
  * a channel with a tiny |mean(L_c)| contributes almost nothing no matter how
    good its solo win-rate looks, because the sum is dominated by whichever
    channel has the largest log-ratio magnitude.

This is also why "the product widens the distribution" is the wrong mental
model: multiplying shrinks the ABSOLUTE range (three numbers <1), so absolute
spread must be compared in relative terms (CV, or spread of log score).
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                        # noqa: E402
import pandas as pd                       # noqa: E402

EPS = 1e-6
CHANNELS = ["sem", "appe_b2", "appe_b9", "appe_b11", "color_emd"]
PAIRS = [("choco_hazelnut_high", "Bear"),
         ("choco_hazelnut_high", "milk"),
         ("choco_hazelnut_high", "saffron"),
         ("Bear", "Rabbit"),
         ("Bear", "Dinosaur"),
         ("Rabbit", "Bear"),
         ("Dinosaur", "Bear")]


def savefig(fig, out, name):
    fig.savefig(os.path.join(out, name), dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {name}")


def logratios(df, src, rival):
    """DataFrame of per-crop log-ratios L_c for one (src, rival) pair."""
    sub = df[df.src_object == src]
    out = {}
    for c in CHANNELS:
        piv = sub.pivot_table(index="cand_id", columns="tgt_object", values=c)
        if src not in piv.columns or rival not in piv.columns:
            return None
        out[c] = np.log(np.clip(piv[src], EPS, None) /
                        np.clip(piv[rival], EPS, None))
    return pd.DataFrame(out).dropna()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cross", required=True)
    ap.add_argument("--cand", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    tdir = os.path.join(a.out, "tables")
    os.makedirs(tdir, exist_ok=True)

    df = pd.read_csv(a.cross)
    cand = pd.read_csv(a.cand)

    # ------------------------------------------------------------------
    # 1) Vote weight: how hard does each channel push, per pair?
    # ------------------------------------------------------------------
    rows = []
    for src, rival in PAIRS:
        L = logratios(df, src, rival)
        if L is None or L.empty:
            continue
        for c in CHANNELS:
            rows.append({"pair": f"{src[:12]} vs {rival[:9]}", "channel": c,
                         "mean_logratio": L[c].mean(),
                         "abs_weight": abs(L[c].mean()),
                         "std_logratio": L[c].std(),
                         "win_rate": (L[c] > 0).mean(),
                         "n": len(L)})
    W = pd.DataFrame(rows)
    W.to_csv(os.path.join(tdir, "why_channel_vote_weight.csv"), index=False)

    piv = W.pivot_table(index="channel", columns="pair", values="mean_logratio")
    fig, ax = plt.subplots(figsize=(1.6 * len(piv.columns) + 4, 4.2))
    v = np.abs(piv.values)
    im = ax.imshow(piv.values, cmap="RdYlGn", vmin=-np.nanmax(v),
                   vmax=np.nanmax(v), aspect="auto")
    ax.set_xticks(range(len(piv.columns)))
    ax.set_xticklabels(piv.columns, rotation=30, ha="right", fontsize=8)
    ax.set_yticks(range(len(piv.index))); ax.set_yticklabels(piv.index, fontsize=9)
    for i in range(piv.shape[0]):
        for j in range(piv.shape[1]):
            if not np.isnan(piv.values[i, j]):
                ax.text(j, i, f"{piv.values[i,j]:+.2f}", ha="center",
                        va="center", fontsize=8)
    ax.set_title("VOTE WEIGHT: mean log-ratio log(score_true / score_rival)\n"
                 "green = pushes to the right answer, red = pushes to the WRONG one.\n"
                 "Magnitude = how much this channel moves the product.")
    fig.colorbar(im, ax=ax, fraction=.03)
    fig.tight_layout()
    savefig(fig, a.out, "17_why_vote_weight.png")

    # ------------------------------------------------------------------
    # 2) Are the channels making the SAME mistake? (error decorrelation)
    # ------------------------------------------------------------------
    rows = []
    for src, rival in PAIRS:
        L = logratios(df, src, rival)
        if L is None or L.empty:
            continue
        for c1 in ["color_emd"]:
            for c2 in ["appe_b9", "appe_b11", "appe_b2", "sem"]:
                rows.append({"pair": f"{src[:12]} vs {rival[:9]}",
                             "pair_of": f"{c1} ~ {c2}",
                             "corr": L[c1].corr(L[c2])})
    C = pd.DataFrame(rows)
    C.to_csv(os.path.join(tdir, "why_logratio_correlation.csv"), index=False)
    pc = C.pivot_table(index="pair_of", columns="pair", values="corr")
    fig, ax = plt.subplots(figsize=(1.6 * len(pc.columns) + 4, 3.4))
    im = ax.imshow(pc.values, cmap="coolwarm", vmin=-1, vmax=1, aspect="auto")
    ax.set_xticks(range(len(pc.columns)))
    ax.set_xticklabels(pc.columns, rotation=30, ha="right", fontsize=8)
    ax.set_yticks(range(len(pc.index))); ax.set_yticklabels(pc.index, fontsize=9)
    for i in range(pc.shape[0]):
        for j in range(pc.shape[1]):
            if not np.isnan(pc.values[i, j]):
                ax.text(j, i, f"{pc.values[i,j]:+.2f}", ha="center",
                        va="center", fontsize=8)
    ax.set_title("Do colour and texture make the SAME mistake on the same crop?\n"
                 "corr of their log-ratios. Near 0 = complementary "
                 "(the product can help); near +1 = redundant.")
    fig.colorbar(im, ax=ax, fraction=.03)
    fig.tight_layout()
    savefig(fig, a.out, "18_why_error_decorrelation.png")

    # ------------------------------------------------------------------
    # 3) Predicted vs actual: does the sum-of-log-ratios explain the product?
    # ------------------------------------------------------------------
    rows = []
    for src, rival in PAIRS:
        L = logratios(df, src, rival)
        if L is None or L.empty:
            continue
        combos = {
            "color": ["color_emd"],
            "b9": ["appe_b9"],
            "b11": ["appe_b11"],
            "color x b9": ["color_emd", "appe_b9"],
            "color x b2 x b9": ["color_emd", "appe_b2", "appe_b9"],
            "sem x b11": ["sem", "appe_b11"],
            "sem x color x b9": ["sem", "color_emd", "appe_b9"],
        }
        for k, cs in combos.items():
            s = L[cs].sum(axis=1)
            rows.append({"pair": f"{src[:12]} vs {rival[:9]}", "combo": k,
                         "win_rate": (s > 0).mean()})
    P = pd.DataFrame(rows)
    P.to_csv(os.path.join(tdir, "why_combo_winrate.csv"), index=False)
    pp = P.pivot_table(index="combo", columns="pair", values="win_rate")
    pp["MEAN"] = pp.mean(axis=1)
    pp = pp.sort_values("MEAN", ascending=False)
    print("\n=== win rate by combination (= P(sum of log-ratios > 0)) ===")
    print(pp.round(3).to_string())

    # ------------------------------------------------------------------
    # 4) Spread: absolute vs RELATIVE. The absolute comparison was unfair.
    # ------------------------------------------------------------------
    d = cand.copy()
    d["prod"] = (d["color_emd_mask"] * d["appe_b2_mask"] * d["appe_b9_mask"])
    rows = []
    for o, g in d.groupby("object"):
        for name, s in (("appe_b11", g["appe_b11_mask"]), ("prod", g["prod"])):
            s = s.dropna()
            s = s[s > EPS]
            if len(s) < 20:
                continue
            p5, p95 = np.percentile(s, [5, 95])
            rows.append({"object": o, "score": name,
                         "median": s.median(),
                         "abs_spread": p95 - p5,
                         "rel_spread": (p95 - p5) / s.median(),
                         "log_spread": np.log(p95) - np.log(p5)})
    S = pd.DataFrame(rows)
    S.to_csv(os.path.join(tdir, "why_spread_relative.csv"), index=False)
    piv_abs = S.pivot(index="object", columns="score", values="abs_spread")
    piv_rel = S.pivot(index="object", columns="score", values="log_spread")
    fig, axes = plt.subplots(1, 2, figsize=(15, 5))
    x = np.arange(len(piv_abs))
    axes[0].bar(x - .2, piv_abs["appe_b11"], .4, label="appe_b11")
    axes[0].bar(x + .2, piv_abs["prod"], .4, label="color x b2 x b9")
    axes[0].set_xticks(x); axes[0].set_xticklabels(piv_abs.index, rotation=35,
                                                   ha="right", fontsize=8)
    axes[0].set_title("ABSOLUTE 5-95 spread  (misleading:\n"
                      "a product of three numbers <1 is smaller by construction)")
    axes[0].legend(); axes[0].grid(alpha=.3, axis="y")
    axes[1].bar(x - .2, piv_rel["appe_b11"], .4, label="appe_b11")
    axes[1].bar(x + .2, piv_rel["prod"], .4, label="color x b2 x b9")
    axes[1].set_xticks(x); axes[1].set_xticklabels(piv_rel.index, rotation=35,
                                                   ha="right", fontsize=8)
    axes[1].set_title("LOG spread = log(p95) - log(p5)  (scale-free,\n"
                      "this is the fair comparison)")
    axes[1].legend(); axes[1].grid(alpha=.3, axis="y")
    fig.tight_layout()
    savefig(fig, a.out, "19_why_spread_absolute_vs_relative.png")
    print("\n=== spread: absolute vs log (scale-free) ===")
    print(S.pivot(index="object", columns="score",
                  values=["abs_spread", "log_spread"]).round(3).to_string())

    # ------------------------------------------------------------------
    # 5) Dynamic range in log space = how much each channel can move a product
    # ------------------------------------------------------------------
    rows = []
    for c in CHANNELS:
        s = cand[f"{c}_mask" if f"{c}_mask" in cand.columns else c].dropna()
        s = s[s > EPS]
        rows.append({"channel": c, "log_std": np.log(s).std(),
                     "log_p5_p95": np.log(np.percentile(s, 95)) -
                                   np.log(np.percentile(s, 5))})
    R = pd.DataFrame(rows).sort_values("log_p5_p95", ascending=False)
    R.to_csv(os.path.join(tdir, "why_channel_log_range.csv"), index=False)
    fig, ax = plt.subplots(figsize=(9, 4.4))
    ax.bar(R.channel, R.log_p5_p95, color="#6a4c93")
    ax.set_ylabel("log(p95) - log(p5)")
    ax.set_title("How much can each channel move a PRODUCT?\n"
                 "In log space the product is a SUM, so the channel with the\n"
                 "largest log range dominates the ranking.")
    ax.grid(alpha=.3, axis="y")
    fig.tight_layout()
    savefig(fig, a.out, "20_why_channel_log_range.png")
    print("\n=== channel dynamic range in log space (product influence) ===")
    print(R.round(3).to_string(index=False))

    print(f"\n[done] -> {a.out}")


if __name__ == "__main__":
    main()
