#!/usr/bin/env python3
"""new_score_cross.py — discriminability analysis from the cross-object scores.

`candidates.csv` scores each proposal only against its OWN object's templates,
which cannot answer "can this channel tell the objects apart?" — a real choco
box and a plain brown carton both match their own best template reasonably
well. `cross_scores.csv` scores EVERY candidate against EVERY object's
templates, which makes the discriminative question answerable:

  for one image crop, does the channel rank the right object first?

CAVEAT, stated up front: there are no TP/FP labels for this bag. `src_object`
is the prompt that produced the box, NOT ground truth — the "maroon box"
prompt famously fires on plain brown cartons. So `src_object` agreement is a
PROXY for discriminability, not an accuracy measurement. What is fully sound
here is the *structure*: which objects a channel confuses with which, and how
peaked (top1 vs top2) each channel's 10-way vote is.
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                        # noqa: E402
import pandas as pd                       # noqa: E402

CHANNELS = ["sem", "appe_b2", "appe_b9", "appe_b11", "color_emd"]
COMBOS = {
    "b2 x b9": lambda d: d["appe_b2"] * d["appe_b9"],
    "(b2+b9)/2": lambda d: 0.5 * d["appe_b2"] + 0.5 * d["appe_b9"],
    "color x b9": lambda d: d["color_emd"] * d["appe_b9"],
    "color x b2 x b9": lambda d: d["color_emd"] * d["appe_b2"] * d["appe_b9"],
    "sem x color x b9": lambda d: d["sem"] * d["color_emd"] * d["appe_b9"],
    "sem x b11 (2-gate)": lambda d: d["sem"] * d["appe_b11"],
}


def savefig(fig, out, name):
    fig.savefig(os.path.join(out, name), dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {name}")


def add_combos(df):
    for k, f in COMBOS.items():
        df[k] = f(df)
    return df


def top1_agreement(df, cols):
    """Per channel: how often is src_object the argmax over the 10 targets?"""
    rows = []
    for c in cols:
        idx = df.groupby("cand_id")[c].idxmax()
        win = df.loc[idx, ["cand_id", "src_object", "tgt_object"]]
        rows.append({"channel": c,
                     "top1_agree": (win.src_object == win.tgt_object).mean()})
    return pd.DataFrame(rows).sort_values("top1_agree", ascending=False)


def peakedness(df, cols):
    """Per channel: margin between the best and 2nd-best object for one crop."""
    rows = []
    for c in cols:
        g = df.groupby("cand_id")[c]
        top2 = g.apply(lambda s: np.sort(s.values)[-2:] if len(s) > 1 else
                       np.array([np.nan, np.nan]))
        m = np.array([t[1] - t[0] for t in top2])
        rel = np.array([(t[1] - t[0]) / t[1] if t[1] > 0 else np.nan
                        for t in top2])
        rows.append({"channel": c, "margin_med": np.nanmedian(m),
                     "margin_p90": np.nanpercentile(m, 90),
                     "rel_margin_med": np.nanmedian(rel)})
    return pd.DataFrame(rows).sort_values("rel_margin_med", ascending=False)


def fig_top1(agree, out):
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.barh(agree.channel[::-1], agree.top1_agree.values[::-1] * 100,
            color="#2a9d8f")
    ax.axvline(10, ls="--", c="k", lw=1)
    ax.text(10.5, 0, "chance (1/10)", fontsize=8, rotation=90, va="bottom")
    ax.set_xlabel("% of crops where the prompt's object wins the 10-way vote")
    ax.set_title("Discriminability: does the channel rank the right object first?\n"
                 "(src_object is the PROMPT, a proxy — not ground truth)")
    ax.grid(alpha=.3, axis="x")
    fig.tight_layout()
    savefig(fig, out, "12_cross_top1_agreement.png")


def fig_margin(peak, out):
    fig, ax = plt.subplots(figsize=(10, 5))
    x = np.arange(len(peak))
    ax.bar(x, peak.rel_margin_med * 100, color="#e76f51")
    ax.set_xticks(x); ax.set_xticklabels(peak.channel, rotation=25, ha="right",
                                         fontsize=9)
    ax.set_ylabel("median (top1-top2)/top1  [%]")
    ax.set_title("How PEAKED is the 10-way vote?\n"
                 "a flat vote means the channel cannot commit to an object")
    ax.grid(alpha=.3, axis="y")
    fig.tight_layout()
    savefig(fig, out, "13_cross_vote_peakedness.png")


def fig_confusion(df, out, channels):
    """src_object x argmax_object confusion structure, per channel."""
    objs = sorted(df.tgt_object.unique())
    n = len(channels)
    fig, axes = plt.subplots(1, n, figsize=(4.6 * n, 4.6))
    for ax, c in zip(np.atleast_1d(axes), channels):
        idx = df.groupby("cand_id")[c].idxmax()
        win = df.loc[idx, ["src_object", "tgt_object"]]
        m = pd.crosstab(win.src_object, win.tgt_object)
        m = m.reindex(index=objs, columns=objs, fill_value=0)
        mn = m.div(m.sum(axis=1).replace(0, 1), axis=0)
        im = ax.imshow(mn.values, vmin=0, vmax=1, cmap="Blues")
        ax.set_xticks(range(len(objs)))
        ax.set_xticklabels(objs, rotation=90, fontsize=6)
        ax.set_yticks(range(len(objs)))
        ax.set_yticklabels(objs if c == channels[0] else [""] * len(objs),
                           fontsize=6)
        ax.set_title(c, fontsize=10)
        ax.set_xlabel("argmax object")
    fig.suptitle("Which object does each channel pick, per prompt "
                 "(row-normalised; diagonal = prompt and channel agree)")
    fig.tight_layout()
    savefig(fig, out, "14_cross_confusion_matrices.png")


def fig_pair_focus(df, out):
    """The two hard confusions, scored head-to-head on the SAME crops."""
    pairs = [("choco_hazelnut_high", ["milk", "saffron", "Bear"]),
             ("Bear", ["Rabbit", "Dinosaur"]),
             ("Rabbit", ["Bear", "Dinosaur"]),
             ("Dinosaur", ["Bear", "Rabbit"])]
    cols = CHANNELS + list(COMBOS)
    rows = []
    for src, rivals in pairs:
        sub = df[df.src_object == src]
        if sub.empty:
            continue
        for c in cols:
            piv = sub.pivot_table(index="cand_id", columns="tgt_object",
                                  values=c)
            if src not in piv.columns:
                continue
            for rv in rivals:
                if rv not in piv.columns:
                    continue
                wins = (piv[src] > piv[rv]).mean()
                rows.append({"prompt": src, "rival": rv, "channel": c,
                             "win_rate": wins})
    r = pd.DataFrame(rows)
    if r.empty:
        return r
    piv = r.pivot_table(index="channel", columns=["prompt", "rival"],
                        values="win_rate")
    piv["MEAN"] = piv.mean(axis=1)
    piv = piv.sort_values("MEAN", ascending=False)
    fig, ax = plt.subplots(figsize=(1.15 * len(piv.columns) + 5,
                                    0.5 * len(piv) + 3))
    im = ax.imshow(piv.values, vmin=0, vmax=1, cmap="RdYlGn", aspect="auto")
    ax.set_xticks(range(len(piv.columns)))
    ax.set_xticklabels([" vs ".join(c) if isinstance(c, tuple) else c
                        for c in piv.columns], rotation=40, ha="right",
                       fontsize=7)
    ax.set_yticks(range(len(piv.index)))
    ax.set_yticklabels(piv.index, fontsize=9)
    for i in range(piv.shape[0]):
        for j in range(piv.shape[1]):
            if not np.isnan(piv.values[i, j]):
                ax.text(j, i, f"{piv.values[i,j]:.2f}", ha="center",
                        va="center", fontsize=7)
    ax.set_title("Head-to-head on the SAME crop: how often does the prompt's\n"
                 "object outscore the confusable rival? (0.5 = coin flip)")
    fig.colorbar(im, ax=ax, fraction=.03)
    fig.tight_layout()
    savefig(fig, out, "15_hard_pair_headtohead.png")
    return piv


def fig_color_vs_texture_pairs(df, out):
    """Are colour and texture complementary on the hard pairs, or redundant?"""
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.2))
    for ax, (src, rival) in zip(axes, [("choco_hazelnut_high", "Bear"),
                                       ("Bear", "Rabbit")]):
        sub = df[df.src_object == src]
        pc = sub.pivot_table(index="cand_id", columns="tgt_object",
                             values="color_emd")
        pa = sub.pivot_table(index="cand_id", columns="tgt_object",
                             values="appe_b9")
        if src not in pc.columns or rival not in pc.columns:
            continue
        dc = pc[src] - pc[rival]
        da = pa[src] - pa[rival]
        ax.scatter(dc, da, s=8, alpha=.4, color="#264653")
        ax.axhline(0, c="r", lw=1); ax.axvline(0, c="r", lw=1)
        ax.set_xlabel(f"color_emd({src}) - color_emd({rival})")
        ax.set_ylabel(f"appe_b9({src}) - appe_b9({rival})")
        ax.set_title(f"{src} vs {rival}\n"
                     "upper-right = both channels agree it's the prompt object")
        ax.grid(alpha=.3)
    fig.tight_layout()
    savefig(fig, out, "16_color_texture_complementarity.png")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    tdir = os.path.join(a.out, "tables")
    os.makedirs(tdir, exist_ok=True)

    df = add_combos(pd.read_csv(a.csv))
    cols = CHANNELS + list(COMBOS)
    print(f"[data] {df.cand_id.nunique()} crops x {df.tgt_object.nunique()} "
          f"objects = {len(df)} rows")

    agree = top1_agreement(df, cols)
    peak = peakedness(df, cols)
    agree.to_csv(os.path.join(tdir, "cross_top1_agreement.csv"), index=False)
    peak.to_csv(os.path.join(tdir, "cross_vote_peakedness.csv"), index=False)
    print("\n=== top-1 agreement with the prompt (proxy for discriminability) ===")
    print(agree.round(3).to_string(index=False))
    print("\n=== peakedness of the 10-way vote ===")
    print(peak.round(4).to_string(index=False))

    print("\n[figures]")
    fig_top1(agree, a.out)
    fig_margin(peak, a.out)
    fig_confusion(df, a.out, ["sem", "appe_b11", "appe_b9", "color_emd"])
    hp = fig_pair_focus(df, a.out)
    if len(hp):
        hp.to_csv(os.path.join(tdir, "hard_pair_headtohead.csv"))
        print("\n=== head-to-head on hard pairs ===")
        print(hp.round(3).to_string())
    fig_color_vs_texture_pairs(df, a.out)
    print(f"\n[done] -> {a.out}")


if __name__ == "__main__":
    main()
