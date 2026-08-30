#!/usr/bin/env python3
"""new_score_analyze.py — tables + figures for the new-score study.

Consumes the per-candidate CSV written by new_score_probe.py and produces:
  tables/  per-object summary of every channel, and channel-vs-channel stats
  *.png    the figures needed to choose a direction

IMPORTANT — no TP/FP labels exist for this bag, so nothing here is a
precision/recall measurement. What IS measurable without labels:
  * how wide each channel's distribution is (a channel squeezed into a narrow
    band cannot separate anything, whatever the threshold)
  * how well a channel separates OBJECT IDENTITIES that are known to be
    confused (the doll trio; box-shaped objects vs each other)
  * how much a product of channels widens the distribution vs a single channel
Anything requiring "is this detection correct?" is flagged as needing labels.

Run in an env with pandas+matplotlib (e.g. sam6d_ros_humble).
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                        # noqa: E402
import pandas as pd                       # noqa: E402

BLOCKS = [2, 9, 11]
DOLLS = ["Bear", "Rabbit", "Dinosaur"]
BOXY = ["choco_hazelnut_high", "milk", "saffron"]

CHANNELS_MASK = (["sem"] + [f"appe_b{b}_mask" for b in BLOCKS] +
                 ["color_bc32_mask", "color_bc16_mask", "color_bc16s_mask",
                  "color_emd_mask"])


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--csv", required=True)
    p.add_argument("--out", required=True)
    return p.parse_args()


def auc(pos, neg):
    """Mann-Whitney AUC: P(a random pos scores above a random neg)."""
    pos, neg = np.asarray(pos, float), np.asarray(neg, float)
    if len(pos) == 0 or len(neg) == 0:
        return np.nan
    allv = np.concatenate([pos, neg])
    r = pd.Series(allv).rank().values
    return (r[:len(pos)].sum() - len(pos) * (len(pos) + 1) / 2) / (len(pos) * len(neg))


def savefig(fig, out, name):
    path = os.path.join(out, name)
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {name}")


# ---------------------------------------------------------------------------
def fig_channel_spread(df, out):
    """How wide is each channel? A narrow band cannot separate anything."""
    chans = [c for c in CHANNELS_MASK if c in df.columns]
    fig, ax = plt.subplots(figsize=(11, 5))
    data = [df[c].dropna().values for c in chans]
    bp = ax.boxplot(data, tick_labels=[c.replace("_mask", "") for c in chans],
                    showfliers=False, patch_artist=True)
    for b in bp["boxes"]:
        b.set_facecolor("#8ecae6")
    for i, d in enumerate(data, 1):
        p5, p95 = np.percentile(d, [5, 95])
        ax.text(i, p95 + 0.02, f"{p95 - p5:.2f}", ha="center", fontsize=8,
                color="#d62828")
    ax.set_ylabel("score")
    ax.set_title("Channel spread over all candidates (red = 5-95 percentile width)\n"
                 "a channel squeezed into a narrow band cannot separate TP from FP")
    ax.tick_params(axis="x", rotation=30)
    ax.grid(alpha=.3, axis="y")
    savefig(fig, out, "01_channel_spread.png")


def fig_per_object_hist(df, out):
    """Per-object distribution of the production score vs the candidate channels."""
    objs = sorted(df.object.unique())
    chans = ["sem", "appe_b11_mask", "appe_b9_mask", "appe_b2_mask",
             "color_emd_mask"]
    chans = [c for c in chans if c in df.columns]
    fig, axes = plt.subplots(len(objs), len(chans),
                             figsize=(3.0 * len(chans), 1.9 * len(objs)),
                             sharex="col")
    for i, o in enumerate(objs):
        sub = df[df.object == o]
        for j, c in enumerate(chans):
            ax = axes[i, j]
            ax.hist(sub[c].dropna(), bins=30, range=(0, 1), color="#023047")
            if i == 0:
                ax.set_title(c.replace("_mask", ""), fontsize=10)
            if j == 0:
                ax.set_ylabel(o[:14], fontsize=8)
            ax.tick_params(labelsize=7)
    fig.suptitle("Per-object score distributions (all candidates, no gate)", y=1.001)
    fig.tight_layout()
    savefig(fig, out, "02_per_object_distributions.png")


def combination_variants(df):
    """Candidate ways of combining the channels, incl. the additive rules.

    The multiplicative rule assumes the factors are independent evidence.
    appe_b2 and appe_b9 are NOT: they are nested prefixes of one DINOv2
    forward pass on the same crop, so multiplying them double-counts. The
    additive variants are here so the data can adjudicate that.
    """
    d = df
    return {
        "appe_b11 (production)": d["appe_b11_mask"],
        "appe_b9": d["appe_b9_mask"],
        "color_emd": d["color_emd_mask"],
        "b2 x b9 (mult)": d["appe_b2_mask"] * d["appe_b9_mask"],
        "0.5*b2 + 0.5*b9 (add)": 0.5 * d["appe_b2_mask"] + 0.5 * d["appe_b9_mask"],
        "color x b9": d["color_emd_mask"] * d["appe_b9_mask"],
        "color x b2 x b9": (d["color_emd_mask"] * d["appe_b2_mask"] *
                            d["appe_b9_mask"]),
        "color x (b2+b9)/2": d["color_emd_mask"] * (
            0.5 * d["appe_b2_mask"] + 0.5 * d["appe_b9_mask"]),
    }


def fig_product_vs_single(df, out):
    """Does multiplying channels widen the distribution vs a single channel?"""
    variants = combination_variants(df)
    fig, axes = plt.subplots(1, 2, figsize=(16, 5.0))
    for k, v in variants.items():
        v = v.dropna()
        axes[0].hist(v, bins=60, range=(0, 1), histtype="step", lw=2, label=k)
    axes[0].set_title("Distribution shape: single channel vs product vs sum")
    axes[0].set_xlabel("score"); axes[0].legend(fontsize=8); axes[0].grid(alpha=.3)

    names, widths, iqrs = [], [], []
    for k, v in variants.items():
        v = v.dropna().values
        p5, p95 = np.percentile(v, [5, 95])
        q1, q3 = np.percentile(v, [25, 75])
        names.append(k); widths.append(p95 - p5); iqrs.append(q3 - q1)
    x = np.arange(len(names))
    axes[1].bar(x - .2, widths, .4, label="5-95 pct width", color="#219ebc")
    axes[1].bar(x + .2, iqrs, .4, label="IQR", color="#fb8500")
    axes[1].set_xticks(x); axes[1].set_xticklabels(names, rotation=20, fontsize=8)
    axes[1].set_title("Dynamic range (wider = more room to place a threshold)")
    axes[1].legend(); axes[1].grid(alpha=.3, axis="y")
    fig.tight_layout()
    savefig(fig, out, "03_product_vs_single.png")


def fig_color_metric_compare(df, out):
    """Which colour metric survives the render-vs-real shift?"""
    metrics = ["color_bc32_mask", "color_bc16_mask", "color_bc16s_mask",
               "color_emd_mask"]
    metrics = [m for m in metrics if m in df.columns]
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.6))
    for m in metrics:
        axes[0].hist(df[m].dropna(), bins=60, range=(0, 1), histtype="step",
                     lw=2, label=m.replace("color_", "").replace("_mask", ""))
    axes[0].set_title("Colour metric distributions\n"
                      "bin-to-bin (bc32) collapses to 0 under the render-vs-real "
                      "hue shift")
    axes[0].set_xlabel("similarity"); axes[0].legend(); axes[0].grid(alpha=.3)

    frac0 = [(df[m] < 1e-6).mean() * 100 for m in metrics]
    axes[1].bar([m.replace("color_", "").replace("_mask", "") for m in metrics],
                frac0, color="#d62828")
    axes[1].set_ylabel("% of candidates scoring ~0")
    axes[1].set_title("Dead-zero rate (a metric that returns 0 carries no information)")
    axes[1].grid(alpha=.3, axis="y")
    fig.tight_layout()
    savefig(fig, out, "04_color_metric_comparison.png")


def fig_confusion_separation(df, out):
    """Label-free separability: can a channel tell confusable OBJECTS apart?

    For every candidate we already score it against ONE object's templates.
    Here we ask the complementary question: within the doll trio (and within
    the box-like objects), do the channels give each object's own candidates a
    higher score than the other members of the group get?
    """
    groups = {"dolls (Bear/Rabbit/Dinosaur)": DOLLS,
              "box-like (choco/milk/saffron)": BOXY}
    chans = [c for c in ["sem", "appe_b2_mask", "appe_b9_mask", "appe_b11_mask",
                         "color_emd_mask"] if c in df.columns]
    fig, axes = plt.subplots(1, len(groups), figsize=(7 * len(groups), 4.6))
    rows = []
    for ax, (gname, members) in zip(np.atleast_1d(axes), groups.items()):
        members = [m for m in members if m in set(df.object)]
        mat = np.full((len(members), len(chans)), np.nan)
        for i, m in enumerate(members):
            pos = df[df.object == m]
            neg = df[df.object.isin([x for x in members if x != m])]
            for j, c in enumerate(chans):
                mat[i, j] = auc(pos[c].dropna(), neg[c].dropna())
                rows.append({"group": gname, "object": m, "channel": c,
                             "auc_vs_group": mat[i, j]})
        im = ax.imshow(mat, vmin=0.3, vmax=1.0, cmap="RdYlGn")
        ax.set_xticks(range(len(chans)))
        ax.set_xticklabels([c.replace("_mask", "") for c in chans],
                           rotation=25, fontsize=8)
        ax.set_yticks(range(len(members))); ax.set_yticklabels(members, fontsize=9)
        for i in range(mat.shape[0]):
            for j in range(mat.shape[1]):
                if not np.isnan(mat[i, j]):
                    ax.text(j, i, f"{mat[i,j]:.2f}", ha="center", va="center",
                            fontsize=8)
        ax.set_title(f"{gname}\nAUC: own candidates vs the rest of the group")
        fig.colorbar(im, ax=ax, fraction=.046)
    fig.tight_layout()
    savefig(fig, out, "05_confusion_group_separability.png")
    return pd.DataFrame(rows)


def fig_scatter(df, out):
    """Where do the objects sit in (texture, colour) space?"""
    objs = sorted(df.object.unique())
    cmap = plt.get_cmap("tab10")
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.6))
    for i, o in enumerate(objs):
        s = df[df.object == o]
        axes[0].scatter(s.appe_b9_mask, s.color_emd_mask, s=6, alpha=.45,
                        color=cmap(i % 10), label=o)
        axes[1].scatter(s.appe_b2_mask, s.appe_b9_mask, s=6, alpha=.45,
                        color=cmap(i % 10))
    axes[0].set_xlabel("appe_b9 (mid-block texture)")
    axes[0].set_ylabel("color_emd")
    axes[0].set_title("colour vs texture — separable clusters mean a product helps")
    axes[1].set_xlabel("appe_b2 (early block)")
    axes[1].set_ylabel("appe_b9 (mid block)")
    axes[1].set_title("early vs mid block — are they complementary or redundant?")
    for a in axes:
        a.grid(alpha=.3)
    axes[0].legend(fontsize=7, markerscale=2, ncol=2)
    fig.tight_layout()
    savefig(fig, out, "06_channel_scatter.png")


def fig_correlation(df, out):
    chans = [c for c in CHANNELS_MASK + ["yolo_conf", "sem_margin",
                                         "sem_entropy", "sem_viewvar"]
             if c in df.columns]
    corr = df[chans].corr(method="spearman")
    fig, ax = plt.subplots(figsize=(9, 7.5))
    im = ax.imshow(corr, vmin=-1, vmax=1, cmap="coolwarm")
    ax.set_xticks(range(len(chans)))
    ax.set_xticklabels([c.replace("_mask", "") for c in chans], rotation=45,
                       ha="right", fontsize=8)
    ax.set_yticks(range(len(chans)))
    ax.set_yticklabels([c.replace("_mask", "") for c in chans], fontsize=8)
    for i in range(len(chans)):
        for j in range(len(chans)):
            ax.text(j, i, f"{corr.iloc[i,j]:.2f}", ha="center", va="center",
                    fontsize=6.5)
    ax.set_title("Spearman correlation — channels that are highly correlated\n"
                 "add nothing when multiplied together")
    fig.colorbar(im, ax=ax, fraction=.046)
    fig.tight_layout()
    savefig(fig, out, "07_channel_correlation.png")
    return corr


def fig_muse_terms(df, out):
    """MUSE-style relative/uncertainty terms, computed label-free."""
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.4))
    axes[0].hist(df.sem_margin.dropna(), bins=60, color="#023047")
    axes[0].set_title("sem_margin (top1 - top2 template cosine)\n"
                      "MUSE's 'am I confused?' signal, free to compute")
    axes[0].set_xlabel("margin")
    axes[1].hist(df.sem_entropy.dropna(), bins=60, color="#219ebc")
    axes[1].set_title("softmax entropy over the 42 templates")
    axes[2].hist(df.sem_viewvar.dropna(), bins=60, color="#fb8500")
    axes[2].set_title("std of the 42 template cosines (view variance)")
    for a in axes:
        a.grid(alpha=.3)
    fig.tight_layout()
    savefig(fig, out, "08_muse_style_terms.png")


def fig_overlap_crosstalk(df, out):
    """How often do two different object labels claim the same image region?

    This is the cross-talk that a relative (cross-object) score would arbitrate.
    """
    def iou(a, b):
        x1 = max(a[0], b[0]); y1 = max(a[1], b[1])
        x2 = min(a[2], b[2]); y2 = min(a[3], b[3])
        iw, ih = max(0, x2 - x1), max(0, y2 - y1)
        inter = iw * ih
        ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
        return inter / ua if ua > 0 else 0.0

    pairs = []
    for _, g in df.groupby("frame_idx"):
        recs = g[["object", "x1", "y1", "x2", "y2", "appe_b9_mask",
                  "color_emd_mask", "sem"]].values
        for i in range(len(recs)):
            for j in range(i + 1, len(recs)):
                if recs[i][0] == recs[j][0]:
                    continue
                v = iou(recs[i][1:5], recs[j][1:5])
                if v > 0.5:
                    pairs.append((recs[i][0], recs[j][0], v))
    if not pairs:
        print("  (no cross-object overlaps > 0.5 IoU)")
        return pd.DataFrame()
    pdf = pd.DataFrame(pairs, columns=["obj_a", "obj_b", "iou"])
    cnt = (pdf.groupby(["obj_a", "obj_b"]).size()
           .reset_index(name="n").sort_values("n", ascending=False))
    top = cnt.head(15)
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.barh([f"{a} / {b}" for a, b in zip(top.obj_a, top.obj_b)][::-1],
            top.n.values[::-1], color="#e76f51")
    ax.set_xlabel("frames where both labels claim the same box (IoU > 0.5)")
    ax.set_title("Cross-object competition — these pairs are what a relative\n"
                 "(MUSE-style) score would have to arbitrate")
    ax.grid(alpha=.3, axis="x")
    fig.tight_layout()
    savefig(fig, out, "09_cross_object_overlap.png")
    return cnt


def fig_combination_separability(df, out):
    """THE decisive label-free test: which combination rule best separates the
    objects that are actually confused with each other?

    Spread alone is not the goal — a wide but uninformative score is useless.
    Here each combination rule is scored by how well it ranks an object's own
    candidates above the other members of its confusion group.
    """
    variants = combination_variants(df)
    d = df.copy()
    for k, v in variants.items():
        d[k] = v
    groups = {"dolls": DOLLS, "box-like": BOXY}
    rows = []
    for gname, members in groups.items():
        members = [m for m in members if m in set(d.object)]
        for m in members:
            pos = d[d.object == m]
            neg = d[d.object.isin([x for x in members if x != m])]
            for k in variants:
                rows.append({"group": gname, "object": m, "rule": k,
                             "auc": auc(pos[k].dropna(), neg[k].dropna())})
    r = pd.DataFrame(rows)
    piv = r.pivot_table(index="rule", columns="object", values="auc")
    piv["MEAN"] = piv.mean(axis=1)
    piv = piv.sort_values("MEAN", ascending=False)

    fig, ax = plt.subplots(figsize=(1.3 * len(piv.columns) + 5, 0.55 * len(piv) + 3))
    im = ax.imshow(piv.values, vmin=0.3, vmax=1.0, cmap="RdYlGn", aspect="auto")
    ax.set_xticks(range(len(piv.columns)))
    ax.set_xticklabels(piv.columns, rotation=30, ha="right", fontsize=8)
    ax.set_yticks(range(len(piv.index)))
    ax.set_yticklabels(piv.index, fontsize=9)
    for i in range(piv.shape[0]):
        for j in range(piv.shape[1]):
            if not np.isnan(piv.values[i, j]):
                ax.text(j, i, f"{piv.values[i,j]:.2f}", ha="center", va="center",
                        fontsize=8)
    ax.set_title("Which combination rule separates the CONFUSED objects?\n"
                 "AUC of an object's own candidates vs its confusion group "
                 "(sorted by mean)")
    fig.colorbar(im, ax=ax, fraction=.03)
    fig.tight_layout()
    savefig(fig, out, "11_combination_rule_separability.png")
    return piv


def fig_rank_effect(df, out):
    """Does the argmax-semantic slot pick the same candidate the new score would?"""
    rows = []
    for (fi, o), g in df.groupby(["frame_idx", "object"]):
        if len(g) < 2:
            continue
        g = g.copy()
        g["prod"] = g.color_emd_mask * g.appe_b2_mask * g.appe_b9_mask
        rows.append({
            "same": int(g["sem"].idxmax() == g["prod"].idxmax()),
            "n": len(g),
            "object": o,
        })
    if not rows:
        return pd.DataFrame()
    r = pd.DataFrame(rows)
    by = r.groupby("object")["same"].agg(["mean", "count"]).reset_index()
    fig, ax = plt.subplots(figsize=(9, 4.6))
    ax.bar(by.object, by["mean"] * 100, color="#457b9d")
    for i, (m, c) in enumerate(zip(by["mean"], by["count"])):
        ax.text(i, m * 100 + 1, f"n={c}", ha="center", fontsize=8)
    ax.set_ylabel("% frames where argmax-sem == argmax-newscore")
    ax.set_title("Slot selection: does picking by semantic choose the same\n"
                 "candidate the new score would? (multi-candidate frames only)")
    ax.tick_params(axis="x", rotation=25)
    ax.grid(alpha=.3, axis="y")
    fig.tight_layout()
    savefig(fig, out, "10_slot_selection_agreement.png")
    return by


# ---------------------------------------------------------------------------
def main():
    args = parse_args()
    out = args.out
    tdir = os.path.join(out, "tables")
    os.makedirs(tdir, exist_ok=True)

    df = pd.read_csv(args.csv)
    print(f"[data] {len(df)} candidates, {df.frame_idx.nunique()} frames, "
          f"{df.object.nunique()} objects")

    # ---- tables ----
    chans = [c for c in CHANNELS_MASK + ["yolo_conf", "sem_margin",
                                         "sem_entropy", "sem_viewvar",
                                         "color_w1_mask"] if c in df.columns]
    summ = df.groupby("object")[chans].agg(["count", "mean", "std", "min",
                                            "median", "max"])
    summ.to_csv(os.path.join(tdir, "per_object_summary.csv"))

    # NB: use df["sem"], never df.sem — DataFrame.sem() is the standard-error method
    d = df.copy()
    d["prod"] = d["color_emd_mask"] * d["appe_b2_mask"] * d["appe_b9_mask"]

    def _spread(s):
        s = s.dropna()
        return np.percentile(s, 95) - np.percentile(s, 5) if len(s) else np.nan

    comp = pd.DataFrame([{
        "object": o,
        "n": len(g),
        "sem_med": g["sem"].median(),
        "appe_b2_med": g["appe_b2_mask"].median(),
        "appe_b9_med": g["appe_b9_mask"].median(),
        "appe_b11_med": g["appe_b11_mask"].median(),
        "color_emd_med": g["color_emd_mask"].median(),
        "prod_med": g["prod"].median(),
        "appe_b11_p5_p95": _spread(g["appe_b11_mask"]),
        "prod_p5_p95": _spread(g["prod"]),
    } for o, g in d.groupby("object")])
    comp.to_csv(os.path.join(tdir, "per_object_compare.csv"), index=False)
    print("\n=== per-object medians / dynamic range ===")
    print(comp.round(3).to_string(index=False))

    # ---- figures ----
    print("\n[figures]")
    fig_channel_spread(df, out)
    fig_per_object_hist(df, out)
    fig_product_vs_single(df, out)
    fig_color_metric_compare(df, out)
    sep = fig_confusion_separation(df, out)
    sep.to_csv(os.path.join(tdir, "group_separability_auc.csv"), index=False)
    fig_scatter(df, out)
    corr = fig_correlation(df, out)
    corr.to_csv(os.path.join(tdir, "channel_correlation.csv"))
    fig_muse_terms(df, out)
    ov = fig_overlap_crosstalk(df, out)
    if len(ov):
        ov.to_csv(os.path.join(tdir, "cross_object_overlap.csv"), index=False)
    rules = fig_combination_separability(df, out)
    rules.to_csv(os.path.join(tdir, "combination_rule_auc.csv"))
    print("\n=== combination-rule separability (AUC vs confusion group) ===")
    print(rules.round(3).to_string())

    ra = fig_rank_effect(df, out)
    if len(ra):
        ra.to_csv(os.path.join(tdir, "slot_selection_agreement.csv"), index=False)

    print(f"\n[done] tables -> {tdir}\n       figures -> {out}")


if __name__ == "__main__":
    main()
