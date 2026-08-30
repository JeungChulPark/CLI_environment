#!/usr/bin/env python3
"""new_score_color_review.py — does the colour score do what it is supposed to?

The colour score answers ONE question, per object:

    how close is this YOLO candidate's colour to THIS object's rendered
    template colour?

That is a per-object number (candidate vs its own templates) — it is not a
10-way object-vs-object comparison. There are no TP/FP labels for this bag, so
the honest way to check the score is to LOOK at it: for each object, show the
candidates it scored highest and lowest, next to the object's own templates.

If the score works, the high-scoring crops should look like the template's
colour and the low-scoring ones should not.

Outputs, per object:
    color_review/<object>.png   template swatches + top-N and bottom-N crops
    color_review/_hist_<object>.png  the score distribution with those picks marked
"""
import argparse
import glob
import os

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                        # noqa: E402
import pandas as pd                       # noqa: E402

N_SHOW = 8
THUMB = 128


def load_crop(frames_root, frame_name, box, pad=0.06):
    p = os.path.join(frames_root, frame_name, "rgb.png")
    img = cv2.imread(p)
    if img is None:
        return None
    h, w = img.shape[:2]
    x1, y1, x2, y2 = box
    dx, dy = int((x2 - x1) * pad), int((y2 - y1) * pad)
    x1, y1 = max(0, x1 - dx), max(0, y1 - dy)
    x2, y2 = min(w, x2 + dx), min(h, y2 + dy)
    if x2 <= x1 or y2 <= y1:
        return None
    c = img[y1:y2, x1:x2]
    s = THUMB / max(c.shape[0], c.shape[1])
    c = cv2.resize(c, (max(1, int(c.shape[1] * s)), max(1, int(c.shape[0] * s))))
    canvas = np.full((THUMB, THUMB, 3), 40, np.uint8)
    oy, ox = (THUMB - c.shape[0]) // 2, (THUMB - c.shape[1]) // 2
    canvas[oy:oy + c.shape[0], ox:ox + c.shape[1]] = c
    return cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)


def template_thumbs(template_dir, n=6):
    rgbs = sorted(glob.glob(os.path.join(template_dir, "rgb_*.png")))
    if not rgbs:
        return []
    pick = np.linspace(0, len(rgbs) - 1, min(n, len(rgbs))).astype(int)
    out = []
    for i in pick:
        img = cv2.imread(rgbs[i])
        if img is None:
            continue
        mp = os.path.join(template_dir,
                          "mask_" + os.path.basename(rgbs[i]).split("_")[1])
        if os.path.isfile(mp):
            m = cv2.imread(mp, 0)
            ys, xs = np.where(m > 0)
            if len(xs):
                img = img[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
        s = THUMB / max(img.shape[0], img.shape[1])
        img = cv2.resize(img, (max(1, int(img.shape[1] * s)),
                               max(1, int(img.shape[0] * s))))
        canvas = np.full((THUMB, THUMB, 3), 40, np.uint8)
        oy, ox = (THUMB - img.shape[0]) // 2, (THUMB - img.shape[1]) // 2
        canvas[oy:oy + img.shape[0], ox:ox + img.shape[1]] = img
        out.append(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--frames", required=True)
    ap.add_argument("--config", default=None)
    ap.add_argument("--out", required=True)
    ap.add_argument("--metric", default="color_emd_mask")
    a = ap.parse_args()

    rdir = os.path.join(a.out, "color_review")
    os.makedirs(rdir, exist_ok=True)
    df = pd.read_csv(a.csv)

    tdirs = {}
    if a.config:
        import yaml
        cfg = yaml.safe_load(open(a.config))
        root = os.path.dirname(os.path.dirname(os.path.abspath(a.config)))
        for o in cfg.get("objects", []):
            if o.get("enabled", True) and o.get("template_dir"):
                td = o["template_dir"]
                tdirs[o["name"]] = td if os.path.isabs(td) else \
                    os.path.join(root, td)

    for obj, g in df.groupby("object"):
        g = g.dropna(subset=[a.metric]).sort_values(a.metric, ascending=False)
        if len(g) < 4:
            continue
        top = g.head(N_SHOW)
        bot = g.tail(N_SHOW).iloc[::-1]

        tt = template_thumbs(tdirs.get(obj, ""), 6) if obj in tdirs else []
        nrow = 2 + (1 if tt else 0)
        fig, axes = plt.subplots(nrow, N_SHOW,
                                 figsize=(1.55 * N_SHOW, 1.75 * nrow))
        axes = np.atleast_2d(axes)
        r = 0
        if tt:
            for j in range(N_SHOW):
                ax = axes[r, j]; ax.axis("off")
                if j < len(tt):
                    ax.imshow(tt[j])
            axes[r, 0].set_ylabel("TEMPLATE")
            axes[r, 0].axis("on"); axes[r, 0].set_xticks([]); axes[r, 0].set_yticks([])
            r += 1

        for label, sub, color in (("HIGHEST colour score", top, "#2a9d8f"),
                                  ("LOWEST colour score", bot, "#e63946")):
            for j in range(N_SHOW):
                ax = axes[r, j]; ax.axis("off")
                if j >= len(sub):
                    continue
                row = sub.iloc[j]
                crop = load_crop(a.frames, row["frame_name"],
                                 [int(row.x1), int(row.y1),
                                  int(row.x2), int(row.y2)])
                if crop is not None:
                    ax.imshow(crop)
                ax.set_title(f"{row[a.metric]:.3f}", fontsize=8, color=color)
            axes[r, 0].axis("on"); axes[r, 0].set_xticks([]); axes[r, 0].set_yticks([])
            axes[r, 0].set_ylabel(label, fontsize=7, color=color)
            r += 1

        fig.suptitle(f"{obj} — colour score ({a.metric}) vs its OWN templates\n"
                     f"n={len(g)} candidates, median={g[a.metric].median():.3f}",
                     fontsize=11)
        fig.tight_layout()
        fig.savefig(os.path.join(rdir, f"{obj}.png"), dpi=130,
                    bbox_inches="tight")
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(7, 3.2))
        ax.hist(g[a.metric], bins=50, color="#264653")
        for v in top[a.metric]:
            ax.axvline(v, color="#2a9d8f", lw=.8)
        for v in bot[a.metric]:
            ax.axvline(v, color="#e63946", lw=.8)
        ax.set_title(f"{obj}: colour score distribution "
                     f"(green = shown as highest, red = lowest)", fontsize=9)
        ax.set_xlabel(a.metric); ax.grid(alpha=.3)
        fig.tight_layout()
        fig.savefig(os.path.join(rdir, f"_hist_{obj}.png"), dpi=130,
                    bbox_inches="tight")
        plt.close(fig)
        print(f"  {obj:22s} n={len(g):5d}  median={g[a.metric].median():.3f}")

    print(f"\n[done] -> {rdir}")


if __name__ == "__main__":
    main()
