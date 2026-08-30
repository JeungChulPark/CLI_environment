#!/usr/bin/env python3
"""물체별 회전 오차 타임라인 (full vs rt) 플롯."""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PROBE = Path(__file__).resolve().parent
out_dir = PROBE / "outputs"
full = json.load(open(out_dir / "timeline_full.json"))
rt = json.load(open(out_dir / "timeline_rt.json"))
soft = json.load(open(out_dir / "timeline_rt_soft5.json"))

objects = sorted(set(full) | set(rt))
t0 = min(v[0][0] for v in list(full.values()) + list(rt.values()) if v)

fig, axes = plt.subplots(len(objects), 1, figsize=(11, 2.2 * len(objects)),
                         sharex=True)
if len(objects) == 1:
    axes = [axes]
for ax, name in zip(axes, objects):
    for tl, color, label in ((full.get(name, []), "#2b6cb0", "full 30fps"),
                             (rt.get(name, []), "#c53030", "rt 2fps raw"),
                             (soft.get(name, []), "#2f855a", "rt 2fps + soft K=5")):
        if not tl:
            continue
        ts = [row[0] - t0 for row in tl]
        errs = [row[1] for row in tl]
        ax.step(ts, errs, where="post", color=color, label=label, lw=1.2, alpha=0.85)
        anchored = [(t, e) for t, e, _, s in [(r[0] - t0, r[1], r[2], r[3]) for r in tl]
                    if s == "anchor"]
        if anchored:
            ax.plot([a[0] for a in anchored], [a[1] for a in anchored], ".",
                    color=color, ms=3)
    ax.axhline(20, color="gray", ls="--", lw=0.8)
    ax.set_ylabel("rot err (deg)")
    ax.set_title(name, fontsize=10, loc="left")
    ax.set_ylim(0, 185)
    ax.legend(fontsize=8, loc="upper right")
axes[-1].set_xlabel("bag time (s)")
fig.suptitle("Temporal correction: displayed-pose rotation error vs pseudo-GT "
             "(dots = anchor output, dashes = 20° threshold)")
fig.tight_layout()
p = out_dir / "timelines.png"
fig.savefig(p, dpi=110)
print(f"saved {p}")
