#!/usr/bin/env python3
"""plot_memory_timeline.py — per-object detection timeline and final existence probability.

Replays a finished run through the object memory with the hub's defaults (tune_memory.replay)
and draws, per object, the moments SAM-6D detected it (left) and the landmark's existence
probability and state when the run ended (right, hatched = lost).

    python objpose/pc/plot_memory_timeline.py objpose/output/<run> --out fig.png
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from tune_memory import load_run, replay  # noqa: E402

KO = {"milk": "우유팩", "saffron": "사프란", "Febreze_high": "탈취제", "choco_hazelnut_high": "초코과자",
      "Bear": "곰인형", "Sikhye_high": "식혜캔", "Dinosaur": "공룡인형", "Mugcup_high": "머그컵",
      "sauce": "소스병"}
STATE = {"active": "확정", "lost": "놓침", "remembered": "장기 기억", "tentative": "후보", "deleted": "삭제"}
BLUE, RED = "#1f6fb4", "#c0392b"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--out", required=True)
    ap.add_argument("--pd", type=float, default=0.08)
    ap.add_argument("--clutter", type=float, default=0.01)
    ap.add_argument("--min-obs", type=int, default=5)
    ap.add_argument("--max-range", type=float, default=1.5)
    ap.add_argument("--font", default="/home/jucpark/.local/share/fonts/malgun.ttf")
    a = ap.parse_args()

    font_manager.fontManager.addfont(a.font)
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=a.font).get_name()
    plt.rcParams["axes.unicode_minus"] = False

    run = Path(a.run)
    frames, K, size, _ = load_run(run)
    r = replay(frames, K, size, pd_base=a.pd, clutter_ratio=a.clutter, min_obs_longterm=a.min_obs,
               pd_max_range_m=a.max_range or None)
    t0 = frames[0][0]
    t_end = (frames[-1][0] - t0) / 1e9
    seen: dict[str, list[float]] = {}
    for t, _, rows in frames:
        for n, _, _ in rows:
            seen.setdefault(n, []).append((t - t0) / 1e9)

    final = {}
    for lm in r["landmarks"]:
        cur = final.get(lm["name"])
        rank = (lm["status"] in ("active", "lost", "remembered"), lm["n_obs"])
        if cur is None or rank > cur[0]:
            final[lm["name"]] = (rank, lm)
    names = sorted(seen, key=lambda n: min(seen[n]))
    names = [n for n in names if n in final]

    fig, (ax, bx) = plt.subplots(1, 2, figsize=(11.85, 4.27), dpi=100,
                                 gridspec_kw={"width_ratios": [2.3, 1]})
    for i, n in enumerate(names):
        y = len(names) - 1 - i
        lm = final[n][1]
        lost = lm["status"] in ("lost", "deleted")
        col = RED if lost else BLUE
        first, last = min(seen[n]), max(seen[n])
        ax.plot([first, last], [y, y], color="#dddddd", lw=9, solid_capstyle="butt", zorder=1)
        ax.plot([last, t_end], [y, y], color=("#e3c3c0" if lost else "#c3cfdc"), lw=9,
                solid_capstyle="butt", zorder=1)
        ax.vlines(seen[n], y - 0.3, y + 0.3, color=col, lw=2, zorder=2)
        label = KO.get(n, n) + (" (놓침)" if lost else "")
        ax.text(-0.02 * t_end, y, label, ha="right", va="center", fontsize=18)
        conf = lm["r"]
        bx.barh(y, conf, height=0.5, color=("white" if lost else BLUE), edgecolor=col,
                hatch=("//" if lost else None), lw=0 if not lost else 0)
        if lost:
            bx.barh(y, conf, height=0.5, color=RED, hatch="//", edgecolor="white", lw=0)
        bx.text(conf + 0.02, y, f"{conf:.2f} {STATE.get(lm['status'], lm['status'])}",
                va="center", fontsize=17, color=(RED if lost else "#333333"))
    ax.set_yticks([]); ax.set_ylim(-1.1, len(names) - 0.4); ax.set_xlim(0, t_end * 1.01)
    ax.set_xlabel("주행 시간 [s]", fontsize=18); ax.set_title("본 순간과 마지막으로 본 뒤", fontsize=20)
    ax.tick_params(labelsize=16); ax.grid(axis="x", color="#eeeeee")
    for s in ("left", "right", "top"):
        ax.spines[s].set_visible(False)
    bx.axvline(0.3, color="#555555", ls=":", lw=1.2); bx.axvline(0.6, color="#555555", ls="--", lw=1.2)
    bx.text(0.3, -1.0, "놓침", ha="center", va="bottom", fontsize=15, color="#555555")
    bx.text(0.6, -1.0, "확정", ha="center", va="bottom", fontsize=15, color="#555555")
    bx.set_xlim(0, 1.62); bx.set_ylim(-1.1, len(names) - 0.4); bx.set_yticks(range(len(names)))
    bx.set_yticklabels([]); bx.set_xticks([0, 0.3, 0.6, 1.0]); bx.tick_params(labelsize=16)
    bx.set_xlabel("주행이 끝난 시점의 존재확률", fontsize=18); bx.set_title("빗금이 놓침" if any(final[n][1]["status"] in ("lost", "deleted") for n in names) else "주행이 끝난 시점의 상태", fontsize=20)
    bx.grid(axis="x", color="#eeeeee")
    fig.tight_layout()
    fig.subplots_adjust(left=0.13)
    fig.savefig(a.out, dpi=100)
    for n in names:
        lm = final[n][1]
        print(f"{n:22s} obs {len(seen[n]):4d}  last {max(seen[n]):6.1f}s  r {lm['r']:.2f}  {lm['status']}")
    print("t_end", round(t_end, 1))


if __name__ == "__main__":
    main()
