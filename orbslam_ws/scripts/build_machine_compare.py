#!/usr/bin/env python3
"""
Fill the WSL2-vs-Mac comparison page with measured numbers.

The Mac side is read from four runs of orbslam_ws/mac_harness (baseline, A, B, C), each a
directory holding TrackPerFrame.csv, TrackingTimeStats.txt, CameraTrajectory.txt and
run.log. The WSL2 side has no per-frame data in this checkout, so it is the summary
published in slam_comparison_report/realtime_30fps/NF2000_REALTIME.md, copied below with
the section each value came from. Values that document does not report stay None and
render as a dash.

Stdlib only, so it runs on a machine without numpy:

    python3 orbslam_ws/scripts/build_machine_compare.py --runs <dir with baseline/A/B/C> \
        --page slam_comparison_report/realtime_30fps/viz/machines/index.html
"""

import argparse
import json
import math
import re
import sys
from pathlib import Path

CONFIGS = ["baseline", "A", "B", "C"]
LABELS = {
    "baseline": "Baseline (unpatched)",
    "A": "A · result-preserving fixes",
    "B": "B · A + fewer iterations",
    "C": "C · B + early exit",
}
BUDGET_MS = 1000.0 / 30.0

# NF2000_REALTIME.md §4 (latency), §1 and §7-1 (frame counts), §5 (accuracy)
WSL2 = {
    "baseline": dict(mean=24.68, p95=31.17, p99=35.64, max=43.64, over=170, frames=7142,
                     closure_cm=26.6, path_m=36.16, kf=160, mp=19855),
    "A": dict(mean=16.34, p95=20.01, p99=21.71, max=30.86, over=0, frames=None,
              closure_cm=26.7, path_m=36.14, kf=170, mp=20715),
    "B": dict(mean=16.37, p95=20.25, p99=22.12, max=39.51, over=1, frames=None,
              closure_cm=None, path_m=None, kf=None, mp=None),
    "C": dict(mean=15.38, p95=18.80, p99=20.34, max=27.03, over=0, frames=7144,
              closure_cm=26.5, path_m=36.44, kf=164, mp=20255),
}
# NF2000_REALTIME.md §2 (baseline stages) and §4 (A stages); "other" is what the three
# timed stages leave of the per-frame total
WSL2_STAGES = {
    "baseline": dict(ext=8.46, pred=4.41, lm=8.95, total=24.68),
    "A": dict(ext=3.63, pred=3.85, lm=7.51, total=16.34),
}


def pct(sorted_v, p):
    return sorted_v[max(0, math.ceil(p / 100 * len(sorted_v)) - 1)]


def mac_run(d):
    rows = [l.split(",") for l in (d / "TrackPerFrame.csv").read_text().splitlines()[1:]]
    t = sorted(float(r[2]) for r in rows)
    log = (d / "run.log").read_text(errors="replace")
    kfs = re.findall(r"KFs in map: (\d+)", log)
    mps = re.findall(r"MPs in map: (\d+)", log)
    P = [tuple(map(float, l.split()[1:4])) for l in (d / "CameraTrajectory.txt").read_text().splitlines()
         if len(l.split()) == 8]
    run = dict(
        mean=round(sum(t) / len(t), 2), p95=round(pct(t, 95), 2), p99=round(pct(t, 99), 2),
        max=round(t[-1], 2), over=sum(v > BUDGET_MS for v in t), frames=len(t),
        closure_cm=round(math.dist(P[0], P[-1]) * 100, 1),
        path_m=round(sum(math.dist(a, b) for a, b in zip(P, P[1:])), 2),
        kf=int(kfs[-1]) if kfs else None, mp=int(mps[-1]) if mps else None,
    )

    # REGISTER_TIMES rows whose per-stage vectors ran short read stale memory; drop any
    # non-physical row (the same filter NF2000_REALTIME.md §7 describes)
    ext, pred, lm, tot = [], [], [], []
    for l in (d / "TrackingTimeStats.txt").read_text().splitlines()[1:]:
        v = [float(x) for x in l.split(",")]
        if len(v) != 9 or any(math.isnan(x) or x < 0 or x > 1000 for x in v):
            continue
        ext.append(v[2]); pred.append(v[5]); lm.append(v[6]); tot.append(v[8])
    mean = lambda a: sum(a) / len(a)
    stages = dict(ext=mean(ext), pred=mean(pred), lm=mean(lm), total=mean(tot))
    return run, stages


def with_other(s):
    s = dict(s)
    s["other"] = s["total"] - s["ext"] - s["pred"] - s["lm"]
    return {k: round(v, 2) for k, v in s.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", required=True, help="directory holding baseline/ A/ B/ C/ Mac runs")
    ap.add_argument("--page", required=True)
    args = ap.parse_args()

    mac, mac_st = {}, {}
    for c in CONFIGS:
        mac[c], mac_st[c] = mac_run(Path(args.runs) / c)

    data = dict(
        configs=CONFIGS, config_labels=LABELS, budget_ms=round(BUDGET_MS, 2),
        runs=dict(wsl2=WSL2, mac=mac),
        stages=dict(wsl2={c: with_other(s) for c, s in WSL2_STAGES.items()},
                    mac={c: with_other(s) for c, s in mac_st.items()}),
    )
    page = Path(args.page)
    html = page.read_text()
    block = re.compile(r'(<script id="data" type="application/json">)(.*?)(</script>)', re.S)
    if not block.search(html):
        sys.exit(f"{page}: no data block")
    html = block.sub(lambda m: m.group(1) + json.dumps(data, separators=(",", ":")) + m.group(3), html, count=1)
    page.write_text(html)

    for c in CONFIGS:
        w, m = WSL2[c], mac[c]
        print(f"{c:8s} WSL2 mean {w['mean']:5.2f} p99 {w['p99']:5.2f} over {w['over']:3d} | "
              f"Mac mean {m['mean']:5.2f} p99 {m['p99']:5.2f} over {m['over']} "
              f"({w['mean']/m['mean']:.2f}x / {w['p99']/m['p99']:.2f}x)")
    print(f"wrote data into {page}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
