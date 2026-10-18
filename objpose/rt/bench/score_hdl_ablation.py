#!/usr/bin/env python3
"""score_hdl_ablation.py — score the run_hdl_ablation.sh runs against the LiDAR pseudo-GT, velodyne frame.

Per run three trajectories:
  live   per-scan pose as streamed (map->odom . odom at that moment)
  final  per-scan pose re-anchored to its keyframe's last graph estimate (hub_record.py)
  kf     the optimised graph keyframes alone (hdl_stream --dump-dir keyframes_tum.txt)
Also the earlier run_bench.sh run (out/<s>/hdl/run1, floor on) as a repeat of the floor-on setting.
Scorer: eval_traj.evaluate (SE(3) ATE, rotation after mean offset, RPE 1 s / 10 s, drift tilt).

    python3 score_hdl_ablation.py [--out-csv hdl_ablation.csv]
"""
from __future__ import annotations
import argparse, csv, sys, types
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "objpose/rt/260901/bigeight"))
import eval_traj  # noqa: E402
from trajlib import load_tum  # noqa: E402

SESSIONS = ("260901_cbnu_bigeightcircle", "260901_cbnu_eightcircle", "260901_cbnu_digut", "260901_cbnu_longcircle")
FILES = {"live": "live_tum.txt", "final": "final_tum.txt", "kf": "kf_tum.txt"}


def g(d, *k):
    for x in k:
        if not isinstance(d, dict) or x not in d:
            return float("nan")
        d = d[x]
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-csv", default=str(HERE / "hdl_ablation.csv"))
    a = ap.parse_args()
    opts = types.SimpleNamespace(anchor_s=15.0, rpe_s=[1.0, 10.0])
    rows = []
    for s in SESSIONS:
        tg, Tg = load_tum(REPO / f"objpose/rt/gt/out/{s}/pgo_tum.txt")
        runs = [("floor_on_prev", HERE / f"out/{s}/hdl/run1")]
        for fl in ("on", "off"):
            runs += [(f"floor_{fl}", r) for r in sorted((HERE / f"out/{s}/hdl_floor_{fl}").glob("run*"))]
        for name, r in runs:
            for kind, fn in FILES.items():
                f = r / fn
                if not f.exists() or f.stat().st_size == 0:
                    continue
                try:
                    res, _ = eval_traj.evaluate(f"{name}_{kind}", str(f), tg, Tg, np.array([0.0, 0.0, 1.0]), opts)
                except Exception as e:  # noqa: BLE001
                    rows.append({"session": s, "config": name, "run": r.name, "traj": kind, "error": str(e)[:80]})
                    continue
                rows.append({"session": s, "config": name, "run": r.name, "traj": kind, "poses": res["poses"],
                             "ate_cm": g(res, "ate", "pos_cm", "rmse"), "ate_max_cm": g(res, "ate", "pos_cm", "max"),
                             "rot_deg": g(res, "ate", "rot_deg", "rmse"),
                             "tilt_deg": g(res, "drift", "tilt_deg", "rmse"),
                             "rpe1_cm": g(res, "rpe_1s", "trans_cm", "rmse"), "rpe1_deg": g(res, "rpe_1s", "rot_deg", "rmse"),
                             "rpe10_cm": g(res, "rpe_10s", "trans_cm", "rmse"), "coverage": res["coverage"]})
    keys = ["session", "config", "run", "traj", "poses", "ate_cm", "ate_max_cm", "rot_deg", "tilt_deg", "rpe1_cm",
            "rpe1_deg", "rpe10_cm", "coverage", "error"]
    with open(a.out_csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore"); w.writeheader(); w.writerows(rows)
    print(f"{'session':28s} {'config':14s} {'traj':5s} {'n':>6s} {'ATE':>6s} {'max':>6s} {'rot':>5s} {'tilt':>5s} "
          f"{'RPE1':>6s} {'RPE1°':>5s} {'RPE10':>6s}")
    for r in rows:
        if "error" in r:
            print(f"{r['session']:28s} {r['config']:14s} {r['traj']:5s} ERROR {r['error']}"); continue
        print(f"{r['session']:28s} {r['config']:14s} {r['traj']:5s} {r['poses']:6d} {r['ate_cm']:6.2f} {r['ate_max_cm']:6.2f} "
              f"{r['rot_deg']:5.2f} {r['tilt_deg']:5.2f} {r['rpe1_cm']:6.2f} {r['rpe1_deg']:5.2f} {r['rpe10_cm']:6.2f}")
    print(f"wrote {a.out_csv}")


if __name__ == "__main__":
    main()
