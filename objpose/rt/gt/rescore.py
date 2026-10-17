#!/usr/bin/env python3
"""rescore.py — score the stored ORB-SLAM3 runs (base vs gyro-aided) against the old KISS-ICP reference and the
new pseudo-GT (pgo_tum.txt), with the same scorer (rt/260901/bigeight/eval_traj.py) and the same rig.

    python3 rescore.py --runs <dir with base_run*_tum.txt ...> --kiss <old kiss_tum.txt> --pgo <pgo_tum.txt> --out <json>
                       [--refit]   # also re-fit T_lidar_cam / time offset against the pseudo-GT (hand-eye, base_run1 opt)
"""
from __future__ import annotations
import argparse, json, sys, types
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[3]
BIG = REPO / "objpose" / "rt" / "260901" / "bigeight"
sys.path.insert(0, str(BIG))
import eval_traj  # noqa: E402
from trajlib import load_tum  # noqa: E402


def score(rig, ref, runs):
    X = np.array(rig["T_lidar_cam"]); tau = rig["time"]["lidar_minus_cam_s"]
    tl, Tl = load_tum(ref)
    tg = tl - tau
    a = types.SimpleNamespace(anchor_s=15.0, rpe_s=[1.0, 10.0])
    out = {}
    for name, path in runs:
        r, _ = eval_traj.evaluate(name, path, tg, Tl @ X, np.array([0.0, 0.0, 1.0]), a)
        out[name] = {"ate_cm": r["ate"]["pos_cm"]["rmse"], "ate_max_cm": r["ate"]["pos_cm"]["max"],
                     "ate_rot_deg": r["ate"]["rot_deg"]["rmse"],
                     "drift_cm": r["drift"]["pos_cm"]["rmse"], "yaw_rmse_deg": r["drift"]["yaw_deg"]["rmse"],
                     "tilt_rmse_deg": r["drift"]["tilt_deg"]["rmse"],
                     "rpe1_cm": r["rpe_1s"]["trans_cm"]["rmse"], "rpe10_cm": r["rpe_10s"]["trans_cm"]["rmse"]}
    return out


def refit(rig, ref, cam, out_json):
    import calib_rig
    a = types.SimpleNamespace(win_s=2.0, out=str(out_json))
    tc, Tc = load_tum(cam); tl, Tl = load_tum(ref)
    o = {"cam": str(cam), "lidar": str(ref)}
    import contextlib, io
    with contextlib.redirect_stdout(io.StringIO()):
        calib_rig.lidar_only(a, tc, Tc, tl, Tl, o)
    new = dict(rig); new["T_lidar_cam"] = o["T_lidar_cam"]; new["time"] = dict(rig["time"])
    new["time"]["lidar_minus_cam_s"] = o["time"]["lidar_minus_cam_s"]
    new["T_lidar_cam_fit"] = o["T_lidar_cam_fit"]
    Path(out_json).write_text(json.dumps(new, indent=1))
    return new


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", required=True); ap.add_argument("--kiss", required=True); ap.add_argument("--pgo", required=True)
    ap.add_argument("--rig", default=str(BIG / "rig_260901_bigeight.json")); ap.add_argument("--out", required=True)
    ap.add_argument("--refit", action="store_true")
    a = ap.parse_args()
    rig = json.loads(Path(a.rig).read_text())
    d = Path(a.runs)
    runs = []
    for f in sorted(d.glob("*_run*_tum.txt")):
        v, i = f.name.split("_run")[0], f.name.split("_run")[1].split("_")[0]
        runs.append((f"{v}_live{i}", f))
        if Path(str(f) + ".optimized.txt").exists():
            runs.append((f"{v}_opt{i}", Path(str(f) + ".optimized.txt")))
    res = {"runs_dir": str(d), "kiss": score(rig, a.kiss, runs), "pgo": score(rig, a.pgo, runs)}
    if a.refit:
        cam = d / "base_run1_tum.txt.optimized.txt"
        r_k = refit(rig, a.kiss, cam, Path(a.out).with_suffix(".rig_kiss.json"))
        r_p = refit(rig, a.pgo, cam, Path(a.out).with_suffix(".rig_pgo.json"))
        res["kiss_refit"] = score(r_k, a.kiss, runs); res["pgo_refit"] = score(r_p, a.pgo, runs)
        res["refit_fit"] = {"kiss": r_k["T_lidar_cam_fit"], "pgo": r_p["T_lidar_cam_fit"]}
    Path(a.out).write_text(json.dumps(res, indent=1))
    keys = [k for k in ("kiss", "pgo", "kiss_refit", "pgo_refit") if k in res]
    print(f"{'run':12s} " + " | ".join(f"{k:>34s}" for k in keys))
    print(f"{'':12s} " + " | ".join(f"{'ATE  max  drift  yaw°  RPE1s':>34s}" for _ in keys))
    for name, _ in runs:
        print(f"{name:12s} " + " | ".join(
            f"{res[k][name]['ate_cm']:6.2f} {res[k][name]['ate_max_cm']:6.1f} {res[k][name]['drift_cm']:6.1f} "
            f"{res[k][name]['yaw_rmse_deg']:5.2f} {res[k][name]['rpe1_cm']:5.2f}" for k in keys))
    # means per variant
    print("-- mean per variant")
    groups = sorted({n.rstrip("0123456789") for n, _ in runs})
    for g in groups:
        ns = [n for n, _ in runs if n.rstrip("0123456789") == g]
        print(f"{g:12s} " + " | ".join(
            f"{np.mean([res[k][n]['ate_cm'] for n in ns]):6.2f} {np.mean([res[k][n]['ate_max_cm'] for n in ns]):6.1f} "
            f"{np.mean([res[k][n]['drift_cm'] for n in ns]):6.1f} {np.mean([res[k][n]['yaw_rmse_deg'] for n in ns]):5.2f} "
            f"{np.mean([res[k][n]['rpe1_cm'] for n in ns]):5.2f}" for k in keys))


if __name__ == "__main__":
    main()
