#!/usr/bin/env python3
"""eval_lidar_variants.py — score KISS-ICP variants (kiss_variants.py) and smoothing on 260910_object.

Same metrics as smooth_lidar_traj.py: per-scan acceleration, 1 s wobble, high-frequency disagreement with
ORB-SLAM3 final (as the SLAM-folder camera; one Sim(3) alignment fitted on the base run, shared by all
variants), and the spread of the live run's SAM-6D estimates re-placed with each trajectory. "Spike scans" are
the top 1% acceleration scans of the base run.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "objpose" / "pc"))
sys.path.insert(0, str(ROOT / "integration"))
from analyze_hdl_wobble import plane_axes, smooth_residual, yaw_rate  # noqa: E402
from fusion import interp_se3, mat  # noqa: E402
from rig_offset import read_tum  # noqa: E402
from smooth_lidar_traj import accel, umeyama_sim3, window_smooth  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=str(HERE / "kiss_variants_260910"))
    ap.add_argument("--run", default=str(ROOT / "objpose/output/live_260910_lidar_v3"))
    ap.add_argument("--orb", default=str(ROOT / "objpose/rt/260910/slam_orb_tum.txt.optimized.txt"))
    ap.add_argument("--out", default=str(HERE / "eval_lidar_variants_260910.json"))
    ap.add_argument("--no-derived", action="store_true", help="score only the *_tum.txt files (no added box5 variants)")
    a = ap.parse_args()
    rt = ROOT / "objpose/rt/260910"
    T_lidar_cam = np.linalg.inv(np.asarray(json.loads((rt / "T_cam_lidar_final.json").read_text())["T_cam_lidar"]))
    xj = json.loads((rt / "X_lidar_sam.json").read_text())
    X, tau = np.asarray(xj["T_slam_sam"]), float(xj.get("sam_tau_s", 0.0))
    to, To = read_tum(a.orb)
    est = [json.loads(l) for l in open(Path(a.run) / "sam6d_estimates.jsonl")]
    est = [e for e in est if e.get("placed")]

    d = Path(a.dir)
    trajs = {}
    trajs["base"] = read_tum(d / "base_tum.txt")
    for f in sorted(d.glob("*_tum.txt")):
        if f.name != "base_tum.txt":
            trajs[f.name[:-len("_tum.txt")]] = read_tum(f)
    ts_b, T_b = trajs["base"]
    if not a.no_derived:
        trajs["base+box5"] = (ts_b, window_smooth(T_b, [1] * 5))
    for k in (() if a.no_derived else ("iterdeskew", "iter2", "iterdamp")):
        if k in trajs:
            trajs[k + "+box5"] = (trajs[k][0], window_smooth(trajs[k][1], [1] * 5))

    acc_b = accel(T_b[:, :3, 3])
    spike_t = ts_b[acc_b > np.percentile(acc_b, 99)]
    e1, e2, _ = plane_axes(T_b[:, :3, 3])
    fit = None
    res = {"spike_scans": int(len(spike_t))}
    for name, (ts, T) in trajs.items():
        P = T[:, :3, 3]
        acc = accel(P)
        spike = np.isin(np.round(ts, 6), np.round(spike_t, 6))
        r = smooth_residual(ts, P, 1.0)
        wob = np.hypot(r @ e1, r @ e2) * 100
        ok = (ts > to[0]) & (ts < to[-1])
        Po = np.array([np.interp(ts[ok], to, To[:, j, 3]) for j in range(3)]).T
        Pc = np.array([(M @ T_lidar_cam)[:3, 3] for M in T])[ok]
        if fit is None:
            fit = umeyama_sim3(Pc, Po)
        s, Rm, t = fit
        dd = (s * Pc @ Rm.T + t) - Po
        hf = np.linalg.norm(smooth_residual(ts[ok], dd, 1.0), axis=1) * 100
        yr = yaw_rate(ts, T, e1, e2)[ok]

        def pose_at(tq):
            k = int(np.clip(np.searchsorted(ts, tq), 1, len(ts) - 1))
            u = float(np.clip((tq - ts[k - 1]) / (ts[k] - ts[k - 1]), 0, 1))
            return interp_se3(T[k - 1], T[k], u)
        by = {}
        for e in est:
            Tco = np.eye(4); Tco[:3, :3] = np.asarray(e["R"]); Tco[:3, 3] = np.asarray(e["t_mm"]) / 1000
            by.setdefault(e["object"], []).append((pose_at(e["t_ns"] / 1e9 + tau) @ X @ Tco)[:3, 3])
        spread = np.concatenate([np.linalg.norm(np.array(L) - np.median(L, 0), axis=1) for L in by.values()]) * 100
        row = {"accel_cm": {"median": round(float(np.median(acc[1:-1])), 3), "p99": round(float(np.percentile(acc[1:-1], 99)), 2),
                            "max": round(float(acc.max()), 2), "at_base_spikes_median": round(float(np.median(acc[spike])), 2)},
               "wobble_h_rms_cm": round(float(np.sqrt(np.mean(wob ** 2))), 3),
               "hf_vs_orb_cm": {"rms": round(float(np.sqrt(np.mean(hf ** 2))), 3),
                                "at_base_spikes_rms": round(float(np.sqrt(np.mean(hf[spike[ok]] ** 2))), 3),
                                "fast_turn_gt40dps_rms": round(float(np.sqrt(np.mean(hf[yr > 40] ** 2))), 3)},
               "lowfreq_vs_orb_rmse_cm": round(float(np.sqrt(np.mean(np.sum(dd ** 2, 1)))) * 100, 2),
               "object_spread_cm": {"median": round(float(np.median(spread)), 3), "p90": round(float(np.percentile(spread, 90)), 3)}}
        res[name] = row
        print(f"{name:16s} {json.dumps(row)}", flush=True)
    Path(a.out).write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
