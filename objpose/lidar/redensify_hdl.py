#!/usr/bin/env python3
"""redensify_hdl.py — measure densification variants for hdl_graph_slam trajectories.

hdl_graph_slam stores optimised keyframes (~0.5-1.2 Hz); trajectory_dense.txt (~9.9 Hz) is
scan-matching odometry with keyframe corrections corr_k = T_map_kf . T_odom_kf^-1 applied. The
original densifier is not available, so the odometry is recovered from the dense track itself: the
recovery that leaves the odometry smooth across keyframe switches identifies how corrections were
applied (time-interpolated, see analysis), and that odometry is re-densified with:

  existing   the stored trajectory_dense.txt
  step       correction of the last keyframe (piecewise constant)
  time       SE(3) interpolation of the corrections by time between keyframes
  distance   SE(3) interpolation by travelled odometry distance between keyframes
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "integration"))
sys.path.insert(0, str(HERE.parents[1] / "objpose" / "pc"))
sys.path.insert(0, str(HERE))
from analyze_hdl_wobble import plane_axes, smooth_residual, yaw_rate  # noqa: E402
from fusion import interp_se3  # noqa: E402
from rig_offset import read_tum  # noqa: E402


def corr_at(tk, corr, x, xk):
    """interpolate corrections at coordinate x (time or distance) given keyframe coordinates xk"""
    k = np.searchsorted(xk, x, side="right") - 1
    if k < 0:
        return corr[0]
    if k >= len(xk) - 1:
        return corr[-1]
    span = xk[k + 1] - xk[k]
    u = 0.0 if span <= 1e-9 else (x - xk[k]) / span
    return interp_se3(corr[k], corr[k + 1], float(np.clip(u, 0, 1)))


def metrics(ts, T, tk):
    P = T[:, :3, 3]
    e1, e2, n = plane_axes(P)
    r = smooth_residual(ts, P, 1.0)
    h = np.hypot(r @ e1, r @ e2) * 100
    v = np.abs(r @ n) * 100
    acc = np.zeros(len(P))
    acc[1:-1] = np.linalg.norm(P[2:] - 2 * P[1:-1] + P[:-2], axis=1) * 100
    kid = np.clip(np.searchsorted(ts, tk), 1, len(ts) - 2)
    near = np.zeros(len(P), bool)
    for k in kid:
        near[max(k - 1, 1):k + 2] = True
    yr = yaw_rate(ts, T, e1, e2)
    return {"wobble_h_rms_cm": round(float(np.sqrt(np.mean(h ** 2))), 3),
            "wobble_v_rms_cm": round(float(np.sqrt(np.mean(v ** 2))), 3),
            "accel_at_kf_cm": round(float(np.median(acc[near])), 3),
            "accel_elsewhere_cm": round(float(np.median(acc[~near][1:-1])), 3),
            "wobble_h_fast_turn_cm": round(float(np.median(h[yr > 20])), 3) if (yr > 20).any() else None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/mnt/d/CLI_environment/output_slam/260724_chungbuk/hdl_graph_slam")
    ap.add_argument("--out", default=str(HERE / "redensify_hdl_260724"))
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    summary = {}
    for sd in sorted(Path(a.root).iterdir()):
        if not (sd / "trajectory_dense.txt").exists():
            continue
        ts, T = read_tum(sd / "trajectory_dense.txt")
        tk, Tk = read_tum(sd / "trajectory.txt")
        _, To = read_tum(sd / "odometry.txt")
        nk = min(len(Tk), len(To))
        tk = tk[:nk]
        corr = [Tk[i] @ np.linalg.inv(To[i]) for i in range(nk)]
        # recover odometry assuming time-interpolated corrections (the smooth recovery)
        O = np.array([np.linalg.inv(corr_at(tk, corr, t, tk)) @ T[i] for i, t in enumerate(ts)])
        dist = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(O[:, :3, 3], axis=0), axis=1))])
        dk = np.interp(tk, ts, dist)
        ids = np.clip(np.searchsorted(tk, ts, side="right") - 1, 0, nk - 1)
        variants = {
            "existing": T,
            "step": np.array([corr[ids[i]] @ O[i] for i in range(len(ts))]),
            "time": np.array([corr_at(tk, corr, t, tk) @ O[i] for i, t in enumerate(ts)]),
            "distance": np.array([corr_at(tk, corr, dist[i], dk) @ O[i] for i in range(len(ts))]),
        }
        row = {}
        for name, V in variants.items():
            row[name] = metrics(ts, V, tk)
            if name == "distance":
                from scipy.spatial.transform import Rotation
                with open(out / f"{sd.name}_dense_distance.txt", "w") as f:
                    for t, M in zip(ts, V):
                        q = Rotation.from_matrix(M[:3, :3]).as_quat()
                        f.write(f"{t:.9f} {M[0,3]:.6f} {M[1,3]:.6f} {M[2,3]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")
        d_te = np.linalg.norm(variants["time"][:, :3, 3] - T[:, :3, 3], axis=1) * 100
        row["time_vs_existing_max_cm"] = round(float(d_te.max()), 4)
        d_de = np.linalg.norm(variants["distance"][:, :3, 3] - T[:, :3, 3], axis=1) * 100
        row["distance_vs_existing_cm"] = {"median": round(float(np.median(d_de)), 3), "max": round(float(d_de.max()), 3)}
        summary[sd.name] = row
        print(sd.name, json.dumps(row))
    (out / "summary.json").write_text(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
