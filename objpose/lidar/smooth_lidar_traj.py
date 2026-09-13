#!/usr/bin/env python3
"""smooth_lidar_traj.py — does smoothing ("부드러운 조밀화") remove the LiDAR position jitter on 260910_object?

The Dataset LiDAR trajectory (KISS-ICP, 10 Hz, from the live run log) shows scan-level spikes of 1.5-2.7 cm
that ORB-SLAM3 does not see at the same instants. Variants, all centred (non-causal) so each needs a
look-ahead that has to fit inside the hub's display delay (0.3 s):

  raw          KISS-ICP poses as streamed
  box3/box5    centred moving average over 3 / 5 scans (look-ahead 0.1 / 0.2 s)
  gauss1/2     Gaussian weights, sigma 1 / 2 scans, truncated at 3 sigma (0.3 / 0.6 s)
  repair       only scans whose second difference exceeds k * robust sigma are replaced by the midpoint
               of their neighbours (look-ahead 0.1 s)
  ca_rts       constant-acceleration Kalman filter + RTS smoother on position (full batch, offline bound)

Metrics:
  accel        per-scan second difference |p[i+1] - 2 p[i] + p[i-1]| (cm)
  hf_vs_orb    high-frequency disagreement with ORB-SLAM3 final (loop-closed) trajectory: both expressed as the
               SLAM-folder camera pose, worlds aligned by Umeyama (with scale), difference minus its 1 s moving
               average (cm). Removing LiDAR noise lowers it; flattening real motion raises it, most visibly on
               fast turns.
  objects      SAM-6D estimates of the live run re-placed with the smoothed trajectory: per-object spread
               around the median (cm)
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "integration"))
sys.path.insert(0, str(ROOT / "objpose" / "pc"))
sys.path.insert(0, str(HERE))
from analyze_hdl_wobble import plane_axes, smooth_residual, yaw_rate  # noqa: E402
from fusion import interp_se3, mat  # noqa: E402
from rig_offset import read_tum  # noqa: E402


def mean_rot(Rs, w):
    q = Rotation.from_matrix(Rs).as_quat()
    q[(q @ q[len(q) // 2]) < 0] *= -1
    m = (w[:, None] * q).sum(0)
    return Rotation.from_quat(m / np.linalg.norm(m)).as_matrix()


def window_smooth(T, weights):
    h = len(weights) // 2
    out = T.copy()
    for i in range(len(T)):
        lo, hi = max(0, i - h), min(len(T), i + h + 1)
        w = np.array(weights[lo - i + h:hi - i + h], dtype=float)      # copy: normalising must not touch the kernel
        w /= w.sum()
        out[i, :3, 3] = (w[:, None] * T[lo:hi, :3, 3]).sum(0)
        out[i, :3, :3] = mean_rot(T[lo:hi, :3, :3], w)
    return out


def accel(P):
    a = np.zeros(len(P))
    a[1:-1] = np.linalg.norm(P[2:] - 2 * P[1:-1] + P[:-2], axis=1) * 100
    return a


def repair(T, k=4.0):
    a = accel(T[:, :3, 3])
    sig = 1.4826 * np.median(np.abs(a[1:-1] - np.median(a[1:-1])))
    thr = np.median(a[1:-1]) + k * sig
    out = T.copy()
    bad = np.nonzero(a > thr)[0]
    for i in bad:
        out[i] = interp_se3(T[i - 1], T[i + 1], 0.5)
    return out, int(len(bad)), float(thr)


def ca_rts(ts, T, meas_sigma=0.004, jerk_psd=2.0):
    """constant-acceleration model per axis, forward Kalman + Rauch-Tung-Striebel backward pass (positions only)"""
    P = T[:, :3, 3]
    n = len(ts)
    out = T.copy()
    H = np.array([[1.0, 0, 0]])
    R = np.array([[meas_sigma ** 2]])
    for ax in range(3):
        xs, Ps, xp, Pp, Fs = [], [], [], [], []
        x = np.array([P[0, ax], 0, 0]); C = np.diag([1e-4, 1.0, 10.0])
        for i in range(n):
            dt = ts[i] - ts[i - 1] if i else 0.1
            F = np.array([[1, dt, dt * dt / 2], [0, 1, dt], [0, 0, 1]])
            Q = jerk_psd * np.array([[dt ** 5 / 20, dt ** 4 / 8, dt ** 3 / 6], [dt ** 4 / 8, dt ** 3 / 3, dt ** 2 / 2],
                                     [dt ** 3 / 6, dt ** 2 / 2, dt]])
            if i:
                x, C = F @ x, F @ C @ F.T + Q
            xp.append(x); Pp.append(C); Fs.append(F)
            K = C @ H.T @ np.linalg.inv(H @ C @ H.T + R)
            x = x + (K @ (P[i, ax] - H @ x)).ravel()
            C = (np.eye(3) - K @ H) @ C
            xs.append(x); Ps.append(C)
        xsm = xs[-1]
        res = [xsm]
        for i in range(n - 2, -1, -1):
            G = Ps[i] @ Fs[i + 1].T @ np.linalg.inv(Pp[i + 1])
            xsm = xs[i] + G @ (xsm - xp[i + 1])
            res.append(xsm)
        out[:, ax, 3] = np.array(res[::-1])[:, 0]
    return out


def umeyama_sim3(A, B):
    ca, cb = A.mean(0), B.mean(0)
    Ac, Bc = A - ca, B - cb
    U, S, Vt = np.linalg.svd(Bc.T @ Ac / len(A))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    Rm = U @ D @ Vt
    s = np.trace(np.diag(S) @ D) / (Ac ** 2).sum(1).mean()
    return s, Rm, cb - s * Rm @ ca


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default=str(ROOT / "objpose/output/live_260910_lidar_v3"))
    ap.add_argument("--orb", default=str(ROOT / "objpose/rt/260910/slam_orb_tum.txt.optimized.txt"))
    ap.add_argument("--cam-lidar", default=str(ROOT / "objpose/rt/260910/T_cam_lidar_final.json"))
    ap.add_argument("--out", default=str(HERE / "smooth_lidar_260910.json"))
    a = ap.parse_args()
    run = Path(a.run)
    S = json.loads((run / "summary.json").read_text())
    X = np.asarray(S["extrinsic"]["T_slam_sam"]) if isinstance(S.get("extrinsic"), dict) and "T_slam_sam" in S["extrinsic"] \
        else np.asarray(json.loads((ROOT / "objpose/rt/260910/X_lidar_sam.json").read_text())["T_slam_sam"])
    poses = [json.loads(l) for l in open(run / "slam_poses.jsonl")]
    poses = [m for m in poses if m.get("T_wc") is not None and m.get("state") == "OK"]
    ts = np.array([m["t_ns"] for m in poses]) / 1e9
    T = np.array([mat(m["T_wc"]) for m in poses])
    cl = json.loads(Path(a.cam_lidar).read_text())
    T_cam_lidar = np.asarray(cl.get("T_cam_lidar", cl.get("T")))
    T_lidar_cam = np.linalg.inv(T_cam_lidar)
    to, To = read_tum(a.orb)

    rep, n_rep, thr = repair(T)
    variants = {
        "raw": (T, 0.0),
        "box3": (window_smooth(T, [1, 1, 1]), 0.1),
        "box5": (window_smooth(T, [1, 1, 1, 1, 1]), 0.2),
        "gauss1": (window_smooth(T, np.exp(-0.5 * (np.arange(-3, 4) / 1.0) ** 2)), 0.3),
        "gauss2": (window_smooth(T, np.exp(-0.5 * (np.arange(-6, 7) / 2.0) ** 2)), 0.6),
        "repair": (rep, 0.1),
        "ca_rts": (ca_rts(ts, T), None),
    }

    # reference: ORB final camera positions at the LiDAR stamps
    ok = (ts > to[0]) & (ts < to[-1])
    Po = np.array([np.interp(ts[ok], to, To[:, j, 3]) for j in range(3)]).T
    raw_acc = accel(T[:, :3, 3])
    spikes = np.nonzero(raw_acc > np.percentile(raw_acc, 99))[0]
    spike_mask = np.zeros(len(ts), bool); spike_mask[spikes] = True
    e1, e2, _ = plane_axes(T[:, :3, 3])
    yr = yaw_rate(ts, T, e1, e2)
    fit = None

    # SAM-6D estimates of the live run, re-placed per variant
    est = [json.loads(l) for l in open(run / "sam6d_estimates.jsonl")]
    est = [e for e in est if e.get("placed")]
    tau = float(json.loads((ROOT / "objpose/rt/260910/X_lidar_sam.json").read_text()).get("sam_tau_s", 0.0))

    def pose_at(V, t):
        k = int(np.clip(np.searchsorted(ts, t), 1, len(ts) - 1))
        u = float(np.clip((t - ts[k - 1]) / (ts[k] - ts[k - 1]), 0, 1))
        return interp_se3(V[k - 1], V[k], u)

    res = {"spike_threshold_raw_p99_cm": round(float(np.percentile(raw_acc, 99)), 2),
           "repair_info": {"replaced_scans": n_rep, "threshold_cm": round(thr, 2)}}
    for name, (V, look) in variants.items():
        P = V[:, :3, 3]
        acc = accel(P)
        r = smooth_residual(ts, P, 1.0)
        wob = np.hypot(r @ e1, r @ e2) * 100
        Pc = np.array([(V[i] @ T_lidar_cam)[:3, 3] for i in range(len(V))])[ok]
        if fit is None:
            fit = umeyama_sim3(Pc, Po)            # fixed from raw, shared by all variants
        s, Rm, t = fit
        d = (s * Pc @ Rm.T + t) - Po
        hf = np.linalg.norm(smooth_residual(ts[ok], d, 1.0), axis=1) * 100
        if name == "raw":
            res["alignment_rmse_cm"] = round(float(np.sqrt(np.mean(np.sum(d ** 2, 1)))) * 100, 2)
        # objects
        by = {}
        worst = 0.0
        for e in est:
            Tco = np.eye(4); Tco[:3, :3] = np.asarray(e["R"]); Tco[:3, 3] = np.asarray(e["t_mm"]) / 1000
            Tw = pose_at(V, e["t_ns"] / 1e9 + tau) @ X @ Tco
            if name == "raw":
                worst = max(worst, float(np.linalg.norm(Tw[:3, 3] - mat(e["T_w_obj"])[:3, 3])))
            by.setdefault(e["object"], []).append(Tw[:3, 3])
        spread = np.concatenate([np.linalg.norm(np.array(L) - np.median(L, 0), axis=1) for L in by.values()]) * 100
        row = {"lookahead_s": look,
               "accel_cm": {"median": round(float(np.median(acc[1:-1])), 3), "p99": round(float(np.percentile(acc[1:-1], 99)), 2),
                            "max": round(float(acc.max()), 2), "at_raw_spikes_median": round(float(np.median(acc[spikes])), 2)},
               "wobble_h_rms_cm": round(float(np.sqrt(np.mean(wob ** 2))), 3),
               "hf_vs_orb_cm": {"rms": round(float(np.sqrt(np.mean(hf ** 2))), 3),
                                "at_raw_spikes_rms": round(float(np.sqrt(np.mean(hf[spike_mask[ok]] ** 2))), 3),
                                "fast_turn_gt40dps_rms": round(float(np.sqrt(np.mean(hf[yr[ok] > 40] ** 2))), 3)},
               "shift_from_raw_cm": {"median": round(float(np.median(np.linalg.norm(P - T[:, :3, 3], axis=1))) * 100, 3),
                                     "max": round(float(np.max(np.linalg.norm(P - T[:, :3, 3], axis=1))) * 100, 2)},
               "object_spread_cm": {"median": round(float(np.median(spread)), 3), "p90": round(float(np.percentile(spread, 90)), 3)}}
        if name == "raw":
            row["object_replay_check_max_cm"] = round(worst * 100, 3)
        res[name] = row
        print(f"{name:7s} {json.dumps(row)}", flush=True)
    res["alignment_scale"] = round(float(fit[0]), 4)
    Path(a.out).write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
