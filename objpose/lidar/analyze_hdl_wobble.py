#!/usr/bin/env python3
"""analyze_hdl_wobble.py — where does the jitter in hdl_graph_slam's position estimate come from?

Per session (260724_chungbuk hdl_graph_slam output, imported runs):
  wobble          position minus a 1 s centred moving average, split into horizontal / vertical
                  (vertical = normal of the trajectory plane); ORB-SLAM3 of the same session as reference
  keyframe steps  second-difference ("jerk") of the dense trajectory at keyframe stamps vs elsewhere —
                  the dense track is scan-matching odometry with the per-keyframe correction pasted on,
                  so a change of correction between keyframes appears as a step
  correction jump translation of corr_{k+1} . corr_k^-1, corr_k = T_map_kf . T_odom_kf^-1
  floor           sensor-to-floor distance d of the floor plane attached to every keyframe
  yaw-rate        wobble magnitude vs yaw rate (no per-point deskew: merged half sweeps)
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "integration"))
from rig_offset import read_tum  # noqa: E402


def smooth_residual(ts, P, win_s=1.0):
    out = np.zeros_like(P)
    for i, t in enumerate(ts):
        m = np.abs(ts - t) <= win_s / 2
        out[i] = P[i] - P[m].mean(0)
    return out


def plane_axes(P):
    c = P - P.mean(0)
    _, _, Vt = np.linalg.svd(c, full_matrices=False)
    return Vt[0], Vt[1], Vt[2]


def wobble(ts, T, win_s=1.0):
    P = T[:, :3, 3]
    e1, e2, n = plane_axes(P)
    r = smooth_residual(ts, P, win_s)
    h = np.hypot(r @ e1, r @ e2) * 100
    v = np.abs(r @ n) * 100
    return h, v, r @ n


def yaw_rate(ts, T, e1, e2):
    f = T[:, :3, 0]                                  # sensor x axis in the world
    yaw = np.unwrap(np.arctan2(f @ e2, f @ e1))
    return np.degrees(np.abs(np.gradient(yaw, ts)))


def floor_d(graph_dump: Path):
    rows = []
    for d in sorted(graph_dump.iterdir()):
        f = d / "data"
        if not f.exists():
            continue
        stamp, dist = None, None
        for line in f.read_text().splitlines():
            if line.startswith("stamp"):
                s = line.split()
                stamp = int(s[1]) + int(s[2]) / 1e9
            if line.startswith("floor_coeffs"):
                dist = float(line.split()[4])
        if stamp is not None and dist is not None:
            rows.append((stamp, dist))
    return np.array(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/mnt/d/CLI_environment/output_slam/260724_chungbuk")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent / "hdl_wobble_260724.json"))
    a = ap.parse_args()
    root = Path(a.root)
    res = {}
    for sd in sorted((root / "hdl_graph_slam").iterdir()):
        if not (sd / "trajectory_dense.txt").exists():
            continue
        name = sd.name
        ts, T = read_tum(sd / "trajectory_dense.txt")
        tk, Tk = read_tum(sd / "trajectory.txt")
        to, To = read_tum(sd / "odometry.txt")
        h, v, vs = wobble(ts, T)
        P = T[:, :3, 3]
        e1, e2, n = plane_axes(P)

        # second difference at keyframe stamps vs elsewhere
        jerk = np.zeros(len(P))
        jerk[1:-1] = np.linalg.norm(P[2:] - 2 * P[1:-1] + P[:-2], axis=1) * 100
        kidx = np.unique(np.clip(np.searchsorted(ts, tk), 1, len(ts) - 2))
        near = np.zeros(len(P), bool)
        for k in kidx:
            near[max(k - 1, 1):k + 2] = True
        # correction jumps between consecutive keyframes
        corr = [Tk[i] @ np.linalg.inv(To[i]) for i in range(min(len(Tk), len(To)))]
        cj = np.array([np.linalg.norm((corr[i + 1] @ np.linalg.inv(corr[i]))[:3, 3]) * 100 for i in range(len(corr) - 1)])
        cr = np.array([np.degrees(np.linalg.norm(Rotation.from_matrix((corr[i + 1] @ np.linalg.inv(corr[i]))[:3, :3]).as_rotvec()))
                       for i in range(len(corr) - 1)])
        # dominant period of the vertical residual
        dt = np.median(np.diff(ts))
        grid = np.arange(ts[0], ts[-1], dt)
        vg = np.interp(grid, ts, vs)
        spec = np.abs(np.fft.rfft(vg - vg.mean()))
        freq = np.fft.rfftfreq(len(vg), dt)
        band = (freq > 0.1) & (freq < 3.0)
        f_peak = float(freq[band][np.argmax(spec[band])])
        kf_rate = 1.0 / float(np.median(np.diff(tk)))
        # yaw-rate dependence
        yr = yaw_rate(ts, T, e1, e2)
        lo, hi = yr < 5, yr > 20
        fl = floor_d(sd / "graph_dump") if (sd / "graph_dump").exists() else np.zeros((0, 2))
        row = {
            "dense_hz": round(1 / dt, 2), "keyframes": len(tk), "keyframe_interval_s": round(float(np.median(np.diff(tk))), 2),
            "wobble_h_cm": {"rms": round(float(np.sqrt(np.mean(h ** 2))), 2), "p95": round(float(np.percentile(h, 95)), 2)},
            "wobble_v_cm": {"rms": round(float(np.sqrt(np.mean(v ** 2))), 2), "p95": round(float(np.percentile(v, 95)), 2)},
            "jerk_cm": {"at_keyframes_median": round(float(np.median(jerk[near])), 3),
                        "elsewhere_median": round(float(np.median(jerk[~near & (jerk > 0)])), 3),
                        "ratio": round(float(np.median(jerk[near]) / max(np.median(jerk[~near & (jerk > 0)]), 1e-9)), 1)},
            "correction_jump": {"trans_cm_median": round(float(np.median(cj)), 2), "trans_cm_p90": round(float(np.percentile(cj, 90)), 2),
                                "rot_deg_median": round(float(np.median(cr)), 3), "rot_deg_p90": round(float(np.percentile(cr, 90)), 3)},
            "vertical_peak_hz": round(f_peak, 3), "keyframe_rate_hz": round(kf_rate, 3),
            "wobble_h_by_yaw_rate_cm": {"slow<5deg/s": round(float(np.median(h[lo])), 2) if lo.any() else None,
                                        "fast>20deg/s": round(float(np.median(h[hi])), 2) if hi.any() else None},
            "floor_distance_m": ({"n": int(len(fl)), "median": round(float(np.median(fl[:, 1])), 3),
                                  "p5": round(float(np.percentile(fl[:, 1], 5)), 3), "p95": round(float(np.percentile(fl[:, 1], 95)), 3),
                                  "consecutive_change_median_cm": round(float(np.median(np.abs(np.diff(fl[:, 1])))) * 100, 1)}
                                 if len(fl) > 2 else None),
        }
        orb = root / "orb3_slam" / name / "CameraTrajectory.txt"
        if orb.exists():
            tso, Tso = read_tum(orb)
            ho, vo, _ = wobble(tso, Tso)
            row["orb_wobble_h_cm_rms"] = round(float(np.sqrt(np.mean(ho ** 2))), 2)
            row["orb_wobble_v_cm_rms"] = round(float(np.sqrt(np.mean(vo ** 2))), 2)
        res[name] = row
        print(name, json.dumps(row, ensure_ascii=False))
    Path(a.out).write_text(json.dumps(res, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
