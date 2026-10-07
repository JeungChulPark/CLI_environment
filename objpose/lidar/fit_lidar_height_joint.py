#!/usr/bin/env python3
"""fit_lidar_height_joint.py — one LiDAR height from both cameras' overlap scans.

Planar motion cannot observe the vertical LiDAR-to-camera arm, so estimate_lidar_cam_rt.py
grid-searches it by maximising the number of LiDAR points landing on the RealSense depth
surface. Run per camera that argmax is weak: on 260901 the SLAM camera's peak varies only 6 %
over 40 cm, and the two cameras' independent answers put the LiDAR 1.59 m and 1.14 m above
the floor — 45 cm apart, which the rig RT (fitted from the two ORB-SLAM3 trajectories, and
agreeing with the LiDAR-routed rotation to 0.05 deg) does not allow.

Measuring the LiDAR's height against the floor directly does not rescue it either: on this
recording the VLP-16 gets no floor return at all. The lowest ring's elevation stays at
-15.4 deg out to 10 m, i.e. it reaches the walls without ever striking the floor — the
grazing-incidence return off the polished floor is too weak.

But there is only ONE unknown. Both cameras sit on the same rigid cart and their heights
above the floor are already measured well from their own aligned depth (RANSAC floor plane,
MAD 0.5-2.3 cm, in the rig-RT fit). Writing each camera's overlap scan as a function of the
LiDAR's height above the floor,

    L = F_camera - height_along_down

puts both curves on one axis, where they are two independent votes on the same quantity.
Their sum has a far sharper maximum than either alone, and the resulting pair of extrinsics
is consistent with the rig RT by construction.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def load_scan(path: Path) -> tuple[np.ndarray, np.ndarray, dict]:
    d = json.loads(path.read_text())
    s = np.asarray(d["overlap_scan"], np.float64)
    return s[:, 0], s[:, 1], d


def parabola_peak(x: np.ndarray, y: np.ndarray, i: int) -> float:
    """Sub-step refinement of the maximum at index i."""
    if i == 0 or i == len(x) - 1:
        return float(x[i])
    y0, y1, y2 = y[i - 1], y[i], y[i + 1]
    den = y0 - 2 * y1 + y2
    if abs(den) < 1e-9:
        return float(x[i])
    return float(x[i] - 0.5 * (x[i + 1] - x[i]) * (y2 - y0) / den)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--slam-rt", required=True, help="T_cam_lidar_final.json for the SLAM camera")
    ap.add_argument("--sam-rt", required=True, help="T_samcam_lidar_final.json for the SAM camera")
    ap.add_argument("--rig-rt", required=True, help="X_slam_sam_*.json, for the two floor heights")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    floor = json.loads(Path(a.rig_rt).read_text())["floor"]
    F = {"SLAM": float(floor["h_slam_m"]), "SAM": float(floor["h_sam_m"])}
    print(f"camera heights above floor: SLAM {F['SLAM']:.4f} m, SAM {F['SAM']:.4f} m "
          f"(MAD {floor['mad_cm'][0]:.2f} / {floor['mad_cm'][1]:.2f} cm)")

    curves, docs = {}, {}
    for cam, path in (("SLAM", Path(a.slam_rt)), ("SAM", Path(a.sam_rt))):
        h, n, d = load_scan(path)
        L = F[cam] - h                                   # LiDAR height above the floor
        order = np.argsort(L)
        curves[cam] = (L[order], n[order])
        docs[cam] = d
        print(f"  {cam:<4} alone: peak L = {L[order][n[order].argmax()]:.3f} m "
              f"(contrast {n.max() / n.min():.2f}x)")

    grid = np.arange(max(c[0][0] for c in curves.values()),
                     min(c[0][-1] for c in curves.values()) + 1e-9, 0.01)
    # normalise each curve to its own maximum so the denser camera does not dominate the sum
    votes = {c: np.interp(grid, L, n / n.max()) for c, (L, n) in curves.items()}
    total = sum(votes.values())
    i = int(total.argmax())
    L_star = parabola_peak(grid, total, i)
    print(f"joint peak: LiDAR {L_star:.4f} m above the floor "
          f"(SLAM vote {votes['SLAM'][i]:.3f}, SAM vote {votes['SAM'][i]:.3f})")

    out = {"lidar_height_above_floor_m": L_star,
           "method": "sum of both cameras' LiDAR/depth overlap scans, re-expressed as the "
                     "LiDAR height above the floor using each camera's measured floor height",
           "camera_floor_heights_m": F,
           "height_along_down_m": {cam: F[cam] - L_star for cam in F},
           "previous_height_along_down_m": {cam: docs[cam]["height_along_down_m"] for cam in F},
           "joint_curve": [[round(float(g), 3), round(float(t), 4)] for g, t in zip(grid, total)],
           "inputs": {"slam_rt": a.slam_rt, "sam_rt": a.sam_rt, "rig_rt": a.rig_rt}}
    Path(a.out).write_text(json.dumps(out, indent=1))
    for cam in F:
        print(f"  {cam:<4} height_along_down: {docs[cam]['height_along_down_m']:+.4f} -> "
              f"{F[cam] - L_star:+.4f} m")
    print("wrote", a.out)


if __name__ == "__main__":
    main()
