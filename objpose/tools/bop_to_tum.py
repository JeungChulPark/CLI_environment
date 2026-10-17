#!/usr/bin/env python3
"""bop_to_tum.py — BOP scene_camera.json -> TUM trajectory on the BopSession clock.

A BOP scene may ship the ground-truth camera pose of every frame as world->camera
(`cam_R_w2c`, `cam_t_w2c` in mm; YCB-V does). The hub's pose protocol wants camera->world in
metres, stamped on the same synthetic clock `bop_session.BopSession` gives the images, so the
stream and the replay line up frame for frame.

Datasets converted from LineMOD (LM, LM-O) have no such field: their ground truth is annotated
per image and the scene is not even rigid across frames (on LM-O the measured distance between
two objects drifts by ~80 mm std), so no global world frame exists. For those the trajectory is
a constant identity pose — world = camera — which is what the per-frame BOP protocol assumes.
The hub must then be run with --no-memory, since a world-frame object memory is meaningless.

    python3 objpose/tools/bop_to_tum.py --scene <YCB_Video>/test/000048 --out gt_000048_tum.txt

The result is fed to the hub exactly like a real SLAM stream:

    python3 objpose/tools/fake_slam_stream.py --connect <pc>:17001 --tum gt_000048_tum.txt
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

DEFAULT_FPS = 30.0
DEFAULT_T0_NS = 1_600_000_000_000_000_000      # must match bop_session.DEFAULT_T0_NS


def R_to_quat(R):
    """rotation matrix -> (qx, qy, qz, qw), branch-stable"""
    m = np.asarray(R, np.float64)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        qw, qx, qy, qz = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        qw, qx, qy, qz = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        qw, qx, qy, qz = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        qw, qx, qy, qz = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([qx, qy, qz, qw])
    return q / np.linalg.norm(q)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True, help="BOP scene folder holding scene_camera.json")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=float, default=DEFAULT_FPS)
    ap.add_argument("--t0-ns", type=int, default=DEFAULT_T0_NS)
    a = ap.parse_args()

    cam = json.loads((Path(a.scene) / "scene_camera.json").read_text())
    ids = sorted(cam, key=int)
    step_ns = int(round(1e9 / a.fps))
    has_pose = "cam_R_w2c" in cam[ids[0]]
    if not has_pose:
        print(f"[bop_to_tum] {Path(a.scene).name}: scene_camera.json has no cam_R_w2c — "
              f"emitting a fixed identity pose (world = camera). Run the hub with --no-memory.")
    lines = []
    for i, fid in enumerate(ids):
        if has_pose:
            c = cam[fid]
            R_w2c = np.asarray(c["cam_R_w2c"], np.float64).reshape(3, 3)
            t_w2c = np.asarray(c["cam_t_w2c"], np.float64) / 1000.0        # mm -> m
            R_wc = R_w2c.T
            t_wc = -R_wc @ t_w2c
        else:
            R_wc, t_wc = np.eye(3), np.zeros(3)
        qx, qy, qz, qw = R_to_quat(R_wc)
        t_s = (a.t0_ns + i * step_ns) / 1e9
        lines.append(f"{t_s:.9f} {t_wc[0]:.6f} {t_wc[1]:.6f} {t_wc[2]:.6f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}")
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text("\n".join(lines) + "\n")
    span = np.linalg.norm(np.ptp(np.asarray([[float(v) for v in l.split()[1:4]] for l in lines]), axis=0))
    print(f"[bop_to_tum] {len(lines)} poses, {len(lines)/a.fps:.1f} s, bbox diag {span:.2f} m -> {a.out}")


if __name__ == "__main__":
    main()
