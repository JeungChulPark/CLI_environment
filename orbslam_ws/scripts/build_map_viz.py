#!/usr/bin/env python3
"""
Pack ORB-SLAM3 map + trajectory output into one JSON for the 3D viewer.

Inputs per run (produced by rgbd_node with save_map_points_on_shutdown:=true):
  map_points.pcd         sparse map points, PCD ascii, 11-line header
  CameraTrajectory.txt   per-frame pose,  TUM: t tx ty tz qx qy qz qw
  KeyFrameTrajectory.txt keyframe pose,   same format

Coordinate note
---------------
These are ORB-SLAM3 world coordinates, which inherit the first camera's optical
frame: +x right, +y DOWN, +z forward. The viewer therefore treats (x, z) as the
ground plane and -y as up; this file emits the raw values and records the
convention so the transform lives in one place.

The two runs are NOT expressed in a common frame. Each starts its own map at
identity from its own first frame, so a direct pose-to-pose difference between
runs is meaningless. What is comparable without alignment: path length, the
start-to-end closure gap, extent, and keyframe count. Those are computed here;
the viewer shows the two trajectories in their own frames, side by side.
"""

import json
import sys
from pathlib import Path

import numpy as np

HDR = 11  # PCD ascii header lines written by rgbd_node


def load_run(d: Path, stride: int):
    pts = np.loadtxt(d / "map_points.pcd", skiprows=HDR)
    tr = np.loadtxt(d / "CameraTrajectory.txt")
    kf = np.loadtxt(d / "KeyFrameTrajectory.txt")

    pos = tr[:, 1:4]
    seg = np.linalg.norm(np.diff(pos, axis=0), axis=1)

    # Drop map points far outside the working volume: a handful of badly
    # triangulated points otherwise set the view scale for everything else.
    c = np.median(pts, axis=0)
    r = np.linalg.norm(pts - c, axis=1)
    keep = r <= np.percentile(r, 99.5)
    pts_k = pts[keep]

    sub = tr[::stride]
    return {
        "points": [[round(float(v), 3) for v in p] for p in pts_k],
        "points_dropped": int((~keep).sum()),
        "traj": [[round(float(v), 4) for v in p] for p in sub[:, 1:4]],
        "quat": [[round(float(v), 4) for v in q] for q in sub[:, 4:8]],
        "t": [round(float(x - tr[0, 0]), 3) for x in sub[:, 0]],
        "kf": [[round(float(v), 4) for v in p] for p in kf[:, 1:4]],
        "stride": stride,
        "metrics": {
            "frames": int(len(tr)),
            "keyframes": int(len(kf)),
            "map_points": int(len(pts_k)),
            "path_len_m": round(float(seg.sum()), 3),
            "closure_gap_m": round(float(np.linalg.norm(pos[-1] - pos[0])), 4),
            "duration_s": round(float(tr[-1, 0] - tr[0, 0]), 2),
            "extent_m": [round(float(pos[:, i].ptp()), 3) for i in range(3)],
            "max_step_mm": round(float(seg.max() * 1000), 1),
            "median_step_mm": round(float(np.median(seg) * 1000), 2),
        },
    }


def main():
    base = Path("/home/jucpark/DeepLearning/CLI_environment/orbslam_ws/output")
    out = {
        "convention": {"up": "-y", "ground": "xz",
                       "note": "ORB-SLAM3 world frame = first camera optical frame"},
        "runs": {},
    }
    for key, sub, label, rate in [("orb10", "map_1.0x", "1.0x replay", "30 Hz"),
                                  ("orb05", "map_0.5x", "0.5x replay", "15 Hz")]:
        d = base / sub
        if not (d / "map_points.pcd").exists():
            print(f"missing: {d}/map_points.pcd", file=sys.stderr)
            return 1
        r = load_run(d, stride=3)
        r["label"] = label
        r["rate"] = rate
        out["runs"][key] = r
        m = r["metrics"]
        print(f"{label:<12} pts {m['map_points']:>6}  frames {m['frames']}  kf {m['keyframes']}  "
              f"path {m['path_len_m']:.2f} m  closure {m['closure_gap_m']*100:.1f} cm")

    dst = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/map_viz.json")
    dst.write_text(json.dumps(out, separators=(",", ":")))
    print(f"\nwrote {dst}  ({dst.stat().st_size/1e6:.2f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
