#!/usr/bin/env python3
"""make_X_lidar_sam.py — the hub extrinsic for the LiDAR SLAM backends.

With --slam lidar/hdl the hub's world is the LiDAR's, so the "SLAM sensor" it needs to place
the SAM camera against is the Velodyne: T_slam_sam = T_lidar_samcam = inv(T_samcam_lidar).

The clock key flips sign on the way in. estimate_lidar_cam_rt.py reports tau as the camera
stamp minus the LiDAR stamp of one event; the hub adds sam_tau_s to the SAM stamps to move
them onto the SLAM host's clock, so sam_tau_s = -tau_s.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def main() -> None:
    src = json.loads((HERE / "T_samcam_lidar_final.json").read_text())
    T = np.asarray(src["T_cam_lidar"], np.float64).reshape(4, 4)
    out = {
        "T_slam_sam": np.linalg.inv(T).tolist(),
        "source": "inv(T_samcam_lidar) from " + src["method"],
        "slam_sensor": "lidar",
        "clock": "header stamps (converted bags)",
        "sam_tau_s": -float(src["tau_s"]),
        "height_uncertainty_m": src["height_uncertainty_m"],
        "inputs": {"T_samcam_lidar": "T_samcam_lidar_final.json"},
    }
    path = HERE / "X_lidar_sam.json"
    path.write_text(json.dumps(out, indent=1))
    print(f"T_lidar_sam t = {np.round(np.linalg.inv(T)[:3, 3], 4).tolist()} m, "
          f"sam_tau_s = {out['sam_tau_s']:+.3f} s")
    print("wrote", path)


if __name__ == "__main__":
    main()
