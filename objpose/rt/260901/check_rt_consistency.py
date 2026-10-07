#!/usr/bin/env python3
"""check_rt_consistency.py — cross-check the three 260901 extrinsics against each other.

The rig RT (X_slam_sam, from the two ORB-SLAM3 trajectories) and the two LiDAR RTs
(T_cam_lidar, one per camera, from each camera's trajectory against KISS-ICP) are fitted
independently. On a rigid cart they must close the loop:

    T_slam_sam  ==  T_slamcam_lidar . inv(T_samcam_lidar)

so the disagreement is an end-to-end accuracy figure for all three fits at once. The gyro
rig file from the same day carries its own T_lidar_cam, which is a fourth, independent
estimate of the SLAM-camera arm.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def load(path: Path, key: str) -> np.ndarray:
    return np.asarray(json.loads(path.read_text())[key], np.float64).reshape(4, 4)


def rot_deg(R: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def report(name: str, A: np.ndarray, B: np.ndarray) -> dict:
    D = np.linalg.inv(A) @ B
    d_t, d_r = float(np.linalg.norm(D[:3, 3])) * 100, rot_deg(D[:3, :3])
    print(f"{name:<38} dt {d_t:6.2f} cm   dR {d_r:5.2f} deg")
    return {"translation_diff_cm": round(d_t, 2), "rotation_diff_deg": round(d_r, 2)}


def main() -> None:
    X = load(HERE / "X_slam_sam_260901.json", "T_slam_sam")
    T_sl = load(HERE / "T_cam_lidar_final.json", "T_cam_lidar")        # lidar in SLAM-camera frame
    T_sa = load(HERE / "T_samcam_lidar_final.json", "T_cam_lidar")     # lidar in SAM-camera frame
    via_lidar = T_sl @ np.linalg.inv(T_sa)

    print(f"X_slam_sam   t = {np.round(X[:3, 3], 4).tolist()} m")
    print(f"via LiDAR    t = {np.round(via_lidar[:3, 3], 4).tolist()} m")
    out = {"rig_RT_vs_via_lidar": report("rig RT vs LiDAR-routed rig RT", X, via_lidar)}

    gyro = HERE.parents[1] / "mac_slam" / "settings" / "rig_260901_gyro.json"
    if gyro.exists():
        # the gyro rig stores the camera in the LiDAR frame; invert to compare arms
        g = json.loads(gyro.read_text())
        G = np.linalg.inv(np.asarray(g["T_lidar_cam"], np.float64))
        D = np.linalg.inv(T_sl) @ G
        # that fit regularises the lever arm along the (vertical) rotation axis to zero, so only
        # its horizontal part is a real measurement — split the disagreement accordingly
        down = np.asarray(g["T_lidar_cam"], np.float64)[:3, :3] @ np.array([0.0, 1.0, 0.0])
        down /= np.linalg.norm(down)
        vert = float(D[:3, 3] @ down)
        horiz = float(np.linalg.norm(D[:3, 3] - vert * down))
        print(f"{'SLAM-cam arm vs gyro-rig T_lidar_cam':<38} dt {np.linalg.norm(D[:3, 3])*100:6.2f} cm"
              f"   dR {rot_deg(D[:3, :3]):5.2f} deg   (horizontal {horiz*100:.2f} cm, "
              f"vertical {vert*100:+.2f} cm - regularised to 0 in that fit)")
        out["slam_arm_vs_gyro_rig"] = {
            "translation_diff_cm": round(float(np.linalg.norm(D[:3, 3])) * 100, 2),
            "rotation_diff_deg": round(rot_deg(D[:3, :3]), 2),
            "horizontal_diff_cm": round(horiz * 100, 2),
            "vertical_diff_cm": round(vert * 100, 2),
            "note": "the gyro rig regularises the vertical lever arm to zero (unobservable in "
                    "planar motion), so only the horizontal figure is a comparison of measurements"}

    (HERE / "rt_consistency.json").write_text(json.dumps(out, indent=1))
    print("wrote", HERE / "rt_consistency.json")


if __name__ == "__main__":
    main()
