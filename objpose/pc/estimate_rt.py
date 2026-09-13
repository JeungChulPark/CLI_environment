#!/usr/bin/env python3
"""estimate_rt.py — rig extrinsic T_slam_sam from two ORB-SLAM3 runs (item 6).

ORB-SLAM3 (f2000, on the Mac) was run separately on the SLAM camera's and the SAM
camera's session. Each trajectory lives in its own map, so both the rig transform X and
the map-to-map transform M are unknown:  T_mapA_slam(t) . X = M . T_mapB_sam(t).
integration/rig_offset.py solves that AX = ZB problem; this wrapper runs it on the two
trajectories (already on the SLAM host clock), writes the X the hub consumes, and
compares it with the dataset's camera_extrinsic.urdf.

    python objpose/pc/estimate_rt.py --slam objpose/rt/slam_traj_tum.txt --sam objpose/rt/sam_traj_tum.txt
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
from hub import load_extrinsic  # noqa: E402


def rot_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def main():
    ap = argparse.ArgumentParser()
    rt = REPO / "objpose" / "rt"
    ap.add_argument("--slam", default=str(rt / "slam_traj_tum.txt"))
    ap.add_argument("--sam", default=str(rt / "sam_traj_tum.txt"))
    ap.add_argument("--urdf", default="/home/jucpark/DeepLearning/Dataset/260826_etri_eightcircle_dark/camera_extrinsic.urdf")
    ap.add_argument("--tau-range", type=float, default=1.0,
                    help="clocks are already aligned; scan +-this [s] only as a check")
    ap.add_argument("--out", default=str(rt / "X_slam_sam.json"))
    a = ap.parse_args()

    fit_path = Path(a.out).with_name("rig_offset_fit.json")
    cmd = [sys.executable, str(REPO / "integration" / "rig_offset.py"), "--a", a.slam, "--b", a.sam,
           "--label", "orbslam3_f2000", "--tau-range", str(a.tau_range), "--out", str(fit_path)]
    print(" ".join(cmd), flush=True)
    subprocess.run(cmd, check=True)
    fit = json.loads(fit_path.read_text())
    X = np.asarray(fit["X_camA_camB"], np.float64)

    U, _ = load_extrinsic(Path(a.urdf))
    D = np.linalg.inv(U) @ X
    result = {
        "T_slam_sam": X.tolist(),
        "source": "ORB-SLAM3 f2000 SLAM+SAM trajectories, AX=ZB (integration/rig_offset.py)",
        "trajectories": {"slam": a.slam, "sam": a.sam},
        "n_pairs": fit["n_pairs"], "tau_s": fit["tau_s"],
        "constancy": fit["constancy"],
        "unobservable_ratio": fit["unobservable_ratio"],
        "vs_urdf": {"urdf": a.urdf,
                    "translation_diff_cm": round(float(np.linalg.norm(D[:3, 3])) * 100, 2),
                    "rotation_diff_deg": round(rot_deg(D[:3, :3]), 2),
                    "urdf_T_slam_sam": U.round(6).tolist()},
    }
    Path(a.out).write_text(json.dumps(result, indent=1))
    c = fit["constancy"]
    print(f"X_slam_sam t = {np.round(X[:3, 3], 4).tolist()} m, tau {fit['tau_s']:+.3f} s, n={fit['n_pairs']}")
    print(f"constancy p90: {c['trans_p90_cm']} cm / {c['rot_p90_deg']} deg")
    print(f"vs urdf: {result['vs_urdf']['translation_diff_cm']} cm, {result['vs_urdf']['rotation_diff_deg']} deg")
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
