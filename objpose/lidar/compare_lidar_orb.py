#!/usr/bin/env python3
"""compare_lidar_orb.py — LiDAR SLAM (KISS-ICP) vs visual SLAM (ORB-SLAM3) on the same recording.

The LiDAR trajectory is carried into the camera frame with the calibrated extrinsic
(T_w_cam = T_w_lidar . T_cam_lidar^-1), both are evaluated at the LiDAR scan times (ORB-SLAM3 is
interpolated from 30 Hz), rigidly aligned (no scale), and compared: absolute error, relative
errors over 1 s / 10 s windows, path length and return-to-start distance. There is no ground
truth, so every number is a disagreement between the two systems.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "objpose" / "pc"))
sys.path.insert(0, str(REPO / "integration"))
from fusion import PoseBuffer  # noqa: E402
from rig_offset import read_tum  # noqa: E402


def rotdeg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def umeyama(A, B):
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((B - cb).T @ (A - ca))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    return R, cb - R @ ca


def path_stats(P):
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    return {"path_m": round(float(seg.sum()), 2), "start_to_end_m": round(float(np.linalg.norm(P[-1] - P[0])), 3)}


def rpe(T_a, T_b, stamps, win_s):
    out_r, out_t = [], []
    for i in range(len(stamps)):
        j = int(np.searchsorted(stamps, stamps[i] + win_s))
        if j >= len(stamps) or stamps[j] - stamps[i] > win_s + 0.15:
            continue
        A = np.linalg.inv(T_a[i]) @ T_a[j]
        B = np.linalg.inv(T_b[i]) @ T_b[j]
        E = np.linalg.inv(A) @ B
        out_r.append(rotdeg(E[:3, :3]))
        out_t.append(np.linalg.norm(A[:3, 3] - B[:3, 3]) * 100)
    r, t = np.array(out_r), np.array(out_t)
    return {"trans_cm_median": round(float(np.median(t)), 2), "trans_cm_p95": round(float(np.percentile(t, 95)), 2),
            "rot_deg_median": round(float(np.median(r)), 3), "rot_deg_p95": round(float(np.percentile(r, 95)), 3)}


def main():
    rt = REPO / "objpose" / "rt" / "260910"
    ap = argparse.ArgumentParser()
    ap.add_argument("--lidar-traj", default=str(REPO / "objpose" / "lidar" / "feasibility_mac" / "lidar_traj_tum.txt"))
    ap.add_argument("--orb", default=str(rt / "slam_orb_tum.txt"))
    ap.add_argument("--extrinsic", default=str(rt / "T_cam_lidar_final.json"))
    ap.add_argument("--out", default=str(REPO / "objpose" / "lidar" / "compare_260910"))
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)

    X = np.asarray(json.loads(Path(a.extrinsic).read_text())["T_cam_lidar"], np.float64)
    ts_l, T_l = read_tum(a.lidar_traj)
    T_lc = T_l @ np.linalg.inv(X)                       # LiDAR trajectory as camera poses
    res = {"extrinsic": a.extrinsic, "lidar": path_stats(T_l[:, :3, 3])}
    curves = {}
    for tag, path in (("orb_final", a.orb + ".optimized.txt"), ("orb_live", a.orb)):
        ts_o, T_o = read_tum(path)
        buf = PoseBuffer(0.1)
        for t, M in zip(ts_o, T_o):
            buf.add(int(round(t * 1e9)), "OK", M)
        keep, A, B = [], [], []
        for t, M in zip(ts_l, T_lc):
            Mo, _ = buf.at(int(round(t * 1e9)))
            if Mo is not None:
                keep.append(t); A.append(M); B.append(Mo)
        A, B, keep = np.array(A), np.array(B), np.array(keep)
        R, tr = umeyama(B[:, :3, 3], A[:, :3, 3])        # ORB -> LiDAR-derived camera frame
        G = np.eye(4); G[:3, :3] = R; G[:3, 3] = tr
        Bw = np.einsum("ij,njk->nik", G, B)
        err = np.linalg.norm(Bw[:, :3, 3] - A[:, :3, 3], axis=1) * 100
        rot = np.array([rotdeg(A[i, :3, :3].T @ Bw[i, :3, :3]) for i in range(len(A))])
        res[tag] = {"samples": int(len(A)), **path_stats(T_o[:, :3, 3]),
                    "ate_cm": {"rmse": round(float(np.sqrt(np.mean(err ** 2))), 2), "median": round(float(np.median(err)), 2),
                               "p95": round(float(np.percentile(err, 95)), 2), "max": round(float(err.max()), 2)},
                    "orientation_diff_deg": {"median": round(float(np.median(rot)), 2), "p95": round(float(np.percentile(rot, 95)), 2)},
                    "rpe_1s": rpe(A, Bw, keep, 1.0), "rpe_10s": rpe(A, Bw, keep, 10.0)}
        curves[tag] = (keep, err, A[:, :3, 3], Bw[:, :3, 3])

    # top view: LiDAR-derived camera path vs ORB-SLAM3 final path
    _, _, PA, PB = curves["orb_final"]
    _, _, _, PL = curves["orb_live"]
    c = PA - PA.mean(0)
    _, _, Vt = np.linalg.svd(c, full_matrices=False)
    e1, e2 = Vt[0], Vt[1]
    allp = np.vstack([PA, PB, PL]) - PA.mean(0)
    u, v = allp @ e1, allp @ e2
    s = 120.0
    lo = (u.min() - 0.3, v.min() - 0.3); W = int((u.max() - lo[0] + 0.3) * s); H = int((v.max() - lo[1] + 0.3) * s)
    img = np.full((H, W, 3), 20, np.uint8)
    def draw(P, col, th):
        q = P - PA.mean(0)
        pts = np.column_stack([(q @ e1 - lo[0]) * s, H - 1 - (q @ e2 - lo[1]) * s]).astype(np.int32)
        cv2.polylines(img, [pts], False, col, th, cv2.LINE_AA)
    draw(PL, (90, 90, 200), 1)
    draw(PB, (60, 190, 255), 2)
    draw(PA, (120, 230, 120), 2)
    for i, (txt, col) in enumerate((("LiDAR KISS-ICP (camera frame)", (120, 230, 120)), ("ORB-SLAM3 final (loop closed)", (60, 190, 255)),
                                    ("ORB-SLAM3 live per-frame", (90, 90, 200)))):
        cv2.putText(img, txt, (10, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
    cv2.imwrite(str(out / "trajectories_topview.png"), img)
    (out / "compare.json").write_text(json.dumps(res, indent=1))
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
