#!/usr/bin/env python3
"""fit_cam_rt_260915.py — camera-camera rig RT (T_slam_sam) for the 2026-09-15 recordings.

Same method as objpose/pc/estimate_rt_offline.py (trajectory-plane gravity + planar SE(2)
hand-eye on the ORB-SLAM3 relative motions + depth floor height); the only difference is that
the 260915 sessions are CONVERTED rosbag2 folders, so the floor stage reads them through
pc/conv_session.ConvSession instead of pc/rs_session.RsSession, and both cameras are already on
one clock (no SAM_MINUS_SLAM_CLOCK_NS shift).

    python3 objpose/rt/260915/fit_cam_rt_260915.py --session 260915_eightcircle \
        --slam-traj .../eightcircle_slam_orb_tum.txt.optimized.txt \
        --sam-traj  .../eightcircle_sam_orb_tum.txt.optimized.txt --out .../fit_cam_eightcircle.json
"""
from __future__ import annotations
import argparse, json, sys
from pathlib import Path
import numpy as np

REPO = Path("/home/jucpark/DeepLearning/CLI_environment")
sys.path.insert(0, str(REPO / "objpose" / "pc"))
sys.path.insert(0, str(REPO / "objpose" / "lidar"))
sys.path.insert(0, str(REPO / "integration"))
import estimate_rt_offline as E              # noqa: E402
from estimate_rt_offline import (trajectory_down, levelling, planar, rel_pairs,  # noqa: E402
                                 solve_planar, floor_heights, rot_deg)
from conv_session import ConvSession          # noqa: E402
from rig_offset import read_tum               # noqa: E402

NAS = Path("/mnt/synology/Dataset/data_slam_converted")
CACHE = REPO / "objpose" / "rt" / "clock"


def robust_h(h):
    med = np.median(h); mad = np.median(np.abs(h - med)) * 1.4826
    keep = np.abs(h - med) < 3 * max(mad, 0.005)
    return float(np.median(h[keep])), float(mad), int(keep.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--slam-traj", required=True)
    ap.add_argument("--sam-traj", required=True)
    ap.add_argument("--windows", default="0.5,1,1.5,2,3,4")
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--tau-range", type=float, default=0.06)
    ap.add_argument("--floor-frames", type=int, default=400)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    windows = [float(v) for v in a.windows.split(",")]
    rep = {"session": a.session, "slam_traj": a.slam_traj, "sam_traj": a.sam_traj}

    ts_a, T_a = read_tum(a.slam_traj)
    ts_b, T_b = read_tum(a.sam_traj)
    d_a, pl_a = trajectory_down(T_a)
    d_b, pl_b = trajectory_down(T_b)
    L_a, L_b = levelling(d_a), levelling(d_b)
    tr_a, tr_b = planar(ts_a, T_a, L_a), planar(ts_b, T_b, L_b)
    span_a = float(np.ptp(T_a[:, :3, 3], 0).max()); path_a = float(np.linalg.norm(np.diff(T_a[:, :3, 3], axis=0), axis=1).sum())
    yaw_range = float(np.degrees(np.ptp(np.unwrap(tr_a[:, 3]))))
    rep["motion"] = {"n_slam": len(ts_a), "n_sam": len(ts_b), "path_m": round(path_a, 2),
                     "bbox_max_m": round(span_a, 2), "yaw_range_deg": round(yaw_range, 1),
                     "plane_rms_slam_cm": round(pl_a * 100, 2), "plane_rms_sam_cm": round(pl_b * 100, 2)}
    print(f"[{a.session}] down SLAM {np.round(d_a,4)} rms {pl_a*100:.2f} cm | SAM {np.round(d_b,4)} rms {pl_b*100:.2f} cm")
    print(f"[{a.session}] motion: {rep['motion']}")

    best = None
    for tau in np.round(np.arange(-a.tau_range, a.tau_range + 1e-9, 0.002), 3):
        P = rel_pairs(tr_a, tr_b, float(tau), [1.0], a.stride * 2)
        if len(P) < 50:
            continue
        e = float(np.median(np.degrees(np.abs(np.angle(np.exp(1j * (P[:, 2] - P[:, 3])))))))
        if best is None or e < best[1]:
            best = (float(tau), e)
    tau, rot_consistency = best
    print(f"[{a.session}] tau {tau:+.3f} s, rotation increments agree to {rot_consistency:.3f} deg @1 s")

    P = rel_pairs(tr_a, tr_b, tau, windows, a.stride)
    rot_err = np.degrees(np.abs(np.angle(np.exp(1j * (P[:, 2] - P[:, 3])))))
    P = P[rot_err < 2.0]
    (tx, ty, phi), res, sd = solve_planar(P)
    print(f"[{a.session}] planar hand-eye {len(P)} pairs tx {tx:+.4f} ty {ty:+.4f} phi {np.degrees(phi):+.3f} deg "
          f"| residual med {np.median(res)*100:.2f} p90 {np.percentile(res,90)*100:.2f} cm "
          f"| sd {sd[0]*1000:.1f}/{sd[1]*1000:.1f} mm {np.degrees(sd[2]):.3f} deg")
    quarters = []
    for part in np.array_split(np.argsort(P[:, 0]), 4):
        (qx, qy, qp), _, _ = solve_planar(P[part])
        quarters.append({"t_from_s": round(float(P[part, 0].min() - P[:, 0].min()), 1), "n": int(len(part)),
                         "d_tx_mm": round((qx - tx) * 1000, 1), "d_ty_mm": round((qy - ty) * 1000, 1),
                         "d_phi_deg": round(float(np.degrees(qp - phi)), 3)})
    print(f"[{a.session}] quarters:", [(q["t_from_s"], q["d_tx_mm"], q["d_ty_mm"], q["d_phi_deg"]) for q in quarters])

    slam = ConvSession(NAS / a.session / "SLAM", 0, cache_dir=str(CACHE))
    sam = ConvSession(NAS / a.session / "SAM", 0, cache_dir=str(CACHE))
    h_a, n_a = floor_heights(slam, d_a, a.floor_frames)
    h_b, n_b = floor_heights(sam, d_b, a.floor_frames)
    ha, mad_a, ka = robust_h(h_a); hb, mad_b, kb = robust_h(h_b)
    fa = n_a.mean(0); fa /= np.linalg.norm(fa)
    fb = n_b.mean(0); fb /= np.linalg.norm(fb)
    print(f"[{a.session}] floor SLAM h {ha:.4f} (MAD {mad_a*100:.2f} cm, {ka}/{len(h_a)}) "
          f"| SAM h {hb:.4f} (MAD {mad_b*100:.2f} cm, {kb}/{len(h_b)}) | dz {ha-hb:+.4f} m")

    Xl = np.eye(4)
    Xl[:3, :3] = np.array([[np.cos(phi), -np.sin(phi), 0], [np.sin(phi), np.cos(phi), 0], [0, 0, 1]])
    Xl[:3, 3] = [tx, ty, ha - hb]
    La, Lb = np.eye(4), np.eye(4)
    La[:3, :3], Lb[:3, :3] = L_a, L_b
    X = La.T @ Xl @ Lb

    rep.update({
        "T_slam_sam": X.tolist(), "tau_s": tau, "rotation_consistency_deg": round(rot_consistency, 3),
        "planar_handeye": {"pairs": int(len(P)), "tx_m": float(tx), "ty_m": float(ty),
                           "phi_deg": float(np.degrees(phi)),
                           "residual_median_cm": round(float(np.median(res)) * 100, 2),
                           "residual_p90_cm": round(float(np.percentile(res, 90)) * 100, 2),
                           "sd_mm_mm_deg": [round(float(sd[0]) * 1000, 2), round(float(sd[1]) * 1000, 2),
                                            round(float(np.degrees(sd[2])), 4)]},
        "quarters": quarters,
        "floor": {"h_slam_m": ha, "mad_slam_cm": round(mad_a * 100, 2), "n_slam": ka, "tried_slam": int(len(h_a)),
                  "h_sam_m": hb, "mad_sam_cm": round(mad_b * 100, 2), "n_sam": kb, "tried_sam": int(len(h_b)),
                  "dz_m": ha - hb,
                  "normal_vs_trajdown_slam_deg": round(float(np.degrees(np.arccos(np.clip(fa @ d_a, -1, 1)))), 2),
                  "normal_vs_trajdown_sam_deg": round(float(np.degrees(np.arccos(np.clip(fb @ d_b, -1, 1)))), 2)},
    })
    old = np.asarray(json.loads((REPO / "objpose/rt/260910/X_slam_sam_260910.json").read_text())["T_slam_sam"])
    D = np.linalg.inv(old) @ X
    rep["vs_260910"] = {"translation_diff_cm": round(float(np.linalg.norm(D[:3, 3])) * 100, 2),
                        "rotation_diff_deg": round(rot_deg(D[:3, :3]), 3)}
    print(f"[{a.session}] vs 260910: {rep['vs_260910']}")
    Path(a.out).write_text(json.dumps(rep, indent=1))
    print("->", a.out)


if __name__ == "__main__":
    main()
