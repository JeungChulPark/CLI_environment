#!/usr/bin/env python3
"""estimate_rt_offline.py — best-effort offline rig extrinsic T_slam_sam.

Why not the earlier two methods
  * AX = ZB over two independent maps: the cart only yaws, so the problem is ill-posed.
  * Localizing the SAM camera in the SLAM camera's saved map: works, but inherits the
    map's regional deformation (10 s windows disagree by 5-45 cm with no consensus).

This method never compares positions across a whole map. It splits X into the parts each
sensor observes well:

  1. "down" in each camera = normal of that camera's trajectory plane (the cart moves on a
     flat floor; out-of-plane RMS is 0.6 cm SLAM / 0.2 cm SAM). R_X must map d_sam -> d_slam.
  2. Planar hand-eye on RELATIVE motions over short windows (0.5-4 s) of each camera's own
     ORB-SLAM3 trajectory, in gravity-levelled coordinates: SE(2) AX = XB solves the
     horizontal offset and the yaw between the cameras. Short windows are locally accurate,
     and the figure-eight path gives large, varied yaw changes, so this is well conditioned.
  3. Height difference, which no planar motion can observe, is measured directly: the floor
     plane in each camera's aligned depth (RANSAC with the normal constrained to "down").

    X = L_slam^T . [Rz(phi) | (tx, ty, h_slam - h_sam)] . L_sam
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "integration"))
from estimate_rt_loc import object_spread, rot_deg  # noqa: E402
from hub import SAM_MINUS_SLAM_CLOCK_NS, load_extrinsic  # noqa: E402
from rig_offset import read_tum  # noqa: E402
from rs_session import RsSession  # noqa: E402

DATASET = Path("/home/jucpark/DeepLearning/Dataset/260826_etri_eightcircle_dark")


# ── 1. down direction ────────────────────────────────────────────────────────
def trajectory_down(T):
    P = T[:, :3, 3]
    c = P - P.mean(0)
    _, sv, Vt = np.linalg.svd(c, full_matrices=False)
    n = Vt[2]
    if n[1] < 0:              # optical frame: +y points down; cameras here are roughly upright
        n = -n
    return n, float((c @ n).std())


def levelling(d):
    """rows e1, e2, e3=d: camera coords -> levelled coords (z = down)."""
    e3 = d / np.linalg.norm(d)
    ref = np.array([0.0, 0.0, 1.0]) if abs(e3[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = ref - (ref @ e3) * e3
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(e3, e1)
    return np.vstack([e1, e2, e3])


def planar(ts, T, L):
    """levelled SE(2) track: (t, x, y, theta, tilt residual deg)."""
    out = []
    for t, M in zip(ts, T):
        R = L @ M[:3, :3] @ L.T
        p = L @ M[:3, 3]
        th = np.arctan2(R[1, 0], R[0, 0])
        tilt = rot_deg(R @ np.array([[np.cos(th), np.sin(th), 0], [-np.sin(th), np.cos(th), 0], [0, 0, 1]]))
        out.append((t, p[0], p[1], th, tilt))
    return np.array(out)


MAX_GAP_S = 0.1   # largest pose gap interpolated across (a 10 Hz LiDAR needs a little more)


def interp_track(tr, t):
    i = np.searchsorted(tr[:, 0], t)
    ok = (i > 0) & (i < len(tr))
    i = np.clip(i, 1, len(tr) - 1)
    t0, t1 = tr[i - 1, 0], tr[i, 0]
    ok &= (t1 - t0) < MAX_GAP_S
    u = (t - t0) / np.maximum(t1 - t0, 1e-9)
    x = tr[i - 1, 1] + u * (tr[i, 1] - tr[i - 1, 1])
    y = tr[i - 1, 2] + u * (tr[i, 2] - tr[i - 1, 2])
    dth = np.angle(np.exp(1j * (tr[i, 3] - tr[i - 1, 3])))
    th = tr[i - 1, 3] + u * dth
    return ok, x, y, th


# ── 2. planar hand-eye ───────────────────────────────────────────────────────
def rel_pairs(slam_tr, sam_tr, tau, windows, stride):
    t = sam_tr[::stride, 0]
    rows = []
    for w in windows:
        ta, tb = t, t + w
        okb1 = np.searchsorted(sam_tr[:, 0], tb) < len(sam_tr)
        ok_a1, xa1, ya1, tha1 = interp_track(slam_tr, ta + tau)
        ok_a2, xa2, ya2, tha2 = interp_track(slam_tr, tb + tau)
        ok_b1, xb1, yb1, thb1 = interp_track(sam_tr, ta)
        ok_b2, xb2, yb2, thb2 = interp_track(sam_tr, tb)
        ok = ok_a1 & ok_a2 & ok_b1 & ok_b2 & okb1
        for k in np.nonzero(ok)[0]:
            # relative motion expressed in the first pose's frame
            ca, sa = np.cos(tha1[k]), np.sin(tha1[k])
            dxa = ca * (xa2[k] - xa1[k]) + sa * (ya2[k] - ya1[k])
            dya = -sa * (xa2[k] - xa1[k]) + ca * (ya2[k] - ya1[k])
            cb, sb = np.cos(thb1[k]), np.sin(thb1[k])
            dxb = cb * (xb2[k] - xb1[k]) + sb * (yb2[k] - yb1[k])
            dyb = -sb * (xb2[k] - xb1[k]) + cb * (yb2[k] - yb1[k])
            rows.append((ta[k], w, np.angle(np.exp(1j * (tha2[k] - tha1[k]))),
                         np.angle(np.exp(1j * (thb2[k] - thb1[k]))), dxa, dya, dxb, dyb))
    return np.array(rows)


def residual_fn(params, P):
    tx, ty, phi = params
    dth_a, dxa, dya, dxb, dyb = P[:, 2], P[:, 4], P[:, 5], P[:, 6], P[:, 7]
    c, s = np.cos(phi), np.sin(phi)
    ca, sa = np.cos(dth_a), np.sin(dth_a)
    # A_ij X = X B_ij  (translation part):  R(dth_a) t + tA = R(phi) tB + t
    rx = (ca * tx - sa * ty) + dxa - (c * dxb - s * dyb) - tx
    ry = (sa * tx + ca * ty) + dya - (s * dxb + c * dyb) - ty
    return np.concatenate([rx, ry])


def solve_planar(P):
    # linear initialisation in (tx, ty, c, s)
    ca, sa = np.cos(P[:, 2]), np.sin(P[:, 2])
    A = np.zeros((2 * len(P), 4))
    b = np.zeros(2 * len(P))
    A[0::2, 0], A[0::2, 1] = ca - 1, -sa
    A[1::2, 0], A[1::2, 1] = sa, ca - 1
    A[0::2, 2], A[0::2, 3] = -P[:, 6], P[:, 7]
    A[1::2, 2], A[1::2, 3] = -P[:, 7], -P[:, 6]
    b[0::2], b[1::2] = -P[:, 4], -P[:, 5]
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    x0 = [sol[0], sol[1], np.arctan2(sol[3], sol[2])]
    r = least_squares(residual_fn, x0, args=(P,), loss="huber", f_scale=0.01)
    res = residual_fn(r.x, P).reshape(2, -1).T
    J = r.jac
    try:
        cov = np.linalg.inv(J.T @ J) * np.median(np.abs(r.fun)) ** 2 / 0.6745 ** 2
        sd = np.sqrt(np.diag(cov))
    except np.linalg.LinAlgError:
        sd = np.full(3, np.nan)
    return r.x, np.linalg.norm(res, axis=1), sd


# ── 3. floor height ──────────────────────────────────────────────────────────
def floor_heights(session: RsSession, d, n_frames=400, seed=0):
    rng = np.random.default_rng(seed)
    step = 4
    uu, vv = np.meshgrid(np.arange(0, session.W, step), np.arange(0, session.H, step))
    pts = np.stack([uu.ravel(), vv.ravel()], 1).astype(np.float32).reshape(-1, 1, 2)
    und = cv2.undistortPoints(pts, session.K, np.asarray(session.kc)).reshape(-1, 2)
    heights, normals = [], []
    for i in np.linspace(0, len(session) - 1, n_frames).astype(int):
        f = session.read(int(i), with_depth=True)
        z = session.align(f.depth_raw)[::step, ::step].ravel().astype(np.float64) / 1000.0
        m = (z > 0.3) & (z < 5.0)
        if m.sum() < 2000:
            continue
        P = np.column_stack([und[m, 0] * z[m], und[m, 1] * z[m], z[m]])
        below = P @ d
        cand = below > 0.35                      # well below the camera centre
        if cand.sum() < 1500:
            continue
        Q = P[cand]
        best = None
        for _ in range(150):
            s = Q[rng.choice(len(Q), 3, replace=False)]
            nrm = np.cross(s[1] - s[0], s[2] - s[0])
            nn = np.linalg.norm(nrm)
            if nn < 1e-9:
                continue
            nrm /= nn
            if nrm @ d < 0:
                nrm = -nrm
            if nrm @ d < np.cos(np.radians(8)):
                continue
            h = nrm @ s[0]
            inl = np.abs(Q @ nrm - h) < 0.02
            # prefer the lowest large horizontal surface (floor, not a table top)
            score = inl.sum() * (1.0 + 0.5 * h)
            if best is None or score > best[0]:
                best = (score, inl)
        if best is None or best[1].sum() < 1200:
            continue
        F = Q[best[1]]
        c = F.mean(0)
        _, _, Vt = np.linalg.svd(F - c, full_matrices=False)
        nrm = Vt[2] if Vt[2] @ d > 0 else -Vt[2]
        h = float(nrm @ c)
        if h < 0.5:                                  # a table top, not the floor
            continue
        heights.append(h)
        normals.append(nrm)
    return np.array(heights), np.array(normals)


def main():
    rt = REPO / "objpose" / "rt"
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--slam-traj", default=str(rt / "slam_traj_tum.txt.optimized.txt"))
    ap.add_argument("--sam-traj", default=str(rt / "sam_traj_tum.txt.optimized.txt"))
    ap.add_argument("--windows", default="0.5,1,1.5,2,3,4")
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--tau-range", type=float, default=0.06)
    ap.add_argument("--floor-frames", type=int, default=400)
    ap.add_argument("--floor-heights", default="", help="'h_slam,h_sam' to skip depth floor fitting (tests)")
    ap.add_argument("--no-validation", action="store_true")
    ap.add_argument("--out", default=str(rt / "X_slam_sam_offline.json"))
    a = ap.parse_args()
    windows = [float(v) for v in a.windows.split(",")]

    ts_a, T_a = read_tum(a.slam_traj)
    ts_b, T_b = read_tum(a.sam_traj)
    d_a, planar_a = trajectory_down(T_a)
    d_b, planar_b = trajectory_down(T_b)
    L_a, L_b = levelling(d_a), levelling(d_b)
    tr_a, tr_b = planar(ts_a, T_a, L_a), planar(ts_b, T_b, L_b)
    print(f"down SLAM {np.round(d_a, 4)} (plane rms {planar_a*100:.2f} cm, tilt p95 {np.percentile(tr_a[:, 4], 95):.2f} deg)")
    print(f"down SAM  {np.round(d_b, 4)} (plane rms {planar_b*100:.2f} cm, tilt p95 {np.percentile(tr_b[:, 4], 95):.2f} deg)")

    # time offset: rotation increments of a rigid rig must be identical
    best_tau = None
    for tau in np.round(np.arange(-a.tau_range, a.tau_range + 1e-9, 0.002), 3):
        P = rel_pairs(tr_a, tr_b, float(tau), [1.0], a.stride * 2)
        e = np.degrees(np.abs(np.angle(np.exp(1j * (P[:, 2] - P[:, 3])))))
        score = float(np.median(e))
        if best_tau is None or score < best_tau[1]:
            best_tau = (float(tau), score)
    tau = best_tau[0]
    print(f"tau from rotation increments: {tau:+.3f} s (median |dtheta_slam - dtheta_sam| {best_tau[1]:.3f} deg @1 s)")

    P = rel_pairs(tr_a, tr_b, tau, windows, a.stride)
    rot_err = np.degrees(np.abs(np.angle(np.exp(1j * (P[:, 2] - P[:, 3])))))
    P = P[rot_err < 2.0]                              # pairs whose tracks disagree are unusable
    (tx, ty, phi), res, sd = solve_planar(P)
    print(f"planar hand-eye: {len(P)} pairs, tx {tx:+.4f} ty {ty:+.4f} m, phi {np.degrees(phi):+.3f} deg | "
          f"residual med {np.median(res)*100:.2f} p90 {np.percentile(res, 90)*100:.2f} cm | "
          f"sd {sd[0]*1000:.1f}/{sd[1]*1000:.1f} mm, {np.degrees(sd[2]):.3f} deg")

    # stability: independent solves on quarters of the recording
    quarters = []
    for part in np.array_split(np.argsort(P[:, 0]), 4):
        (qx, qy, qp), qres, _ = solve_planar(P[part])
        quarters.append({"t_from_s": round(float(P[part, 0].min() - P[:, 0].min()), 1), "n": int(len(part)),
                         "d_tx_mm": round((qx - tx) * 1000, 1), "d_ty_mm": round((qy - ty) * 1000, 1),
                         "d_phi_deg": round(float(np.degrees(qp - phi)), 3)})
    print("quarters:", [(q["t_from_s"], q["d_tx_mm"], q["d_ty_mm"], q["d_phi_deg"]) for q in quarters])

    if a.floor_heights:
        h_a, h_b = (np.array([float(v)]) for v in a.floor_heights.split(","))
        n_a, n_b = d_a[None], d_b[None]
    else:
        sam = RsSession(DATASET / "SAM", -SAM_MINUS_SLAM_CLOCK_NS)
        slam = RsSession(DATASET / "SLAM", 0)
        h_a, n_a = floor_heights(slam, d_a, a.floor_frames)
        h_b, n_b = floor_heights(sam, d_b, a.floor_frames)

    def robust_h(h):
        med = np.median(h)
        mad = np.median(np.abs(h - med)) * 1.4826
        keep = np.abs(h - med) < 3 * max(mad, 0.005)
        return float(np.median(h[keep])), float(mad), int(keep.sum())
    ha, mad_a, ka = robust_h(h_a)
    hb, mad_b, kb = robust_h(h_b)
    fa = n_a.mean(0); fa /= np.linalg.norm(fa)
    fb = n_b.mean(0); fb /= np.linalg.norm(fb)
    print(f"floor SLAM: h {ha:.4f} m (MAD {mad_a*100:.2f} cm, {ka}/{len(h_a)} frames), normal vs traj-down {np.degrees(np.arccos(np.clip(fa @ d_a, -1, 1))):.2f} deg")
    print(f"floor SAM : h {hb:.4f} m (MAD {mad_b*100:.2f} cm, {kb}/{len(h_b)} frames), normal vs traj-down {np.degrees(np.arccos(np.clip(fb @ d_b, -1, 1))):.2f} deg")

    Xl = np.eye(4)
    Xl[:3, :3] = np.array([[np.cos(phi), -np.sin(phi), 0], [np.sin(phi), np.cos(phi), 0], [0, 0, 1]])
    Xl[:3, 3] = [tx, ty, ha - hb]
    La, Lb = np.eye(4), np.eye(4)
    La[:3, :3], Lb[:3, :3] = L_a, L_b
    X = La.T @ Xl @ Lb

    U, _ = load_extrinsic(DATASET / "camera_extrinsic.urdf")
    cands = {"offline_X": X, "urdf": U}
    for name, path in (("previous_axzb", rt / "X_slam_sam.json"), ("shared_map_loc", rt / "X_slam_sam_loc.json"),
                       ("shared_map_loc_ftime", rt / "X_slam_sam_loc_ftime.json"),
                       ("offline_assoc_stamps", rt / "X_slam_sam_offline.json")):
        if path.exists():
            cands[name] = np.asarray(json.loads(path.read_text())["T_slam_sam"])
    comp = {}
    for k, C in cands.items():
        if k == "offline_X":
            continue
        D = np.linalg.inv(C) @ X
        comp[k] = {"translation_diff_cm": round(float(np.linalg.norm(D[:3, 3])) * 100, 2),
                   "rotation_diff_deg": round(rot_deg(D[:3, :3]), 2)}
    spread = {}
    ftime = "ftime" in Path(a.slam_traj).name
    if ftime:
        from clock import frame_clock
        sam_times = frame_clock(DATASET / "SAM", rt / "clock")["frame_ns"]
    stem = "slam_traj_ftime.txt" if ftime else "slam_traj_tum.txt"
    for traj_name, traj in (() if a.no_validation else
                            (("per_frame_slam_poses", rt / stem),
                             ("optimized_slam_poses", rt / (stem + ".optimized.txt")))):
        spread[traj_name] = {k: object_spread(C, traj, sam_times if ftime else None) for k, C in cands.items()}

    out = {
        "T_slam_sam": X.tolist(),
        "source": "offline: trajectory-plane gravity + planar SE(2) hand-eye on ORB-SLAM3 f2000 relative motions + depth floor height",
        "inputs": {"slam_traj": a.slam_traj, "sam_traj": a.sam_traj, "windows_s": windows},
        "down": {"slam": d_a.tolist(), "sam": d_b.tolist(), "plane_rms_cm": [round(planar_a * 100, 3), round(planar_b * 100, 3)]},
        "tau_s": tau,
        "planar_handeye": {"pairs": int(len(P)), "tx_m": tx, "ty_m": ty, "phi_deg": float(np.degrees(phi)),
                           "residual_median_cm": round(float(np.median(res)) * 100, 3),
                           "residual_p90_cm": round(float(np.percentile(res, 90)) * 100, 3),
                           "sd_tx_mm": round(float(sd[0]) * 1000, 2), "sd_ty_mm": round(float(sd[1]) * 1000, 2),
                           "sd_phi_deg": round(float(np.degrees(sd[2])), 4), "quarters": quarters},
        "floor": {"h_slam_m": ha, "h_sam_m": hb, "mad_cm": [round(mad_a * 100, 2), round(mad_b * 100, 2)],
                  "frames_used": [ka, kb], "normal_vs_trajectory_deg":
                      [round(float(np.degrees(np.arccos(np.clip(fa @ d_a, -1, 1)))), 3),
                       round(float(np.degrees(np.arccos(np.clip(fb @ d_b, -1, 1)))), 3)]},
        "compare": comp,
        "object_consistency_check (reference only)": spread,
    }
    Path(a.out).write_text(json.dumps(out, indent=1))
    print(f"X t = {np.round(X[:3, 3], 4).tolist()} m")
    print("compare:", comp)
    print("object spread:", json.dumps(spread))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
