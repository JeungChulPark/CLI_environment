#!/usr/bin/env python3
"""estimate_lidar_cam_rt.py — LiDAR (VLP-16) to camera extrinsic T_cam_lidar, offline.

Stage 1 — planar hand-eye (same method as objpose/pc/estimate_rt_offline.py):
    each sensor's "down" = normal of its own trajectory plane; in gravity-levelled coordinates
    the relative motions of the ORB-SLAM3 camera trajectory and the KISS-ICP LiDAR trajectory
    give the horizontal offset and yaw. Height is unobservable from planar motion, and the
    VLP-16 (lowest ring -15 deg) does not see enough floor indoors to measure it.

Stage 2 — LiDAR-to-depth registration:
    at low-motion instants, LiDAR points inside the camera's view are pulled onto the RealSense
    depth surface (point-to-plane) jointly over many frames. Walls, tables and furniture fix
    all six degrees of freedom, including the height. Stage 1 initialises it; the height is
    first grid-searched because it is unconstrained there.

    python objpose/lidar/estimate_lidar_cam_rt.py --cam-traj <orb tum> --lidar-traj <kiss tum>
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "objpose" / "pc"))
sys.path.insert(0, str(REPO / "integration"))
from bag_io import COLOR, DEPTH, POINTS, Bag, parse_image, parse_pc2  # noqa: E402
import estimate_rt_offline  # noqa: E402
from estimate_rt_offline import levelling, planar, rel_pairs, solve_planar  # noqa: E402

estimate_rt_offline.MAX_GAP_S = 0.15
from rig_offset import read_tum  # noqa: E402


def rotdeg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def plane_down(T, hint):
    P = T[:, :3, 3]
    c = P - P.mean(0)
    _, _, Vt = np.linalg.svd(c, full_matrices=False)
    n = Vt[2]
    return (n if n @ hint > 0 else -n), float((c @ Vt[2]).std())


def se3(xi):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(xi[:3]).as_matrix()
    T[:3, 3] = xi[3:]
    return T


class DepthFrame:
    def __init__(self, depth, K, D, stride=2, zmax=4.0):
        h, w = depth.shape
        vv, uu = np.mgrid[0:h:stride, 0:w:stride]
        z = depth[::stride, ::stride].astype(np.float64) / 1000.0
        pts = np.stack([uu.ravel(), vv.ravel()], 1).astype(np.float32).reshape(-1, 1, 2)
        und = cv2.undistortPoints(pts, K, D).reshape(-1, 2)
        P = np.column_stack([und[:, 0] * z.ravel(), und[:, 1] * z.ravel(), z.ravel()]).reshape(z.shape + (3,))
        # normals from the organised grid (cross product of neighbour differences)
        du = np.zeros_like(P); dv = np.zeros_like(P)
        du[:, 1:-1] = P[:, 2:] - P[:, :-2]
        dv[1:-1] = P[2:] - P[:-2]
        N = np.cross(du, dv)
        nn = np.linalg.norm(N, axis=2, keepdims=True)
        ok = (z > 0.3) & (z < zmax) & (nn[..., 0] > 1e-9)
        # reject depth edges: neighbour depth jumps
        zz = z
        jump = np.zeros_like(zz, bool)
        jump[:, 1:-1] |= np.abs(zz[:, 2:] - zz[:, :-2]) > 0.05 * zz[:, 1:-1] + 0.02
        jump[1:-1] |= np.abs(zz[2:] - zz[:-2]) > 0.05 * zz[1:-1] + 0.02
        ok &= ~jump
        N = N / np.maximum(nn, 1e-12)
        self.P = P[ok]
        self.N = N[ok]
        self.tree = cKDTree(self.P)
        self.K, self.D, self.w, self.h = K, D, w, h


def load_pairs(dataset, lidar_tr, n_pairs, max_speed, max_yaw_rate, K, D, camera="SLAM"):
    cam = Bag(Path(dataset) / camera)
    lid = Bag(Path(dataset) / "lidar")
    idp, td = cam.stamps(DEPTH)
    il, tl = lid.stamps(POINTS)
    # motion of the lidar around each scan, from its own trajectory
    ts = lidar_tr[:, 0]
    speed = np.zeros(len(ts)); yawr = np.zeros(len(ts))
    speed[1:] = np.hypot(np.diff(lidar_tr[:, 1]), np.diff(lidar_tr[:, 2])) / np.maximum(np.diff(ts), 1e-6)
    yawr[1:] = np.degrees(np.abs(np.angle(np.exp(1j * np.diff(lidar_tr[:, 3]))))) / np.maximum(np.diff(ts), 1e-6)
    cand = []
    for k, t in enumerate(tl):
        j = int(np.argmin(np.abs(ts - t / 1e9)))
        if abs(ts[j] - t / 1e9) > 0.02:
            continue
        s = max(speed[max(j - 1, 0):j + 2]); yr = max(yawr[max(j - 1, 0):j + 2])
        if s > max_speed or yr > max_yaw_rate:
            continue
        c = int(np.argmin(np.abs(td - t)))
        if abs(td[c] - t) > 20_000_000:
            continue
        cand.append((k, c, s, yr))
    pick = [cand[i] for i in np.linspace(0, len(cand) - 1, min(n_pairs, len(cand))).astype(int)] if cand else []
    pairs = []
    for k, c, s, yr in pick:
        _, pc = parse_pc2(lid.blob(il[k]))
        X = np.column_stack([pc["x"], pc["y"], pc["z"]]).astype(np.float64)
        r = np.linalg.norm(X, axis=1)
        X = X[np.isfinite(r) & (r > 0.5) & (r < 6.0)]
        _, dep = parse_image(cam.blob(idp[c]))
        pairs.append({"t_lidar": int(tl[k]), "t_cam": int(td[c]), "L": X, "F": DepthFrame(dep, K, D)})
    return pairs, len(cand)


def residuals(T, pairs, max_d, per_pair_cap=1500, rng=None):
    out = []
    for p in pairs:
        Q = (T[:3, :3] @ p["L"].T).T + T[:3, 3]
        F = p["F"]
        front = Q[:, 2] > 0.4
        Q = Q[front]
        uv, _ = cv2.projectPoints(Q, np.zeros(3), np.zeros(3), F.K, F.D)
        uv = uv.reshape(-1, 2)
        inside = (uv[:, 0] > 5) & (uv[:, 0] < F.w - 5) & (uv[:, 1] > 5) & (uv[:, 1] < F.h - 5)
        Q = Q[inside]
        if len(Q) > per_pair_cap:
            Q = Q[(rng or np.random.default_rng(0)).choice(len(Q), per_pair_cap, replace=False)]
        d, idx = F.tree.query(Q, distance_upper_bound=max_d)
        ok = np.isfinite(d)
        if ok.sum():
            out.append((Q[ok], F.P[idx[ok]], F.N[idx[ok]]))
    return out


def icp(T0, pairs, schedule=(0.40, 0.25, 0.15, 0.10, 0.07, 0.05, 0.05, 0.05), fix_height_dir=None):
    T = T0.copy()
    rng = np.random.default_rng(1)
    stats = None
    for max_d in schedule:
        corr = residuals(T, pairs, max_d, rng=rng)
        if not corr:
            break
        # express correspondences in the lidar frame so the update composes as T <- T . exp(xi)
        Ls, Ss, Ns = [], [], []
        for Q, S, N in corr:
            Ls.append((T[:3, :3].T @ (Q - T[:3, 3]).T).T); Ss.append(S); Ns.append(N)
        L, S, N = np.vstack(Ls), np.vstack(Ss), np.vstack(Ns)

        def f(xi):
            Tn = T @ se3(xi)
            Q = (Tn[:3, :3] @ L.T).T + Tn[:3, 3]
            return np.sum((Q - S) * N, axis=1)
        r = least_squares(f, np.zeros(6), loss="huber", f_scale=0.02)
        T = T @ se3(r.x)
        res = np.abs(f(r.x))
        stats = {"max_corr_m": max_d, "n": int(len(res)), "median_cm": round(float(np.median(res)) * 100, 2),
                 "p90_cm": round(float(np.percentile(res, 90)) * 100, 2)}
    return T, stats


def main():
    rt = REPO / "objpose" / "rt" / "260910"
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", default="/home/jucpark/DeepLearning/Dataset/260910_object")
    ap.add_argument("--camera", default="SLAM", choices=["SLAM", "SAM"], help="dataset camera folder the trajectory belongs to")
    ap.add_argument("--cam-traj", default=str(rt / "slam_orb_tum.txt.optimized.txt"))
    ap.add_argument("--stage2", action="store_true", help="also run the (unstable here) 6-DoF depth registration")
    ap.add_argument("--lidar-traj", default=str(REPO / "objpose" / "lidar" / "feasibility_mac" / "lidar_traj_tum.txt"))
    ap.add_argument("--pairs", type=int, default=120)
    ap.add_argument("--max-speed", type=float, default=0.12)
    ap.add_argument("--max-yaw-rate", type=float, default=8.0)
    ap.add_argument("--out", default=str(rt / "T_cam_lidar.json"))
    a = ap.parse_args()

    info = json.loads((Path(a.dataset) / "info.json").read_text())
    K = np.array(info[a.camera]["K"], np.float64).reshape(3, 3)
    D = np.array(info[a.camera]["D"], np.float64)

    ts_c, T_c = read_tum(a.cam_traj)
    ts_l, T_l = read_tum(a.lidar_traj)
    d_c, pr_c = plane_down(T_c, np.array([0, 1, 0]))      # optical frame: y down
    d_l, pr_l = plane_down(T_l, np.array([0, 0, -1]))     # velodyne frame: z up
    L_c, L_l = levelling(d_c), levelling(d_l)
    tr_c, tr_l = planar(ts_c, T_c, L_c), planar(ts_l, T_l, L_l)
    print(f"down cam {np.round(d_c, 4)} (plane rms {pr_c*100:.2f} cm) | down lidar {np.round(d_l, 4)} (plane rms {pr_l*100:.2f} cm)")

    best = None
    for tau in np.round(np.arange(-0.10, 0.1001, 0.002), 3):
        P = rel_pairs(tr_c, tr_l, float(tau), [1.0], 6)
        if len(P) < 50:
            continue
        e = float(np.median(np.degrees(np.abs(np.angle(np.exp(1j * (P[:, 2] - P[:, 3])))))))
        if best is None or e < best[1]:
            best = (float(tau), e)
    tau = best[0]
    P = rel_pairs(tr_c, tr_l, tau, [0.5, 1, 1.5, 2, 3, 4], 3)
    P = P[np.degrees(np.abs(np.angle(np.exp(1j * (P[:, 2] - P[:, 3]))))) < 2.0]
    (tx, ty, phi), res, sd = solve_planar(P)
    print(f"tau {tau:+.3f} s (rotation increments agree to {best[1]:.3f} deg @1 s) | planar hand-eye {len(P)} pairs: "
          f"tx {tx:+.4f} ty {ty:+.4f} phi {np.degrees(phi):+.3f} deg | residual med {np.median(res)*100:.2f} cm")

    def compose(dz):
        Xl = np.eye(4)
        Xl[:3, :3] = [[np.cos(phi), -np.sin(phi), 0], [np.sin(phi), np.cos(phi), 0], [0, 0, 1]]
        Xl[:3, 3] = [tx, ty, dz]
        A, B = np.eye(4), np.eye(4)
        A[:3, :3], B[:3, :3] = L_c, L_l
        return A.T @ Xl @ B

    pairs, n_cand = load_pairs(a.dataset, tr_l, a.pairs, a.max_speed, a.max_yaw_rate, K, D, a.camera)
    print(f"registration pairs: {len(pairs)} (of {n_cand} low-motion scans)")

    # height: planar motion cannot observe it, and the depth residual is flat in it here, but the
    # number of LiDAR points that land on the camera's depth surface peaks at the right height
    scan = []
    for dz in np.arange(-1.2, 1.2001, 0.05):
        corr = residuals(compose(dz), pairs, 0.10, per_pair_cap=800)
        scan.append((float(dz), int(sum(len(c[0]) for c in corr))))
    dzs = np.array([s_[0] for s_ in scan]); cnt = np.array([s_[1] for s_ in scan], float)
    k = int(np.argmax(cnt))
    dz_peak = dzs[k]
    if 0 < k < len(cnt) - 1:                       # parabola through the peak and its neighbours
        y0, y1, y2 = cnt[k - 1], cnt[k], cnt[k + 1]
        den = y0 - 2 * y1 + y2
        if den < 0:
            dz_peak = float(dzs[k] + 0.05 * 0.5 * (y0 - y2) / den)
    print(f"height by overlap maximum: {dz_peak:+.3f} m (peak {int(cnt[k])} points)")
    X_final = compose(dz_peak)
    final = {"T_cam_lidar": X_final.tolist(), "camera": a.camera,
             "method": "planar hand-eye (tx, ty, yaw, tilt) + height from LiDAR/depth overlap maximum",
             "height_along_down_m": dz_peak, "height_uncertainty_m": 0.1, "tau_s": tau,
             "stage1": {"pairs": int(len(P)), "tx_m": tx, "ty_m": ty, "phi_deg": float(np.degrees(phi)),
                        "residual_median_cm": round(float(np.median(res)) * 100, 2)},
             "overlap_scan": scan}
    final_path = Path(a.out).with_name(Path(a.out).stem + "_final.json")
    final_path.parent.mkdir(parents=True, exist_ok=True)
    final_path.write_text(json.dumps(final, indent=1))
    print("final T_cam_lidar t =", np.round(X_final[:3, 3], 4).tolist(), "->", final_path)
    if not a.stage2:
        return

    # height grid search (unobservable in stage 1), then full 6-DoF refinement
    grid = []
    for dz in np.arange(-1.0, 1.01, 0.1):
        T, st = icp(compose(dz), pairs, schedule=(0.40, 0.25, 0.15))
        if st:
            grid.append((st["median_cm"] / max(st["n"], 1) ** 0.25, dz, T, st))
    grid.sort(key=lambda g: g[0])
    print("height grid (best 4):", [(round(g[1], 1), g[3]["median_cm"], g[3]["n"]) for g in grid[:4]])
    T_best, st = icp(grid[0][2], pairs)
    print("refined residual:", st)

    # stability: refine on each third of the pairs separately
    thirds = []
    for part in np.array_split(np.arange(len(pairs)), 3):
        Tp, stp = icp(T_best, [pairs[i] for i in part])
        Dd = np.linalg.inv(T_best) @ Tp
        thirds.append({"pairs": int(len(part)), "d_trans_cm": round(float(np.linalg.norm(Dd[:3, 3])) * 100, 2),
                       "d_rot_deg": round(rotdeg(Dd[:3, :3]), 3), "residual": stp})
    print("thirds:", [(t["d_trans_cm"], t["d_rot_deg"]) for t in thirds])

    X1 = compose(float(grid[0][1]))
    D1 = np.linalg.inv(X1) @ T_best
    out = {
        "T_cam_lidar": T_best.tolist(),
        "frames": "camera = SLAM-folder RealSense colour optical frame; lidar = velodyne frame",
        "tau_s": tau,
        "stage1_planar_handeye": {"pairs": int(len(P)), "tx_m": tx, "ty_m": ty, "phi_deg": float(np.degrees(phi)),
                                  "residual_median_cm": round(float(np.median(res)) * 100, 2),
                                  "sd_mm_deg": [round(float(sd[0]) * 1000, 2), round(float(sd[1]) * 1000, 2), round(float(np.degrees(sd[2])), 4)]},
        "stage2_registration": {"pairs": len(pairs), "low_motion_candidates": n_cand, "residual": st,
                                "height_grid_best": [(round(g[1], 1), g[3]) for g in grid[:4]],
                                "change_vs_stage1": {"trans_cm": round(float(np.linalg.norm(D1[:3, 3])) * 100, 2),
                                                     "rot_deg": round(rotdeg(D1[:3, :3]), 3)},
                                "thirds": thirds},
    }
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(out, indent=1))
    print("T_cam_lidar t =", np.round(T_best[:3, 3], 4).tolist(), "| rot vs stage1 %.2f deg" % rotdeg(D1[:3, :3]))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
