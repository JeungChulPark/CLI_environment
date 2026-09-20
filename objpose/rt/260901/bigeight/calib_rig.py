#!/usr/bin/env python3
"""calib_rig.py — rig calibration of the 260901 Husky rig from the data itself (no targets):

  camera <-> VLP-16   time offset + T_lidar_cam   (needed to score camera SLAM against the KISS-ICP reference)
  camera <-> xsens    time offset + R_cam_imu + gyro bias   (needed to feed the gyro to ORB-SLAM3)

Time offsets come from the angular SPEED |w| (invariant to the unknown rotation between the sensors), the
rotations from aligning the angular-velocity vectors, and T_lidar_cam from hand-eye on short relative motions
(A X = X B over 2 s windows -> insensitive to slow SLAM drift). The robot moves in a plane, so the lever arm
along the rotation axis is unobservable (regularised to 0; it does not affect any trajectory comparison).

    python3 calib_rig.py --cam <ORB TUM> --lidar kiss/kiss_tum.txt --imu imu.npz --out rig_260901_bigeight.json
"""
from __future__ import annotations
import argparse, json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from trajlib import load_tum, Traj, body_rate, rotvec, inv, se3


def best_shift(t_a, s_a, t_b, s_b, span=0.6, step=0.002):
    """tau maximising corr(s_a(t), s_b(t + tau)): s_b's clock is tau ahead of s_a's for the same event"""
    grid = np.arange(max(t_a[0], t_b[0]) + span + 1, min(t_a[-1], t_b[-1]) - span - 1, 0.02)
    a = np.interp(grid, t_a, s_a); a = a - a.mean()
    best = (-2, 0.0); curve = []
    for tau in np.arange(-span, span + 1e-9, step):
        b = np.interp(grid + tau, t_b, s_b); b = b - b.mean()
        c = float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12))
        curve.append((tau, c))
        if c > best[0]:
            best = (c, tau)
    curve = np.array(curve)
    i = int(np.argmax(curve[:, 1]))
    if 0 < i < len(curve) - 1:           # parabolic refinement
        y0, y1, y2 = curve[i - 1:i + 2, 1]
        tau = curve[i, 0] + 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2) * step
    else:
        tau = best[1]
    return float(tau), float(best[0])


def kabsch_vectors(a, b, w=None):
    """R minimising sum w |a - R b|^2 for vector sets a, b [N,3]; returns R, singular values"""
    w = np.ones(len(a)) if w is None else w
    H = (b * w[:, None]).T @ a
    U, S, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    return Vt.T @ D @ U.T, S


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", required=True); ap.add_argument("--lidar", required=True)
    ap.add_argument("--imu", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--static-s", type=float, default=25.0, help="initial standstill used for the gyro bias")
    ap.add_argument("--win-s", type=float, default=2.0)
    a = ap.parse_args()
    tc, Tc = load_tum(a.cam); tl, Tl = load_tum(a.lidar)
    imu = np.load(a.imu)
    ti = imu["t_hdr_ns"] * 1e-9; gyro = imu["gyro"]; acc = imu["acc"]
    out = {"cam": str(a.cam), "lidar": str(a.lidar), "imu": str(a.imu)}

    # ---------------- gyro bias from the initial standstill
    st = ti < ti[0] + a.static_s
    bias = gyro[st].mean(0)
    out["gyro_bias_rad_s"] = bias.tolist()
    out["gyro_static_std_rad_s"] = gyro[st].std(0).tolist()
    out["acc_static_mean"] = acc[st].mean(0).tolist()
    g = gyro - bias

    # ---------------- time offsets from |w|
    sm = 0.3
    tcm, wc = body_rate(tc, Tc, sm); tlm, wl = body_rate(tl, Tl, 0.0)
    k = max(1, int(round(sm / 0.005))); ker = np.ones(k) / k
    gs = np.column_stack([np.convolve(g[:, j], ker, mode="same") for j in range(3)])
    # lidar at 10 Hz: smooth camera to the same effective window for the lidar comparison
    tau_cl, c_cl = best_shift(tcm, np.linalg.norm(wc, axis=1), tlm, np.linalg.norm(wl, axis=1))
    tau_ci, c_ci = best_shift(tcm, np.linalg.norm(wc, axis=1), ti, np.linalg.norm(gs, axis=1))
    tau_li, c_li = best_shift(tlm, np.linalg.norm(wl, axis=1), ti, np.linalg.norm(gs, axis=1))
    out["time"] = {"lidar_minus_cam_s": tau_cl, "corr_cl": c_cl, "imu_minus_cam_s": tau_ci, "corr_ci": c_ci,
                   "imu_minus_lidar_s": tau_li, "corr_li": c_li,
                   "meaning": "an event stamped t on the camera clock is stamped t + tau on the other sensor's clock",
                   "closure_s": tau_cl + tau_li - tau_ci}

    # ---------------- R_cam_imu from angular velocity vectors (w_c = R_ci w_i)
    gi = np.column_stack([np.interp(tcm + tau_ci, ti, gs[:, j]) for j in range(3)])
    mv = np.linalg.norm(wc, axis=1) > 0.03
    R_ci, S = kabsch_vectors(wc[mv], gi[mv])
    res = wc[mv] - gi[mv] @ R_ci.T
    out["R_cam_imu"] = R_ci.tolist()
    out["R_cam_imu_fit"] = {"n": int(mv.sum()), "singular_values": S.tolist(),
                            "resid_rms_rad_s": float(np.sqrt((res ** 2).sum(1).mean())),
                            "rate_rms_rad_s": float(np.sqrt((wc[mv] ** 2).sum(1).mean())),
                            "imu_axis_std_rad_s": gi[mv].std(0).tolist()}

    # ---------------- T_lidar_cam by hand-eye on relative motions
    Lc = Traj(tl, Tl, 0.25); Cc = Traj(tc, Tc, 0.2)
    t0 = np.arange(max(tc[0], tl[0] - tau_cl) + 0.5, min(tc[-1], tl[-1] - tau_cl) - a.win_s - 0.5, 0.25)
    C0, ok0 = Cc.at(t0); C1, ok1 = Cc.at(t0 + a.win_s)
    L0, ok2 = Lc.at(t0 + tau_cl); L1, ok3 = Lc.at(t0 + a.win_s + tau_cl)
    ok = ok0 & ok1 & ok2 & ok3
    A = inv(L0[ok]) @ L1[ok]; B = inv(C0[ok]) @ C1[ok]
    moving = (np.linalg.norm(rotvec(A[:, :3, :3]), axis=1) > np.radians(3)) | (np.linalg.norm(A[:, :3, 3], axis=1) > 0.1)
    A, B = A[moving], B[moving]
    R0, _ = kabsch_vectors(rotvec(A[:, :3, :3]), rotvec(B[:, :3, :3]))

    def resid(x):
        RX = Rotation.from_rotvec(x[:3]).as_matrix(); tX = x[3:]
        er = rotvec(np.einsum("nij,jk,nlk,ml->nim", A[:, :3, :3], RX, B[:, :3, :3], RX))   # R_A R_X R_B^T R_X^T
        et = np.einsum("nij,j->ni", A[:, :3, :3], tX) + A[:, :3, 3] - B[:, :3, 3] @ RX.T - tX
        return np.concatenate([er.ravel() * 2.0, et.ravel(), [1e-3 * x[5]]])

    sol = least_squares(resid, np.concatenate([Rotation.from_matrix(R0).as_rotvec(), [0, 0, 0]]), loss="soft_l1", f_scale=0.02)
    RX = Rotation.from_rotvec(sol.x[:3]).as_matrix(); tX = sol.x[3:]
    r = resid(sol.x)[:-1]; n = len(A)
    out["T_lidar_cam"] = se3(RX, tX).tolist()
    out["T_lidar_cam_fit"] = {"windows": int(n), "win_s": a.win_s,
                              "rot_resid_deg_rms": float(np.degrees(np.sqrt(((r[:3 * n] / 2.0).reshape(n, 3) ** 2).sum(1).mean()))),
                              "trans_resid_m_rms": float(np.sqrt((r[3 * n:].reshape(n, 3) ** 2).sum(1).mean())),
                              "note": "lever arm along the (vertical) rotation axis is unobservable in planar motion; regularised to 0",
                              "cam_z_in_lidar": RX[:, 2].tolist(), "cam_y_in_lidar": RX[:, 1].tolist()}

    # ---------------- gyro quality: integrated gyro rotation vs KISS rotation (through the two extrinsics)
    dt = np.diff(ti)
    Rg = [Rotation.identity()]
    dR = Rotation.from_rotvec(0.5 * (g[:-1] + g[1:]) * dt[:, None])
    for k_ in range(len(dR)):
        Rg.append(Rg[-1] * dR[k_])
    Rg = Rotation.concatenate(Rg)                                   # R_i0_i(t)
    from scipy.spatial.transform import Slerp
    sl = Slerp(ti, Rg)
    tq = tl[(tl + tau_li > ti[0]) & (tl + tau_li < ti[-1])]
    Rgi = sl(tq + tau_li).as_matrix()
    R_li = RX @ R_ci                                                # lidar <- imu
    Rl = Tl[np.isin(tl, tq), :3, :3]
    # relative rotation from the first sample, expressed in the lidar frame
    dG = np.einsum("ij,njk,kl->nil", R_li, np.einsum("ji,njk->nik", Rgi[0], Rgi), R_li.T)
    dL = np.einsum("ji,njk->nik", Rl[0], Rl)
    err = np.degrees(np.linalg.norm(rotvec(np.einsum("nji,njk->nik", dL, dG)), axis=1))
    out["gyro_vs_kiss"] = {"err_deg_end": float(err[-1]), "err_deg_max": float(err.max()), "err_deg_median": float(np.median(err)),
                           "duration_s": float(tq[-1] - tq[0])}
    Path(a.out).write_text(json.dumps(out, indent=1))
    print(json.dumps({k_: v for k_, v in out.items() if k_ not in ("R_cam_imu", "T_lidar_cam")}, indent=1))
    np.set_printoptions(precision=4, suppress=True)
    print("R_cam_imu=\n", R_ci, "\nT_lidar_cam=\n", se3(RX, tX))


if __name__ == "__main__":
    main()
