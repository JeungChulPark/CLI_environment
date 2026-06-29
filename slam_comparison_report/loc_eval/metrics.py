#!/usr/bin/env python3
"""Alignment + trajectory metrics (numpy-only).

Provides Umeyama SE3/Sim3 alignment, ATE, RPE, translation/rotation error,
and tracking-lost / pose-jump detection.
"""
import numpy as np


def quat_to_rot(q):
    """xyzw quaternion -> 3x3 rotation. q: [...,4]."""
    q = np.asarray(q, float)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    n = np.sqrt(x * x + y * y + z * z + w * w)
    n = np.where(n == 0, 1.0, n)
    x, y, z, w = x / n, y / n, z / n, w / n
    R = np.empty(q.shape[:-1] + (3, 3))
    R[..., 0, 0] = 1 - 2 * (y * y + z * z)
    R[..., 0, 1] = 2 * (x * y - z * w)
    R[..., 0, 2] = 2 * (x * z + y * w)
    R[..., 1, 0] = 2 * (x * y + z * w)
    R[..., 1, 1] = 1 - 2 * (x * x + z * z)
    R[..., 1, 2] = 2 * (y * z - x * w)
    R[..., 2, 0] = 2 * (x * z - y * w)
    R[..., 2, 1] = 2 * (y * z + x * w)
    R[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def rot_geodesic_deg(Ra, Rb):
    """Geodesic angle (deg) between rotation matrix stacks Ra,Rb [N,3,3]."""
    Rrel = np.matmul(Ra.transpose(0, 2, 1), Rb)
    tr = np.clip((np.trace(Rrel, axis1=1, axis2=2) - 1.0) / 2.0, -1.0, 1.0)
    return np.degrees(np.arccos(tr))


def umeyama(src, dst, with_scale):
    """Align src->dst (both [N,3]). Returns (s, R, t) so dst ~= s*R@src + t."""
    n = src.shape[0]
    mu_s = src.mean(0); mu_d = dst.mean(0)
    sc = src - mu_s; dc = dst - mu_d
    cov = (dc.T @ sc) / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    if with_scale:
        var_s = (sc ** 2).sum() / n
        s = (D * np.diag(S)).sum() / var_s if var_s > 0 else 1.0
    else:
        s = 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def apply_sim3(xyz, s, R, t):
    return (s * (R @ xyz.T)).T + t


def stats(arr):
    if len(arr) == 0:
        return {"mean": None, "median": None, "max": None, "std": None, "rmse": None, "n": 0}
    a = np.asarray(arr, float)
    return {"mean": float(a.mean()), "median": float(np.median(a)),
            "max": float(a.max()), "std": float(a.std()),
            "rmse": float(np.sqrt((a ** 2).mean())), "n": int(len(a))}


def ate_rpe(ref_xyz, est_xyz, ref_q=None, est_q=None, with_scale=True):
    """Align est to ref (paired, same length) and compute ATE/rotation/RPE."""
    s, R, t = umeyama(est_xyz, ref_xyz, with_scale)
    est_a = apply_sim3(est_xyz, s, R, t)
    ate = np.linalg.norm(est_a - ref_xyz, axis=1)
    out = {"alignment": {"type": "Sim3" if with_scale else "SE3",
                         "scale": float(s)},
           "ate_m": stats(ate)}
    # rotation error after alignment
    if ref_q is not None and est_q is not None and len(ref_q) == len(est_q):
        Rr = quat_to_rot(ref_q)
        Re = np.matmul(R[None], quat_to_rot(est_q))
        out["rot_err_deg"] = stats(rot_geodesic_deg(Rr, Re))
    # RPE: frame-to-frame translation delta difference (uses aligned est)
    if len(ref_xyz) >= 2:
        dref = np.linalg.norm(np.diff(ref_xyz, axis=0), axis=1)
        dest = np.linalg.norm(np.diff(est_a, axis=0), axis=1)
        out["rpe_trans_m"] = stats(np.abs(dref - dest))
    return out, est_a, (s, R, t)


def detect_tracking_issues(t, xyz, jump_factor=3.0, gap_factor=3.0):
    """Return dict of tracking-lost gaps and pose jumps using time/space heuristics."""
    res = {"n_pose": int(len(t)), "gaps": [], "jumps": []}
    if len(t) < 3:
        return res
    dt = np.diff(t)
    med_dt = float(np.median(dt))
    step = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    med_step = float(np.median(step)) if len(step) else 0.0
    gap_thr = med_dt * gap_factor
    jump_thr = max(med_step * jump_factor, 1e-6)
    for i in range(len(dt)):
        if dt[i] > gap_thr:
            res["gaps"].append({"i": int(i), "t0": float(t[i]), "t1": float(t[i + 1]),
                                "dt": float(dt[i])})
        if step[i] > jump_thr and dt[i] <= gap_thr:
            res["jumps"].append({"i": int(i), "step_m": float(step[i])})
    res["median_dt"] = med_dt
    res["median_step_m"] = med_step
    res["n_gaps"] = len(res["gaps"]); res["n_jumps"] = len(res["jumps"])
    return res
