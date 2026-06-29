#!/usr/bin/env python3
"""Trajectory I/O + time association utilities (numpy-only).

Common CSV format: timestamp,x,y,z,qx,qy,qz,qw  (header row included).
Input reference files are TUM text: `t x y z qx qy qz qw` (space-separated).
"""
import csv
from pathlib import Path
import numpy as np

COLS = ["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"]


def load_tum(path):
    """Load TUM-format trajectory -> (t[N], xyz[N,3], quat[N,4] xyzw)."""
    t, xyz, q = [], [], []
    p = Path(path)
    if not p.exists():
        return np.empty(0), np.empty((0, 3)), np.empty((0, 4))
    for line in p.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        f = line.replace(",", " ").split()
        if len(f) < 8:
            continue
        try:
            vals = [float(x) for x in f[:8]]
        except ValueError:
            continue
        t.append(vals[0]); xyz.append(vals[1:4]); q.append(vals[4:8])
    return (np.asarray(t, float),
            np.asarray(xyz, float).reshape(-1, 3),
            np.asarray(q, float).reshape(-1, 4))


def save_csv(path, t, xyz, q):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(COLS)
        for i in range(len(t)):
            w.writerow([f"{t[i]:.6f}"] + [f"{v:.6f}" for v in xyz[i]] + [f"{v:.6f}" for v in q[i]])


def load_csv(path):
    t, xyz, q = [], [], []
    with open(path) as fh:
        r = csv.reader(fh)
        next(r, None)
        for row in r:
            if len(row) < 8:
                continue
            v = [float(x) for x in row[:8]]
            t.append(v[0]); xyz.append(v[1:4]); q.append(v[4:8])
    return (np.asarray(t, float), np.asarray(xyz, float).reshape(-1, 3),
            np.asarray(q, float).reshape(-1, 4))


def associate_interp(t_ref, xyz_ref, q_ref, t_est, xyz_est, q_est):
    """Density-independent association via relative-time linear interpolation.

    Both trajectories are normalized by their own t0 (handles RTAB whose absolute
    stamps differ run-to-run). For every reference pose whose relative time falls
    within the estimated trajectory's relative-time span, the estimated position
    is linearly interpolated and the estimated orientation is nearest-in-time.

    Returns (ref_xyz, ref_q, est_xyz_i, est_q_i, t_ref_used, coverage_frac).
    coverage_frac = fraction of reference poses covered by the estimated span
    (proxy for how much of the trajectory the new run localized).
    """
    if len(t_ref) < 2 or len(t_est) < 2:
        z3 = np.empty((0, 3)); z4 = np.empty((0, 4)); z1 = np.empty(0)
        return z3, z4, z3, z4, z1, 0.0
    rr = t_ref - t_ref[0]
    ee = t_est - t_est[0]
    order = np.argsort(ee)
    ee_s = ee[order]; xe = xyz_est[order]; qe = q_est[order]
    lo, hi = ee_s[0], ee_s[-1]
    mask = (rr >= lo) & (rr <= hi)
    if mask.sum() < 3:
        z3 = np.empty((0, 3)); z4 = np.empty((0, 4)); z1 = np.empty(0)
        return z3, z4, z3, z4, z1, float(mask.mean())
    tr = rr[mask]
    est_xyz_i = np.empty((len(tr), 3))
    for d in range(3):
        est_xyz_i[:, d] = np.interp(tr, ee_s, xe[:, d])
    j = np.searchsorted(ee_s, tr)
    j = np.clip(j, 0, len(ee_s) - 1)
    jm = np.clip(j - 1, 0, len(ee_s) - 1)
    pick = np.where(np.abs(ee_s[j] - tr) <= np.abs(ee_s[jm] - tr), j, jm)
    est_q_i = qe[pick]
    return (xyz_ref[mask], q_ref[mask], est_xyz_i, est_q_i, t_ref[mask], float(mask.mean()))


def associate(t_ref, t_est, max_dt=0.05, use_relative=True):
    """Associate est->ref by nearest timestamp.

    use_relative: subtract each trajectory's t0 first (needed for RTAB whose
    absolute stamps differ run-to-run; harmless for ORB3 which already aligns).
    Returns (idx_ref[M], idx_est[M], n_total_est, frac_matched).
    """
    if len(t_ref) == 0 or len(t_est) == 0:
        return np.empty(0, int), np.empty(0, int), len(t_est), 0.0
    rr = t_ref - (t_ref[0] if use_relative else 0.0)
    ee = t_est - (t_est[0] if use_relative else 0.0)
    order = np.argsort(rr)
    rr_s = rr[order]
    ir, ie = [], []
    for j, te in enumerate(ee):
        k = np.searchsorted(rr_s, te)
        cands = [c for c in (k - 1, k) if 0 <= c < len(rr_s)]
        best, bdt = None, max_dt
        for c in cands:
            dt = abs(rr_s[c] - te)
            if dt <= bdt:
                best, bdt = order[c], dt
        if best is not None:
            ir.append(best); ie.append(j)
    ir = np.asarray(ir, int); ie = np.asarray(ie, int)
    frac = len(ie) / len(t_est) if len(t_est) else 0.0
    return ir, ie, len(t_est), frac
