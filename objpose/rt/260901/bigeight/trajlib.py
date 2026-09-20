"""trajlib.py — small SE(3) trajectory helpers shared by calib_rig.py / eval_traj.py (numpy + scipy only)."""
from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def load_tum(path):
    """TUM file -> (t_s float64 [N], T [N,4,4]); rows sorted by time, duplicate stamps dropped."""
    A = np.loadtxt(path, ndmin=2)
    A = A[np.argsort(A[:, 0], kind="stable")]
    A = A[np.concatenate([[True], np.diff(A[:, 0]) > 0])]
    T = np.tile(np.eye(4), (len(A), 1, 1))
    T[:, :3, :3] = Rotation.from_quat(A[:, 4:8]).as_matrix()
    T[:, :3, 3] = A[:, 1:4]
    return A[:, 0].copy(), T


def inv(T):
    Ti = np.tile(np.eye(4), T.shape[:-2] + (1, 1)) if T.ndim == 3 else np.eye(4)
    R = np.swapaxes(T[..., :3, :3], -1, -2)
    Ti[..., :3, :3] = R
    Ti[..., :3, 3] = -np.einsum("...ij,...j->...i", R, T[..., :3, 3])
    return Ti


class Traj:
    """pose interpolation (slerp + linear) with a validity mask: no interpolation across gaps > max_gap_s"""

    def __init__(self, t, T, max_gap_s=0.5):
        self.t, self.T = np.asarray(t, float), np.asarray(T, float)
        self.slerp = Slerp(self.t, Rotation.from_matrix(self.T[:, :3, :3]))
        self.max_gap = max_gap_s

    def at(self, tq):
        tq = np.asarray(tq, float)
        ok = (tq >= self.t[0]) & (tq <= self.t[-1])
        tc = np.clip(tq, self.t[0], self.t[-1])
        i = np.clip(np.searchsorted(self.t, tc, side="right") - 1, 0, len(self.t) - 2)
        ok &= (self.t[i + 1] - self.t[i]) <= self.max_gap
        w = (tc - self.t[i]) / (self.t[i + 1] - self.t[i])
        out = np.tile(np.eye(4), (len(tq), 1, 1))
        out[:, :3, :3] = self.slerp(tc).as_matrix()
        out[:, :3, 3] = self.T[i, :3, 3] * (1 - w)[:, None] + self.T[i + 1, :3, 3] * w[:, None]
        return out, ok


def rotvec(R):
    return Rotation.from_matrix(R).as_rotvec()


def angle_deg(R):
    return np.degrees(np.linalg.norm(rotvec(R), axis=-1))


def body_rate(t, T, smooth_s=0.0):
    """body-frame angular velocity (rad/s) at interval midpoints from a pose sequence"""
    dR = np.einsum("nji,njk->nik", T[:-1, :3, :3], T[1:, :3, :3])
    dt = np.diff(t)
    w = rotvec(dR) / dt[:, None]
    tm = 0.5 * (t[:-1] + t[1:])
    if smooth_s > 0:
        k = max(1, int(round(smooth_s / np.median(dt))))
        ker = np.ones(k) / k
        w = np.column_stack([np.convolve(w[:, a], ker, mode="same") for a in range(3)])
    return tm, w


def umeyama_se3(A, B):
    """rigid (R, t) minimising |R A + t - B| (A, B: [N,3])"""
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cb - R @ ca


def se3(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    return T
