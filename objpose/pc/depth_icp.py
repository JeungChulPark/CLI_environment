"""depth_icp.py — where is a map object really, in this depth frame? (no SAM-6D)

The object's CAD points are put in the camera with the pose the display would use; their visible front surface
(z-buffer) is aligned to the depth points around the object by translation-only ICP with a shrinking gate and a
robust (median) step. The result is the shift of the object in the camera frame, how much of the surface found a
depth point close by (inlier ratio), and the residual — a camera-object observation available at camera rate.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree


class DepthICP:
    def __init__(self, model_points: dict, n_model=400, ds=2, margin_px=25, stride=3):
        self.models = model_points          # name -> Nx3 [m] in the object frame
        self.n_model, self.ds, self.margin, self.stride = n_model, ds, margin_px, stride

    def front_points(self, T_cam_obj, name, K, W, H):
        P = self.models[name] @ T_cam_obj[:3, :3].T + T_cam_obj[:3, 3]
        P = P[P[:, 2] > 0.1]
        if len(P) == 0:
            return None, None
        u = K[0, 0] * P[:, 0] / P[:, 2] + K[0, 2]
        v = K[1, 1] * P[:, 1] / P[:, 2] + K[1, 2]
        ok = (u >= 0) & (u < W) & (v >= 0) & (v < H)
        if ok.sum() < 30:
            return None, None
        P, u, v = P[ok], u[ok], v[ok]
        idx = (v // self.ds).astype(int) * (W // self.ds + 1) + (u // self.ds).astype(int)
        order = np.argsort(P[:, 2])
        _, first = np.unique(idx[order], return_index=True)           # nearest point per pixel = visible surface
        keep = order[first]
        if len(keep) > self.n_model:
            keep = keep[np.random.default_rng(0).choice(len(keep), self.n_model, replace=False)]
        return P[keep], (u[keep].min(), u[keep].max(), v[keep].min(), v[keep].max())

    def depth_points(self, depth_m, K, box, zlo, zhi):
        H, W = depth_m.shape
        u0, u1, v0, v1 = box
        u0 = int(max(0, u0 - self.margin)); u1 = int(min(W - 1, u1 + self.margin))
        v0 = int(max(0, v0 - self.margin)); v1 = int(min(H - 1, v1 + self.margin))
        s = self.stride
        vv, uu = np.mgrid[v0:v1 + 1:s, u0:u1 + 1:s]
        z = depth_m[vv, uu]
        m = (z > zlo) & (z < zhi)
        z, uu, vv = z[m], uu[m], vv[m]
        return np.c_[(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z]

    def align(self, T_cam_obj, name, depth_m, K, iters=12, g0=0.10, g1=0.03):
        """-> dict(shift 3-vector [m] in the camera, inlier ratio, residual median [m]) or None"""
        H, W = depth_m.shape
        P, box = self.front_points(T_cam_obj, name, K, W, H)
        if P is None:
            return None
        Q = self.depth_points(depth_m, K, box, P[:, 2].min() - 0.3, P[:, 2].max() + 0.3)
        if len(Q) < 50:
            return None
        tree = cKDTree(Q)
        d_shift = np.zeros(3)
        for k in range(iters):
            g = g0 + (g1 - g0) * k / max(iters - 1, 1)
            dist, j = tree.query(P + d_shift, distance_upper_bound=g)
            inl = np.isfinite(dist)
            if inl.sum() < 30:
                return None
            step = np.median(Q[j[inl]] - (P[inl] + d_shift), axis=0)
            d_shift += step
            if np.linalg.norm(step) < 0.001:
                break
        dist, _ = tree.query(P + d_shift, distance_upper_bound=0.05)
        fin = np.isfinite(dist)
        return {"shift": d_shift, "inlier": float((dist < 0.015).mean()), "resid": float(np.median(dist[fin])) if fin.any() else 1.0,
                "n": int(len(P))}
