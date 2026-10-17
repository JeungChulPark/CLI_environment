"""Map-based occlusion: is a landmark hidden behind other landmarks already in the map?

The depth check cannot see an occluder that stands only a few cm in front of the object (it looks like the object's
own front face), so such a landmark collects in-view miss penalties and is deleted while it is merely hidden
(YCB-M 2026-09-26: 4 of 8 failed re-links). The map already holds the occluders: project every landmark's CAD box
into the image and measure how much of this landmark's box is covered by boxes of landmarks in front of it.
"""
from __future__ import annotations

from typing import Dict, Iterable, Optional, Sequence

import numpy as np

GRID_PX = 8   # raster cell size for the coverage test


def _box_rect(T_map_cam, T_map_obj, corners, K, img_size):
    """(u0, v0, u1, v1, z_center) of the landmark's projected CAD box, or None when not in front of the camera."""
    Tc = np.asarray(T_map_cam, np.float64)
    To = np.asarray(T_map_obj, np.float64)
    T = np.linalg.inv(Tc) @ To
    p = corners @ T[:3, :3].T + T[:3, 3]
    zc = float(T[2, 3])
    if zc <= 0.05 or (p[:, 2] <= 0.02).any():
        return None
    u = K[0][0] * p[:, 0] / p[:, 2] + K[0][2]
    v = K[1][1] * p[:, 1] / p[:, 2] + K[1][2]
    w, h = img_size
    u0, u1 = max(0.0, u.min()), min(float(w), u.max())
    v0, v1 = max(0.0, v.min()), min(float(h), v.max())
    if u1 <= u0 or v1 <= v0:
        return None
    return u0, v0, u1, v1, zc


def occluded_fraction(T_map_cam, lm, others: Iterable, extents: Dict[str, np.ndarray], K, img_size,
                      margin_m: float = 0.02) -> Optional[float]:
    """Share of ``lm``'s projected box covered by the boxes of ``others`` whose centre is at least ``margin_m``
    nearer the camera. None when ``lm`` has no extents or is not in view."""
    c = extents.get(lm.object_name)
    if c is None:
        return None
    me = _box_rect(T_map_cam, lm.T_map_obj, c, K, img_size)
    if me is None:
        return None
    u0, v0, u1, v1, z = me
    gw = max(1, int(np.ceil((u1 - u0) / GRID_PX)))
    gh = max(1, int(np.ceil((v1 - v0) / GRID_PX)))
    us = u0 + (np.arange(gw) + 0.5) * (u1 - u0) / gw
    vs = v0 + (np.arange(gh) + 0.5) * (v1 - v0) / gh
    covered = np.zeros((gh, gw), bool)
    for o in others:
        co = extents.get(o.object_name)
        if co is None:
            continue
        r = _box_rect(T_map_cam, o.T_map_obj, co, K, img_size)
        if r is None or r[4] > z - margin_m:
            continue
        covered |= ((vs >= r[1]) & (vs <= r[3]))[:, None] & ((us >= r[0]) & (us <= r[2]))[None, :]
    return float(covered.mean())


def box_corners(points: np.ndarray) -> np.ndarray:
    """8 corners of the axis-aligned box of model points (N x 3, metres, object frame)."""
    lo, hi = points.min(0), points.max(0)
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
