"""Depth-consistency evidence for landmarks that were not detected in a frame (2026-09-17).

The existence filter alone cannot tell "not detected but still there" from "not there any more": a landmark in view
that is missed only loses existence slowly (P_D = 0.5), and a landmark with enough observations becomes `remembered`,
which misses never touch. After a SLAM jump or a loop closure in a world-anchored map this leaves the old copy of an
object in the map forever, next to the new one.

The aligned depth image of the same frame answers the question directly. Project the landmark centre (CAD models are
centred on their origin) into the frame and look at the measured depth in a small patch:

    measured clearly FARTHER than the landmark  -> the camera sees through the spot: FREE space, the object is gone
    measured clearly CLOSER                     -> something is in front of it: OCCLUDED, no evidence either way
    about the same                               -> SURFACE: something is there (maybe the object, not detected)
    too few valid pixels / outside the image     -> UNKNOWN

The caller (pipeline.object_memory_runner) deletes a landmark after several consecutive FREE frames and suppresses the
miss penalty for OCCLUDED ones. Margins must exceed the pose error of the SLAM->SAM chain (3-17 cm measured while
moving), so the caller also skips the check while the camera moves fast.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

FREE, OCCLUDED, SURFACE, UNKNOWN = "free", "occluded", "surface", "unknown"


@dataclass
class DepthFrame:
    """A (possibly downsampled) depth image with the intrinsics of THAT image."""
    depth_mm: np.ndarray            # HxW uint16/float, 0 = invalid
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_full(cls, depth_small: np.ndarray, K: Sequence[Sequence[float]], full_size: Tuple[int, int]):
        """depth_small = depth_full[::s, ::s]; K and full_size (w, h) of the full-resolution image"""
        w, h = full_size
        sx = depth_small.shape[1] / float(w)
        sy = depth_small.shape[0] / float(h)
        return cls(depth_small, K[0][0] * sx, K[1][1] * sy, (K[0][2] + 0.5) * sx - 0.5, (K[1][2] + 0.5) * sy - 0.5)


@dataclass
class DepthCheckConfig:
    free_margin_m: float = 0.15       # measured farther than landmark + this -> free
    occluded_margin_m: float = 0.10   # measured closer than landmark - this -> occluded
    free_fraction: float = 0.8        # share of valid patch pixels that must be farther
    occluded_fraction: float = 0.5
    min_valid_fraction: float = 0.5   # patch pixels with a usable depth
    radius_m: float = 0.03            # centre patch half-size (surface / occluded test; objects are 10-25 cm)
    free_radius_m: float = 0.15       # FREE needs the whole window of this half-size farther: the object (>= 5 cm
                                      # radius) must be missing even if the pose is 10-15 cm off. With the 3 cm patch
                                      # alone, ORB-SLAM3 f2000 deleted all 9 real objects on inlong_middle (2026-09-17):
                                      # a few degrees of rotation error put the patch beside the object, onto the wall 4 m behind
    free_window_fraction: float = 0.95
    min_depth_m: float = 0.25
    max_depth_m: float = 6.0          # D455f depth beyond this is too noisy to call free space
    border_px: int = 1
    confirm_frames: int = 3           # consecutive FREE frames before deletion
    max_speed_mps: float = 0.6        # skip the check above this camera speed (0.10 left it idle while walking)
    max_rot_dps: float = 45.0


def _point_in_camera(T_map_cam, T_map_obj):
    dx = T_map_obj[0][3] - T_map_cam[0][3]
    dy = T_map_obj[1][3] - T_map_cam[1][3]
    dz = T_map_obj[2][3] - T_map_cam[2][3]
    x = T_map_cam[0][0] * dx + T_map_cam[1][0] * dy + T_map_cam[2][0] * dz
    y = T_map_cam[0][1] * dx + T_map_cam[1][1] * dy + T_map_cam[2][1] * dz
    z = T_map_cam[0][2] * dx + T_map_cam[1][2] * dy + T_map_cam[2][2] * dz
    return x, y, z


def depth_evidence(T_map_cam, T_map_obj, frame: Optional[DepthFrame], cfg: DepthCheckConfig):
    """(verdict, info) for one landmark in one frame."""
    if frame is None:
        return UNKNOWN, {"why": "no depth"}
    x, y, z = _point_in_camera(T_map_cam, T_map_obj)
    if z < cfg.min_depth_m or z > cfg.max_depth_m:
        return UNKNOWN, {"why": "range", "z": z}
    u = frame.fx * x / z + frame.cx
    v = frame.fy * y / z + frame.cy
    h, w = frame.depth_mm.shape[:2]
    if not (cfg.border_px <= u < w - cfg.border_px and cfg.border_px <= v < h - cfg.border_px):
        return UNKNOWN, {"why": "outside", "z": z}
    r = max(1, int(round(frame.fx * cfg.radius_m / z)))
    ui, vi = int(round(u)), int(round(v))
    patch = np.asarray(frame.depth_mm[max(0, vi - r):vi + r + 1, max(0, ui - r):ui + r + 1], dtype=np.float64) / 1000.0
    if patch.size == 0:
        return UNKNOWN, {"why": "empty", "z": z}
    valid = (patch > cfg.min_depth_m) & (patch < cfg.max_depth_m)
    frac_valid = float(valid.mean())
    if frac_valid < cfg.min_valid_fraction:
        return UNKNOWN, {"why": "holes", "z": z, "valid": frac_valid}
    d = patch[valid]
    farther = float(np.mean(d > z + cfg.free_margin_m))
    closer = float(np.mean(d < z - cfg.occluded_margin_m))
    info = {"z": round(z, 3), "measured_median": round(float(np.median(d)), 3), "farther": round(farther, 2),
            "closer": round(closer, 2), "valid": round(frac_valid, 2), "px": (ui, vi), "r": r}
    if farther >= cfg.free_fraction:
        # confirm on the wide window: anything near the landmark's depth (or closer) inside it vetoes FREE
        R = max(r, int(round(frame.fx * cfg.free_radius_m / z)))
        win = np.asarray(frame.depth_mm[max(0, vi - R):vi + R + 1, max(0, ui - R):ui + R + 1], dtype=np.float64) / 1000.0
        wv = win[(win > cfg.min_depth_m) & (win < cfg.max_depth_m)]
        info["R"] = R
        if wv.size < cfg.min_valid_fraction * win.size:
            return UNKNOWN, {**info, "why": "window holes"}
        wfar = float(np.mean(wv > z + cfg.free_margin_m))
        info["window_farther"] = round(wfar, 2)
        if wfar >= cfg.free_window_fraction:
            return FREE, info
        return SURFACE, {**info, "why": "window has near depth"}
    if closer >= cfg.occluded_fraction:
        return OCCLUDED, info
    return SURFACE, info
