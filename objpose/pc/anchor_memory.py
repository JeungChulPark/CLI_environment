"""anchor_memory.py — ObjectMemory fed with an explicit camera pose (the object-anchored one) instead of the
keyframe-anchored SLAM pose. Everything else (association, fusion, depth check, rebuild) is ObjectMemory's.
"""
from __future__ import annotations

import numpy as np

from fusion import Fusion, PoseBuffer
from memory import ObjectMemory, SamFrame

KEEP = ("active", "lost", "remembered")


class WorldPoseMemory(ObjectMemory):
    def __init__(self, K, img_size, depth_check=None, **kw):
        super().__init__(Fusion(np.eye(4), PoseBuffer()), K, img_size, depth_check=depth_check, **kw)

    def add_frame_world(self, t_ns: int, T_w_cam, dets: list, depth=None):
        """dets: [(name, score, R, t_mm)] in the camera whose world pose is T_w_cam (None = no pose)."""
        rows = []
        for name, score, R, t_mm in dets:
            T = np.eye(4)
            T[:3, :3] = np.asarray(R, np.float64).reshape(3, 3)
            T[:3, 3] = np.asarray(t_mm, np.float64) / 1000.0
            rows.append((name, float(score), T))
        with self._lock:
            self.frames.append(SamFrame(int(t_ns), None, None if T_w_cam is None else np.asarray(T_w_cam, float), rows, depth))
            self.frames.sort(key=lambda f: f.t_ns)

    def anchor_landmarks(self):
        """[(id, name, T_w_obj, n_obs)] of the kept landmarks, for ObjectAnchor.observe"""
        return [(m["object_id"], m["name"], m["T_w_obj"], m["n_obs"]) for m in self.landmarks() if m["status"] in KEEP]


class CorrectedSlamMemory(ObjectMemory):
    """The keyframe-anchored ObjectMemory (follows SLAM loop closures), with every frame registered at the
    object-anchored camera pose: frame i keeps the correction C_i in effect when it arrived, and its camera is
    C_i . T_w_kf(latest) . T_kf_sam. When SLAM re-optimises its map (loop closure) the corrections of the frames
    before it are dropped — the loop closure has already fixed the drift they compensated, keeping them would
    correct it twice — and the memory is rebuilt."""

    def __init__(self, fusion, K, img_size, depth_check=None, **kw):
        super().__init__(fusion, K, img_size, depth_check=depth_check, **kw)
        self.corr = {}          # frame t_ns -> (C, epoch)
        self.epoch = 0

    def add_frame(self, t_ns: int, dets: list, depth=None, C=None):
        if C is not None:
            with self._lock:
                self.corr[int(t_ns)] = (np.asarray(C, float).copy(), self.epoch)
        super().add_frame(t_ns, dets, depth)

    def new_epoch(self):
        """call when SLAM re-optimises its map: older corrections no longer apply; forces a rebuild"""
        with self._lock:
            self.epoch += 1
            self._kf_version_seen = -1

    def _camera(self, f: SamFrame):
        T = super()._camera(f)
        c = self.corr.get(int(f.t_ns))
        if T is None or c is None or c[1] != self.epoch:
            return T
        return c[0] @ T
