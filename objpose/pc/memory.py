"""memory.py — object memory on top of keyframe-anchored SAM-6D observations.

Reuses objectmemory_ws's StreamingObjectMemory unchanged (association gates on class,
3D distance and rotation with symmetry, tentative -> active promotion, existence filter
with a field-of-view detection model, SE(3) pose fusion, multiple instances per class).
That memory works in world coordinates, while this system keeps everything relative to
ORB-SLAM3 keyframes so loop closures can move it. The adapter bridges the two:

  * every SAM-6D processed frame is kept: its SAM-camera pose anchored to a keyframe
    (T_kf_sam) and its detections in the SAM camera (T_sam_obj), which never change;
  * frames are fed to the memory one by one as they arrive;
  * when keyframe poses change (a `kf_update`), the memory is rebuilt from scratch by
    replaying every kept frame with its corrected camera pose. A run holds a few hundred
    processed frames, so a rebuild costs milliseconds.
"""
from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "objectmemory_ws" / "object_memory" / "src"))
from core.models import Sam6DDetection, SlamCameraPose, TrackingStatus  # noqa: E402
from pipeline.object_memory_runner import StreamingObjectMemory  # noqa: E402
from core.depth_check import DepthCheckConfig, DepthFrame  # noqa: E402

from fusion import Fusion, inv_se3  # noqa: E402

import os
LOOP_W_D0_M = float(os.environ.get("OBJPOSE_LOOP_W_D0_M", "0"))   # 0 (default) = off; 0.05 = the §3.3 weighting — did NOT help (see README 2026-09-14 F1/F2/가중치 절)


MAP_OCCL = os.environ.get("OBJPOSE_MAP_OCCL", "0") == "1"   # 1 = map-based occlusion (no miss penalty behind mapped objects)
MODEL_POINTS = REPO / "sam6d_realtime" / "assets" / "model_points"


def map_occlusion_kwargs(names) -> dict:
    """ObjectMemory kwargs for map-based occlusion (CAD box of each object) when OBJPOSE_MAP_OCCL=1, else {}."""
    if not MAP_OCCL:
        return {}
    from core.map_occlusion import box_corners
    ext = {n: box_corners(np.load(MODEL_POINTS / f"{n}.npy") / 1000.0) for n in names if (MODEL_POINTS / f"{n}.npy").exists()}
    return {"occlusion_extents": ext}


def to_tuple(T) -> tuple:
    return tuple(tuple(float(v) for v in row) for row in np.asarray(T))


@dataclass
class SamFrame:
    t_ns: int
    kf: int | None
    T_anchor_sam: np.ndarray | None    # T_kf_sam, or T_w_sam when kf is None; None = no SLAM pose
    dets: list                         # (name, score, T_sam_obj 4x4)
    depth: object = None               # downsampled aligned depth (uint16 mm) of the frame, for the depth check


class ObjectMemory:
    def __init__(self, fusion: Fusion, K, img_size, depth_check: DepthCheckConfig | None = None, **mem_kwargs):
        self.fusion = fusion
        self.depth_check = depth_check
        self.K = [list(map(float, r)) for r in np.asarray(K)]
        self.img_size = tuple(img_size)
        self.mem_kwargs = mem_kwargs
        self.frames: list[SamFrame] = []
        self._lock = threading.Lock()
        self._mem = self._new()
        self._fed = 0
        self._kf_version_seen = -1
        self.rebuilds = 0
        self.last_rebuild_ms = 0.0

    def _new(self):
        return StreamingObjectMemory(cam_K=self.K, img_size=self.img_size, depth_check=self.depth_check, **self.mem_kwargs)

    # ── input ────────────────────────────────────────────────────────────────
    def add_frame(self, t_ns: int, dets: list, depth=None):
        """dets: [(name, score, R 3x3, t_mm 3)] from SAM-6D for the frame captured at t_ns; depth: the frame's aligned
        depth downsampled (e.g. [::8, ::8], uint16 mm) for the depth check, or None."""
        poses = self.fusion.poses
        T_w_sam, _ = self.fusion.T_w_sam(t_ns)
        kf = poses.anchor_at(self.fusion.slam_t(t_ns)) if T_w_sam is not None else None
        anchor = None
        if T_w_sam is not None:
            T_w_kf, _ = poses.kfs.resolve(kf) if kf is not None else (None, None)
            if T_w_kf is None:
                kf, anchor = None, T_w_sam
            else:
                anchor = inv_se3(T_w_kf) @ T_w_sam
        rows = []
        for name, score, R, t_mm in dets:
            T = np.eye(4)
            T[:3, :3] = np.asarray(R, np.float64).reshape(3, 3)
            T[:3, 3] = np.asarray(t_mm, np.float64) / 1000.0
            rows.append((name, float(score), T))
        with self._lock:
            self.frames.append(SamFrame(t_ns, kf, anchor, rows, depth))
            self.frames.sort(key=lambda f: f.t_ns)

    # ── replay ───────────────────────────────────────────────────────────────
    def _camera(self, f: SamFrame):
        if f.T_anchor_sam is None:
            return None
        if f.kf is None:
            return f.T_anchor_sam
        T_w_kf, _ = self.fusion.poses.kfs.resolve(f.kf)
        return None if T_w_kf is None else T_w_kf @ f.T_anchor_sam

    def _weight(self, f: SamFrame) -> float:
        """Pose-fusion weight of a frame: w = 1 / (1 + (d / d0)^2), d = cumulative correction the anchor keyframe
        has received (docs/orb_improvement_plan_260914.md §3.3). Observations made before a loop closure sit on a
        drifted map; after the closure they still miss the corrected observations by several cm, so they get
        less say in the running average. d0 = LOOP_W_D0_M (0 disables); LiDAR corrections (<3 cm) stay near 1."""
        if LOOP_W_D0_M <= 0 or f.kf is None:
            return 1.0
        d = self.fusion.poses.kfs.correction(f.kf)
        return 1.0 / (1.0 + (d / LOOP_W_D0_M) ** 2)

    def _step(self, mem, f: SamFrame, idx: int):
        stamp = f.t_ns / 1e9
        w = self._weight(f)
        dets = [Sam6DDetection(stamp=stamp, frame_id="sam_camera", detection_id=idx * 100 + j,
                               object_name=name, T_cam_obj=to_tuple(T), score=score, weight=w)
                for j, (name, score, T) in enumerate(f.dets)]
        T_w_sam = self._camera(f)
        if T_w_sam is None:
            return mem.step_no_pose(dets, stamp, idx)
        pose = SlamCameraPose(stamp=stamp, frame_id="map", child_frame_id="sam_camera",
                              T_map_cam=to_tuple(T_w_sam), tracking_status=TrackingStatus.OK,
                              source_slam_id="orbslam3")
        depth = None
        if self.depth_check is not None and f.depth is not None:
            depth = DepthFrame.from_full(f.depth, self.K, self.img_size)
        return mem.step(dets, pose, stamp, idx, depth=depth)

    def update(self):
        """Feed new frames; rebuild everything if keyframe poses changed since last time.

        A rebuild runs on a private memory and is swapped in at the end, so readers never wait
        for it."""
        kfs = self.fusion.poses.kfs
        version = kfs.updates
        with self._lock:
            frames = list(self.frames)
            rebuild = version != self._kf_version_seen
            if not rebuild:
                for i in range(self._fed, len(frames)):
                    self._step(self._mem, frames[i], i)
                self._fed = len(frames)
                return
        t0 = time.perf_counter()
        mem = self._new()
        for i, f in enumerate(frames):
            self._step(mem, f, i)
        with self._lock:
            self._mem = mem
            self._fed = len(frames)
            self._kf_version_seen = version
            self.rebuilds += 1
            self.last_rebuild_ms = (time.perf_counter() - t0) * 1e3

    # ── output ───────────────────────────────────────────────────────────────
    def landmarks(self):
        with self._lock:
            out = []
            for lm in self._mem.store.all_landmarks():
                out.append({
                    "object_id": lm.object_id, "name": lm.object_name,
                    "status": getattr(lm.status, "value", str(lm.status)),
                    "T_w_obj": np.asarray(lm.T_map_obj_smoothed or lm.T_map_obj, np.float64),
                    "confidence": float(lm.confidence), "n_obs": int(lm.observation_count),
                    "last_seen_ns": int(round(lm.last_seen_time * 1e9)),
                    "first_seen_ns": int(round(lm.first_seen_time * 1e9)),
                    "score": lm.last_sam6d_score,
                })
            return out

    def stats(self):
        lms = self.landmarks()
        by_status = {}
        for lm in lms:
            by_status[lm["status"]] = by_status.get(lm["status"], 0) + 1
        out = {"frames": len(self.frames), "landmarks": len(lms), "by_status": by_status,
               "rebuilds": self.rebuilds, "last_rebuild_ms": round(self.last_rebuild_ms, 1)}
        if self.depth_check is not None:
            with self._lock:
                dl = self._mem.depth_log
                out["depth_check"] = {"frames_checked": dl["frames_checked"], "frames_skipped_speed": dl["frames_skipped_speed"],
                                      "verdicts": dict(dl["verdicts"]), "why": dict(dl.get("unknown_why", {})), "deleted": len(dl["deleted"]),
                                      "frames_with_depth": sum(1 for f in self.frames if f.depth is not None)}
        return out

    def depth_samples(self):
        """first per-landmark depth verdicts with their numbers (for tuning)"""
        with self._lock:
            return list(self._mem.depth_log.get("samples", [])) if self.depth_check is not None and self._mem is not None else []

    def depth_deletions(self):
        """landmarks the depth check deleted (in the current memory build)"""
        with self._lock:
            return list(self._mem.depth_log["deleted"]) if self.depth_check is not None else []
