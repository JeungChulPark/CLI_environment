"""bop_session.py — BOP scene folder (YCB-V, LM, LM-O, HOPE) with the RsSession interface.

A BOP test scene is a folder of images rather than a rosbag, so it carries no clock: the
frames are a fixed-rate video (YCB-V was recorded at 30 fps). Stamps are therefore
synthesised as `t0_ns + index / fps`, and the pose stream for the same scene must be built
on that same clock (tools/bop_to_tum.py does it from the same scene_camera.json).

Depth is stored in `depth_scale` units (0.1 mm on YCB-V), while the rest of this pipeline
assumes millimetres, so `read()` converts. Intrinsics come from scene_camera.json; a BOP
scene's images are already undistorted, so `kc` is zero.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

DEFAULT_FPS = 30.0
DEFAULT_T0_NS = 1_600_000_000_000_000_000      # arbitrary but fixed, so reruns share a clock


@dataclass
class Frame:
    index: int
    t_ns: int
    color_bgr: np.ndarray
    depth_raw: np.ndarray | None = None


class BopSession:
    def __init__(self, session_dir, offset_ns: int = 0, fps: float = DEFAULT_FPS,
                 t0_ns: int = DEFAULT_T0_NS, times_ns=None, cache_dir=None):
        self.dir = Path(session_dir)
        cam = json.loads((self.dir / "scene_camera.json").read_text())
        self.ids = sorted(cam, key=int)
        Ks = {tuple(cam[i]["cam_K"]) for i in self.ids}
        if len(Ks) != 1:
            raise ValueError(f"{self.dir}: {len(Ks)} distinct intrinsics in one scene")
        self._K = np.asarray(cam[self.ids[0]]["cam_K"], np.float64).reshape(3, 3)
        self.depth_scale = float(cam[self.ids[0]].get("depth_scale", 1.0))
        self.rgb = [self._pick("rgb", i) for i in self.ids]
        self.depth = [self._pick("depth", i) for i in self.ids]
        img = cv2.imread(str(self.rgb[0]))
        if img is None:
            raise ValueError(f"{self.dir}: cannot read {self.rgb[0]}")
        self.H, self.W = img.shape[:2]
        self.fps = float(fps)
        self.offset_ns = int(offset_ns)
        if times_ns is not None:
            self.t_ns = np.asarray(times_ns, np.int64)
        else:
            self.t_ns = (int(t0_ns) + np.arange(len(self.ids), dtype=np.int64)
                         * int(round(1e9 / self.fps))) + self.offset_ns
        self.kc = [0.0] * 5                    # BOP images are undistorted
        self.scene_camera = cam

    def _pick(self, sub: str, frame_id: str) -> Path:
        d = self.dir / sub
        for ext in (".png", ".jpg"):
            p = d / f"{int(frame_id):06d}{ext}"
            if p.exists():
                return p
        raise FileNotFoundError(f"{d}/{int(frame_id):06d}.[png|jpg]")

    @property
    def K(self):
        return self._K.copy()

    def __len__(self):
        return len(self.ids)

    def read(self, i: int, with_depth: bool) -> Frame:
        img = cv2.imread(str(self.rgb[i]), cv2.IMREAD_COLOR)
        depth = None
        if with_depth:
            raw = cv2.imread(str(self.depth[i]), cv2.IMREAD_UNCHANGED)
            # BOP stores depth in `depth_scale` units; downstream code assumes uint16 millimetres
            depth = np.rint(raw.astype(np.float32) * self.depth_scale).astype(np.uint16)
        return Frame(i, int(self.t_ns[i]), np.ascontiguousarray(img), depth)

    def align(self, depth_mm):
        """BOP depth is already registered to the colour frame"""
        return depth_mm
