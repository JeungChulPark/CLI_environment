"""conv_session.py — converted-bag camera session with the RsSession interface.

A converted session folder holds one rosbag2 sqlite file with standard RealSense topics:
colour (bgr8), depth already aligned to colour (16UC1 mm) and CameraInfo. Frames are colour/depth
pairs matched by header stamp; timestamps are the colour header stamps (colour Global Time in the
260910 datasets) plus `offset_ns`.
"""
from __future__ import annotations

import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lidar"))
from bag_io import COLOR, DEPTH, Bag, parse_image  # noqa: E402

CAMERA_INFO = "/camera/camera/color/camera_info"


def parse_camera_info(blob):
    o = 12
    fl = struct.unpack_from("<I", blob, o)[0]; o += 4 + fl; o = (o + 3) & ~3
    h, w = struct.unpack_from("<II", blob, o); o += 8
    ml = struct.unpack_from("<I", blob, o)[0]; o += 4 + ml; o = (o + 3) & ~3
    nd = struct.unpack_from("<I", blob, o)[0]; o += 4
    align8 = lambda x: 4 + ((x - 4 + 7) & ~7)   # CDR aligns relative to the payload after the 4-byte header
    if nd:
        o = align8(o)
    D = np.frombuffer(blob, "<f8", nd, o).copy(); o += 8 * nd
    o = align8(o)
    K = np.frombuffer(blob, "<f8", 9, o).copy().reshape(3, 3)
    return w, h, K, D


@dataclass
class Frame:
    index: int
    t_ns: int
    color_bgr: np.ndarray
    depth_raw: np.ndarray | None = None


class ConvSession:
    def __init__(self, session_dir: str | Path, offset_ns: int = 0, times_ns=None):
        self.dir = Path(session_dir)
        self.bag = Bag(self.dir)
        ic, tc = self.bag.stamps(COLOR)
        idp, td = self.bag.stamps(DEPTH)
        pos = {int(t): int(i) for i, t in zip(idp, td)}
        keep = [k for k, t in enumerate(tc) if int(t) in pos]
        if len(keep) < len(tc):
            print(f"[conv_session] {len(tc) - len(keep)} colour frames without an exactly stamped depth frame dropped")
        self.color_ids = ic[keep]
        self.depth_ids = np.array([pos[int(tc[k])] for k in keep])
        self.t_ns = tc[keep].astype(np.int64) + int(offset_ns)
        if times_ns is not None:
            self.t_ns = np.asarray(times_ns, np.int64)
        order = np.argsort(self.t_ns)
        self.color_ids, self.depth_ids, self.t_ns = self.color_ids[order], self.depth_ids[order], self.t_ns[order]
        cid = self.bag.topic_id[CAMERA_INFO]
        blob = self.bag.con.execute("select data from messages where topic_id=? limit 1", (cid,)).fetchone()[0]
        self.W, self.H, K, D = parse_camera_info(blob)
        self._K = K
        self.kc = [float(v) for v in (list(D) + [0.0] * 5)[:5]]
        self.offset_ns = int(offset_ns)

    @property
    def K(self):
        return self._K.copy()

    def __len__(self):
        return len(self.t_ns)

    def read(self, i: int, with_depth: bool) -> Frame:
        _, img = parse_image(self.bag.blob(self.color_ids[i]))
        depth = None
        if with_depth:
            _, depth = parse_image(self.bag.blob(self.depth_ids[i]))
        return Frame(i, int(self.t_ns[i]), np.ascontiguousarray(img), depth)

    def align(self, depth_mm):
        """depth in a converted bag is already aligned to the colour grid"""
        return depth_mm
