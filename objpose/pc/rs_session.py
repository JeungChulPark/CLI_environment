"""rs_session.py — read a RAW RealSense SDK session folder (no ROS).

A session folder holds one `*.db3` (/device_0/... topics, depth NOT aligned to color)
plus `rgbd_timestamp_associations.json`. Frames are served in association order with
host-epoch timestamps shifted by `offset_ns` onto the reference (SLAM host) clock.

The depth->color alignment is a line-for-line port of `align()` in
sam6d_realtime/data/convert_recording.py, the converter that produced the SAM-6D bag
this project already validated; the Mac-side SLAM reader ports the same function.
"""
from __future__ import annotations

import json
import sqlite3
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

COLOR_INFO = "/device_0/sensor_1/Color_0/camera_info"
DEPTH_INFO = "/device_0/sensor_0/Depth_0/camera_info"
TF_REF = "/device_0/sensor_1/Color_0/tf/ref_0"
DEPTH_UNITS = "/device_0/sensor_0/option/Depth_Units/value"


def _str_msg(blob: bytes) -> str:
    n = struct.unpack_from("<I", blob, 4)[0]
    return blob[8:8 + n - 1].decode("utf-8", "replace")


def _kv(s: str) -> dict:
    return {k.strip(): v.strip() for k, v in (p.split("=", 1) for p in s.split(";") if "=" in p)}


def _image(blob: bytes):
    off = 4
    off += 8
    flen = struct.unpack_from("<I", blob, off)[0]; off += 4 + flen; off = (off + 3) & ~3
    h, w = struct.unpack_from("<II", blob, off); off += 8
    elen = struct.unpack_from("<I", blob, off)[0]; off += 4
    enc = blob[off:off + elen - 1].decode(); off += elen
    off += 1; off = (off + 3) & ~3
    off += 4
    dlen = struct.unpack_from("<I", blob, off)[0]; off += 4
    return w, h, enc, memoryview(blob)[off:off + dlen]


@dataclass
class Frame:
    index: int
    t_ns: int            # reference-clock timestamp
    color_bgr: np.ndarray
    depth_raw: np.ndarray | None = None


class RsSession:
    def __init__(self, session_dir: str | Path, offset_ns: int = 0, times_ns: np.ndarray | None = None):
        self.dir = Path(session_dir)
        dbs = sorted(self.dir.glob("*.db3"))
        if len(dbs) != 1:
            raise FileNotFoundError(f"expected exactly one .db3 in {self.dir}, found {len(dbs)}")
        self.db_path = dbs[0]
        assoc = json.loads((self.dir / "rgbd_timestamp_associations.json").read_text())["associations"]
        self.color_ids = np.array([a["color_message_id"] for a in assoc], np.int64)
        self.depth_ids = np.array([a["depth_message_id"] for a in assoc], np.int64)
        self.t_ns = np.array([int(a["associated_host_epoch_timestamp_ns"]) for a in assoc], np.int64) + int(offset_ns)
        if times_ns is not None:            # corrected per-frame clock (see clock.py)
            if len(times_ns) != len(assoc):
                raise ValueError("times_ns length does not match the associations")
            self.t_ns = np.asarray(times_ns, np.int64)
        self.offset_ns = int(offset_ns)
        self._con = sqlite3.connect(f"file:{self.db_path}?mode=ro", uri=True, check_same_thread=False)

        cci, dci = _kv(self._one(COLOR_INFO)), _kv(self._one(DEPTH_INFO))
        self.W, self.H = int(cci["width"]), int(cci["height"])
        self.fxc, self.fyc = float(cci["fx"]), float(cci["fy"])
        self.ppxc, self.ppyc = float(cci["ppx"]), float(cci["ppy"])
        self.kc = [float(x) for x in cci["coeffs"].split(",")]
        fxd, fyd, ppxd, ppyd = float(dci["fx"]), float(dci["fy"]), float(dci["ppx"]), float(dci["ppy"])
        rot = trans = None
        for p in self._one(TF_REF).split(";"):
            if p.startswith("rotation="):
                rot = [float(x) for x in p[len("rotation="):].split(",")]
            elif p.startswith("translation="):
                trans = [float(x) for x in p[len("translation="):].split(",")]
        self.R_d2c = np.array(rot, np.float64).reshape(3, 3)
        self.t_d2c = np.array(trans, np.float64)
        self.depth_units = float(self._one(DEPTH_UNITS))
        uu, vv = np.meshgrid(np.arange(self.W), np.arange(self.H))
        self._xnd = (uu - ppxd) / fxd
        self._ynd = (vv - ppyd) / fyd

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fxc, 0, self.ppxc], [0, self.fyc, self.ppyc], [0, 0, 1]], np.float64)

    def __len__(self):
        return len(self.t_ns)

    def _one(self, topic: str) -> str:
        row = self._con.execute(
            "SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name=? LIMIT 1",
            (topic,)).fetchone()
        if row is None:
            raise KeyError(f"topic missing in {self.db_path.name}: {topic}")
        return _str_msg(row[0])

    def _blob(self, msg_id: int) -> bytes:
        return self._con.execute("SELECT data FROM messages WHERE id=?", (int(msg_id),)).fetchone()[0]

    def read(self, i: int, with_depth: bool) -> Frame:
        w, h, enc, data = _image(self._blob(self.color_ids[i]))
        img = np.frombuffer(data, np.uint8).reshape(h, w, 3)
        if enc == "rgb8":
            img = img[:, :, ::-1]
        elif enc != "bgr8":
            raise ValueError(f"unexpected color encoding {enc}")
        depth = None
        if with_depth:
            dw, dh, denc, ddata = _image(self._blob(self.depth_ids[i]))
            if denc not in ("mono16", "16UC1") or (dw, dh) != (self.W, self.H):
                raise ValueError(f"unexpected depth {denc} {dw}x{dh}")
            depth = np.frombuffer(ddata, np.uint16).reshape(dh, dw)
        return Frame(i, int(self.t_ns[i]), np.ascontiguousarray(img), depth)

    def align(self, depth_mm: np.ndarray) -> np.ndarray:
        """Depth (depth-sensor frame) -> uint16 mm on the color pixel grid, nearest wins."""
        R, t = self.R_d2c, self.t_d2c
        Z = depth_mm.astype(np.float64) * self.depth_units
        valid = Z > 0
        X = self._xnd * Z; Y = self._ynd * Z
        Xc = R[0, 0] * X + R[0, 1] * Y + R[0, 2] * Z + t[0]
        Yc = R[1, 0] * X + R[1, 1] * Y + R[1, 2] * Z + t[1]
        Zc = R[2, 0] * X + R[2, 1] * Y + R[2, 2] * Z + t[2]
        valid &= Zc > 0
        k1, k2, p1, p2, k3 = self.kc
        with np.errstate(divide="ignore", invalid="ignore"):
            x = Xc / Zc; y = Yc / Zc
            r2 = x * x + y * y
            f = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
            xd = x * f + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
            yd = y * f + 2 * p2 * x * y + p1 * (r2 + 2 * y * y)
            iu = np.round(self.fxc * xd + self.ppxc).astype(np.int32)
            iv = np.round(self.fyc * yd + self.ppyc).astype(np.int32)
        inb = valid & (iu >= 0) & (iu < self.W) & (iv >= 0) & (iv < self.H)
        flat = (iv[inb] * self.W + iu[inb]).astype(np.int64)
        zmm = np.round(Zc[inb] * 1000.0).astype(np.uint16)
        out = np.zeros(self.W * self.H, np.uint16)
        order = np.argsort(-zmm.astype(np.int64))
        out[flat[order]] = zmm[order]
        return out.reshape(self.H, self.W)
