"""bag_io.py — read converted rosbag2 (sqlite3, CDR) topics without ROS.

Standard-topic camera bags (sensor_msgs/Image bgr8 / 16UC1, already depth-aligned) and the
Velodyne bag (sensor_msgs/PointCloud2 with per-point time).
"""
from __future__ import annotations

import sqlite3
import struct
from pathlib import Path

import numpy as np

COLOR = "/camera/camera/color/image_raw"
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"
POINTS = "/velodyne_points"
_DT = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def _header(blob, o=4):
    sec, nsec = struct.unpack_from("<iI", blob, o); o += 8
    fl = struct.unpack_from("<I", blob, o)[0]; o += 4 + fl; o = (o + 3) & ~3
    return sec * 1_000_000_000 + nsec, o


def parse_image(blob):
    t, o = _header(blob)
    h, w = struct.unpack_from("<II", blob, o); o += 8
    el = struct.unpack_from("<I", blob, o)[0]; o += 4
    enc = blob[o:o + el - 1].decode(); o += el
    o += 1; o = (o + 3) & ~3
    step = struct.unpack_from("<I", blob, o)[0]; o += 4
    n = struct.unpack_from("<I", blob, o)[0]; o += 4
    data = np.frombuffer(blob, np.uint8, n, o)
    if enc in ("16UC1", "mono16"):
        img = data.reshape(h, step)[:, :w * 2].copy().view("<u2").reshape(h, w)
    else:
        img = data.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        if enc == "rgb8":
            img = img[:, :, ::-1]
    return t, img


def parse_pc2(blob):
    t, o = _header(blob)
    o += 8
    nf = struct.unpack_from("<I", blob, o)[0]; o += 4
    names, offs, fmts = [], [], []
    for _ in range(nf):
        o = (o + 3) & ~3
        sl = struct.unpack_from("<I", blob, o)[0]; o += 4
        names.append(blob[o:o + sl - 1].decode()); o += sl; o = (o + 3) & ~3
        offs.append(struct.unpack_from("<I", blob, o)[0]); o += 4
        fmts.append("<" + _DT[blob[o]]); o += 1; o = (o + 3) & ~3
        o += 4
    o += 1; o = (o + 3) & ~3
    step = struct.unpack_from("<I", blob, o)[0]; o += 8
    dl = struct.unpack_from("<I", blob, o)[0]; o += 4
    dtype = np.dtype({"names": names, "formats": fmts, "offsets": offs, "itemsize": step})
    return t, np.frombuffer(blob[o:o + dl], dtype)


class Bag:
    def __init__(self, folder_or_db: str | Path):
        p = Path(folder_or_db)
        db = p if p.suffix == ".db3" else next(p.glob("*.db3"))
        self.con = sqlite3.connect(f"file:{db}?mode=ro", uri=True, check_same_thread=False)
        self.topic_id = {n: i for i, n in self.con.execute("select id, name from topics")}

    def stamps(self, topic):
        """(message ids, header stamps) in bag order; header stamps are parsed from each message."""
        # only the first bytes are needed for the header: never load whole images here
        rows = self.con.execute("select id, substr(data, 1, 64) from messages where topic_id=? order by timestamp",
                                (self.topic_id[topic],)).fetchall()
        return np.array([r[0] for r in rows]), np.array([_header(r[1])[0] for r in rows], np.int64)

    def blob(self, msg_id):
        return self.con.execute("select data from messages where id=?", (int(msg_id),)).fetchone()[0]
