#!/usr/bin/env python3
"""vlp16_check.py — compare vlp16_decode against the ROS driver's /velodyne_points recorded in the same bag.
    python vlp16_check.py <lidar dir with *.db3> [--scans 20]
"""
import argparse, glob, sqlite3, struct, sys
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
sys.path.insert(0, str(Path(__file__).resolve().parent))
from vlp16_decode import decode_scan

def packets_of(blob):
    o = 4 + 8; L = struct.unpack_from("<I", blob, o)[0]; o += 4 + L; o = (o + 3) & ~3
    n = struct.unpack_from("<I", blob, o)[0]; o += 4; out = []
    for _ in range(n):
        o = (o + 3) & ~3; o += 8; out.append(blob[o:o + 1206]); o += 1206
    return out

def cloud(m):
    dt = np.dtype({"names": [f.name for f in m.fields], "formats": ["<f4" if f.datatype == 7 else "<u2" for f in m.fields],
                   "offsets": [f.offset for f in m.fields], "itemsize": m.point_step})
    return np.frombuffer(bytes(m.data), dt).reshape(m.height, m.width)

ap = argparse.ArgumentParser(); ap.add_argument("dir"); ap.add_argument("--scans", type=int, default=20); a = ap.parse_args()
c = sqlite3.connect(glob.glob(a.dir + "/*.db3")[0]); tid = {n: i for i, n, _ in c.execute("select id,name,type from topics")}
P = c.execute("select data from messages where topic_id=? order by timestamp limit ?", (tid["/velodyne_packets"], a.scans)).fetchall()
C = c.execute("select data from messages where topic_id=? order by timestamp limit ?", (tid["/velodyne_points"], a.scans)).fetchall()
err, nboth, nref, nme, telt = [], 0, 0, 0, []
for (pb,), (cb,) in zip(P, C):
    ref = cloud(deserialize_message(cb, PointCloud2)); me = decode_scan(packets_of(pb))
    if ref.shape != me["x"].shape: print("shape", ref.shape, me["x"].shape); continue
    rv = ~np.isnan(ref["x"]); mv = ~np.isnan(me["x"])
    both = rv & mv; nboth += both.sum(); nref += rv.sum(); nme += mv.sum()
    d = np.sqrt((ref["x"] - me["x"]) ** 2 + (ref["y"] - me["y"]) ** 2 + (ref["z"] - me["z"]) ** 2)[both]
    err.append(d); telt.append(np.abs(ref["time"] - me["time"])[both])
e = np.concatenate(err); t = np.concatenate(telt)
print(f"points: driver {nref}, mine {nme}, both {nboth}")
print(f"xyz difference: median {np.median(e)*1000:.2f} mm  p99 {np.percentile(e,99)*1000:.2f} mm  max {e.max()*1000:.1f} mm")
print(f"time difference: median {np.median(t)*1e6:.1f} us  max {t.max()*1e6:.1f} us")
