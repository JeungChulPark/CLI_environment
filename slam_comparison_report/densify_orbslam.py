#!/usr/bin/env python3
"""Pattern A (offline realization): rebuild ORB-SLAM3 dense map from the
OPTIMIZED keyframe trajectory + bag RGB-D, so it is loop-closure-consistent.

The orbslam3_ros2 node accumulates its dense cloud from LIVE per-frame poses
(pre-BA), so late loop-closure corrections aren't reflected (outer "spray"
noise). Here we reuse ORB-SLAM3's saved OPTIMIZED keyframe poses (keyframes.txt)
and reproject each keyframe's full-res RGB-D from the bag into world, then voxel
downsample. No node C++ change, no SLAM re-run, no Open3D — numpy only.

Run in `conda activate rtabmap` (rclpy + sensor_msgs + numpy).
"""
from __future__ import annotations

import sqlite3
import sys
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

ROOT = Path(__file__).resolve().parents[1]
COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
# RealSense D455 640x480 color intrinsics (matches orbslam3_d455f_640x480_rgbd.yaml)
FX, FY, CX, CY = 386.918, 386.347, 322.784, 250.280
DEPTH_SCALE = 1000.0
MIN_D, MAX_D = 0.2, 4.0
PIX_STRIDE = 2
VOXEL = 0.015

DATASETS = ["SLAM_forward_backward_repeat", "SLAM_one_lap",
            "SLAM_one_lap_back_and_forth", "SLAM_three_laps"]


def quat_to_R(qx, qy, qz, qw):
    n = (qx*qx + qy*qy + qz*qz + qw*qw) ** 0.5
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
    ], dtype=np.float64)


def load_keyframes(path: Path):
    kfs = []
    for ln in path.read_text(errors="replace").splitlines():
        p = ln.split()
        if len(p) >= 8:
            try:
                kfs.append([float(x) for x in p[:8]])  # ts x y z qx qy qz qw
            except ValueError:
                pass
    return kfs


def msg_index(con, topic_id):
    """Return (ts_ns array, rowid array) sorted by bag timestamp (no blob decode)."""
    rows = con.execute(
        "select rowid, timestamp from messages where topic_id=? order by timestamp", (topic_id,)
    ).fetchall()
    ts = np.array([r[1] for r in rows], dtype=np.int64)
    rid = np.array([r[0] for r in rows], dtype=np.int64)
    return ts, rid


def fetch_image(con, rowid):
    blob = con.execute("select data from messages where rowid=?", (rowid,)).fetchone()[0]
    return deserialize_message(bytes(blob), Image)


def nearest(ts_arr, target_ns):
    i = int(np.searchsorted(ts_arr, target_ns))
    if i <= 0:
        return 0
    if i >= len(ts_arr):
        return len(ts_arr) - 1
    return i if abs(ts_arr[i] - target_ns) < abs(ts_arr[i-1] - target_ns) else i - 1


def voxel_dedup(pts, cols, voxel):
    if len(pts) == 0:
        return pts, cols
    keys = np.floor(pts / voxel).astype(np.int64)
    # 1D hash of 3D voxel index
    h = (keys[:, 0] * 73856093) ^ (keys[:, 1] * 19349663) ^ (keys[:, 2] * 83492791)
    _, idx = np.unique(h, return_index=True)
    return pts[idx], cols[idx]


def densify(ds: str) -> dict:
    import os
    out_dir = ROOT / "orbslam_ws" / "output" / os.environ.get("ORB_OUTPUT_SUBDIR", "") / ds
    kfs = load_keyframes(out_dir / "keyframes.txt")
    db = ROOT / "data_slam" / ds / "bag_0.db3"
    con = sqlite3.connect(str(db))
    ctid = con.execute("select id from topics where name=?", (COLOR_TOPIC,)).fetchone()[0]
    dtid = con.execute("select id from topics where name=?", (DEPTH_TOPIC,)).fetchone()[0]
    cts, crid = msg_index(con, ctid)
    dts, drid = msg_index(con, dtid)

    # pixel grid (strided)
    us = np.arange(0, 640, PIX_STRIDE)
    vs = np.arange(0, 480, PIX_STRIDE)
    uu, vv = np.meshgrid(us, vs)
    uu = uu.ravel(); vv = vv.ravel()

    all_pts, all_cols = [], []
    for kf in kfs:
        ts = kf[0]
        tns = int(ts * 1e9)
        ci = nearest(cts, tns); di = nearest(dts, tns)
        cm = fetch_image(con, int(crid[ci]))
        dm = fetch_image(con, int(drid[di]))
        rgb = np.frombuffer(bytes(cm.data), np.uint8).reshape(cm.height, cm.width, 3)
        depth = np.frombuffer(bytes(dm.data), np.uint16).reshape(dm.height, dm.width)
        z = depth[vv, uu].astype(np.float64) / DEPTH_SCALE
        m = (z > MIN_D) & (z < MAX_D)
        if not m.any():
            continue
        zu, uuu, vvv = z[m], uu[m], vv[m]
        x = (uuu - CX) * zu / FX
        y = (vvv - CY) * zu / FY
        pc = np.stack([x, y, zu], axis=1)             # camera frame (Nx3)
        R = quat_to_R(kf[4], kf[5], kf[6], kf[7])
        t = np.array([kf[1], kf[2], kf[3]])
        pw = pc @ R.T + t                              # world frame
        col = rgb[vvv, uuu]                            # rgb8
        all_pts.append(pw.astype(np.float32))
        all_cols.append(col.astype(np.uint8))
    con.close()
    if not all_pts:
        return {"ds": ds, "raw": 0, "dense": 0}
    pts = np.concatenate(all_pts); cols = np.concatenate(all_cols)
    raw = len(pts)
    pts, cols = voxel_dedup(pts, cols, VOXEL)
    write_pcd(out_dir / "orbslam3_dense_kf.pcd", pts, cols)
    return {"ds": ds, "kfs": len(kfs), "raw": raw, "dense": len(pts),
            "out": str(out_dir / "orbslam3_dense_kf.pcd")}


def write_pcd(path: Path, pts, cols):
    rgb = (cols[:, 0].astype(np.uint32) << 16) | (cols[:, 1].astype(np.uint32) << 8) | cols[:, 2].astype(np.uint32)
    n = len(pts)
    with path.open("w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n")
        f.write("FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n")
        f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n")
        buf = np.column_stack([pts, rgb.astype(np.float64)])
        np.savetxt(f, buf, fmt="%.5f %.5f %.5f %d")


def main() -> int:
    targets = sys.argv[1:] or DATASETS
    for ds in targets:
        r = densify(ds)
        print(f"{ds:30s} kfs={r.get('kfs','?')} raw={r['raw']:,} -> dense(voxel {VOXEL}m)={r['dense']:,}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
