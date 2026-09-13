#!/usr/bin/env python3
"""lidar_feasibility.py — does LiDAR-only odometry work on this recording?

Reads the Velodyne VLP-16 bag (sensor_msgs/PointCloud2 with per-point `time`) straight from
sqlite (no ROS), runs KISS-ICP with motion deskewing, and reports speed, trajectory sanity
and a top-view map rendered from the estimated poses (a blurred map means the poses are
wrong).

    PYTHONPATH=<kiss-icp> python objpose/lidar/lidar_feasibility.py <dataset dir> --out <dir>
"""
from __future__ import annotations

import argparse
import json
import sqlite3
import struct
import time
from pathlib import Path

import numpy as np

DT = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def parse_pc2(blob: bytes):
    o = 4
    sec, nsec = struct.unpack_from("<iI", blob, o); o += 8
    fl = struct.unpack_from("<I", blob, o)[0]; o += 4 + fl; o = (o + 3) & ~3
    o += 8
    nf = struct.unpack_from("<I", blob, o)[0]; o += 4
    names, offs, fmts = [], [], []
    for _ in range(nf):
        o = (o + 3) & ~3
        sl = struct.unpack_from("<I", blob, o)[0]; o += 4
        names.append(blob[o:o + sl - 1].decode()); o += sl; o = (o + 3) & ~3
        offs.append(struct.unpack_from("<I", blob, o)[0]); o += 4
        fmts.append("<" + DT[blob[o]]); o += 1; o = (o + 3) & ~3
        o += 4
    o += 1; o = (o + 3) & ~3
    step = struct.unpack_from("<I", blob, o)[0]; o += 8
    dl = struct.unpack_from("<I", blob, o)[0]; o += 4
    dtype = np.dtype({"names": names, "formats": fmts, "offsets": offs, "itemsize": step})
    return sec * 1_000_000_000 + nsec, np.frombuffer(blob[o:o + dl], dtype)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dataset")
    ap.add_argument("--out", required=True)
    ap.add_argument("--min-range", type=float, default=0.5)
    ap.add_argument("--max-range", type=float, default=30.0)
    ap.add_argument("--voxel", type=float, default=0.10)
    ap.add_argument("--no-deskew", action="store_true")
    a = ap.parse_args()

    from kiss_icp.config import load_config
    from kiss_icp.kiss_icp import KissICP

    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    db = next((Path(a.dataset) / "lidar").glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    tid = con.execute("select id from topics where name='/velodyne_points'").fetchone()[0]

    cfg = load_config(None)
    cfg.data.min_range = a.min_range
    cfg.data.max_range = a.max_range
    cfg.data.deskew = not a.no_deskew
    cfg.mapping.voxel_size = a.voxel
    odom = KissICP(cfg)

    stamps, poses, times = [], [], []
    world_pts = []
    for k, (blob,) in enumerate(con.execute("select data from messages where topic_id=? order by timestamp", (tid,))):
        t_ns, pts = parse_pc2(blob)
        xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
        r = np.linalg.norm(xyz, axis=1)
        ok = np.isfinite(r) & (r > 0.1)
        xyz = xyz[ok]
        ts = pts["time"][ok].astype(np.float64)
        ts = (ts - ts.min()) / max(ts.max() - ts.min(), 1e-9)          # 0..1 across the sweep
        t0 = time.perf_counter()
        odom.register_frame(xyz, ts)
        times.append((time.perf_counter() - t0) * 1e3)
        T = odom.last_pose.copy()
        stamps.append(t_ns)
        poses.append(T)
        if k % 5 == 0:
            sub = xyz[::8]
            world_pts.append((T[:3, :3] @ sub.T).T + T[:3, 3])
    P = np.array(poses)
    pos = P[:, :3, 3]
    seg = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    dt = np.diff(np.array(stamps)) / 1e9
    speed = seg / np.maximum(dt, 1e-6)
    c = pos - pos.mean(0)
    _, sv, Vt = np.linalg.svd(c, full_matrices=False)
    planar_rms = float((c @ Vt[2]).std())
    yaw_total = 0.0
    for i in range(1, len(P)):
        R = P[i - 1, :3, :3].T @ P[i, :3, :3]
        yaw_total += abs(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
    res = {
        "scans": len(P), "sensor": "Velodyne VLP-16 (16 rings, 10 Hz, per-point time)",
        "config": {"min_range": a.min_range, "max_range": a.max_range, "voxel": a.voxel, "deskew": not a.no_deskew},
        "time_ms": {"mean": round(float(np.mean(times)), 2), "p99": round(float(np.percentile(times, 99)), 2),
                    "max": round(float(np.max(times)), 2)},
        "path_length_m": round(float(seg.sum()), 2),
        "start_to_end_m": round(float(np.linalg.norm(pos[-1] - pos[0])), 3),
        "extent_m": np.round(sv / np.sqrt(len(pos)), 3).tolist(),
        "trajectory_plane_rms_cm": round(planar_rms * 100, 2),
        "speed_mps": {"median": round(float(np.median(speed)), 3), "p99": round(float(np.percentile(speed, 99)), 3),
                      "max": round(float(speed.max()), 3)},
        "cumulative_yaw_deg": round(yaw_total, 1),
    }
    with open(out / "lidar_traj_tum.txt", "w") as f:
        from scipy.spatial.transform import Rotation
        for t, T in zip(stamps, P):
            q = Rotation.from_matrix(T[:3, :3]).as_quat()
            f.write(f"{t / 1e9:.9f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")
    (out / "lidar_feasibility.json").write_text(json.dumps(res, indent=1))

    # top view of the accumulated map (gravity ~ trajectory-plane normal)
    import cv2
    W = np.vstack(world_pts)
    n = Vt[2] if Vt[2][2] >= 0 else -Vt[2]
    e1 = Vt[0]; e2 = np.cross(n, e1)
    rel = W - pos.mean(0)
    h = rel @ n
    keep = (h > -1.5) & (h < 1.5)
    u, v = rel[keep] @ e1, rel[keep] @ e2
    tu, tv = c @ e1, c @ e2
    scale = 60.0                                   # px per metre
    lo = np.percentile(np.concatenate([u, tu]), 0.5) - 0.5, np.percentile(np.concatenate([v, tv]), 0.5) - 0.5
    hi = np.percentile(np.concatenate([u, tu]), 99.5) + 0.5, np.percentile(np.concatenate([v, tv]), 99.5) + 0.5
    Wpx, Hpx = int((hi[0] - lo[0]) * scale), int((hi[1] - lo[1]) * scale)
    img = np.full((Hpx, Wpx, 3), 18, np.uint8)
    px = ((u - lo[0]) * scale).astype(int); py = (Hpx - 1 - (v - lo[1]) * scale).astype(int)
    m = (px >= 0) & (px < Wpx) & (py >= 0) & (py < Hpx)
    acc = np.zeros((Hpx, Wpx), np.float32)
    np.add.at(acc, (py[m], px[m]), 1.0)
    acc = np.clip(np.log1p(acc) / np.log1p(np.percentile(acc[acc > 0], 99)), 0, 1)
    img[..., 0] = img[..., 1] = img[..., 2] = (18 + acc * 220).astype(np.uint8)
    tp = np.column_stack([((tu - lo[0]) * scale), (Hpx - 1 - (tv - lo[1]) * scale)]).astype(np.int32)
    cv2.polylines(img, [tp], False, (60, 180, 255), 2, cv2.LINE_AA)
    cv2.circle(img, tuple(tp[0]), 6, (80, 220, 80), -1); cv2.circle(img, tuple(tp[-1]), 6, (60, 60, 230), -1)
    cv2.putText(img, "KISS-ICP VLP-16 map (top view, 1 m = 60 px) green=start red=end", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(out / "lidar_map_topview.png"), img)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
