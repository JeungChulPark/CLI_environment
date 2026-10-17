#!/usr/bin/env python3
"""kiss_260915.py — KISS-ICP "base" LiDAR odometry for the 2026-09-15 VLP-16 bags.

Same odometry as objpose/lidar/kiss_variants.py "base" (kiss-icp 1.3.0, deskew with the previous
scan's motion, min_range 0.5 m, max_range 30 m, voxel 0.10 m) and the same TUM output stamped with
the velodyne header stamp.

The only difference is the input stream. Unlike 260910 (a clean 10 Hz /velodyne_points), the 260915
bags contain ~40 messages/s on /velodyne_points:
  * every message is recorded TWICE (byte-identical, two rosbag2 subscriptions), and
  * there are TWO independent 10 Hz publishers whose full 29 184-point rotations are ~6.5 ms apart.
Feeding that to KISS-ICP breaks its constant-velocity deskew (alternating 6.5 ms / 94 ms steps), so
this script de-duplicates by header stamp and keeps ONE of the two publishers -> a clean ~10 Hz
stream, exactly the input 260910 had.

    python3 objpose/rt/260915/kiss_260915.py --session <lidar dir> --out <dir> [--stream a|b]
"""
from __future__ import annotations
import argparse, json, sqlite3, sys, time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

REPO = Path("/home/jucpark/DeepLearning/CLI_environment")
sys.path.insert(0, str(REPO / "objpose" / "lidar"))
from lidar_stream import header_stamp, parse_pc2  # noqa: E402



def make(voxel=0.10, deskew=True):
    from kiss_icp.config import load_config
    from kiss_icp.kiss_icp import KissICP
    cfg = load_config(None)
    cfg.data.min_range, cfg.data.max_range, cfg.data.deskew = 0.5, 30.0, deskew
    cfg.mapping.voxel_size = voxel
    return KissICP(cfg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stream", default="a", choices=["a", "b"])
    ap.add_argument("--voxel", type=float, default=0.10)
    a = ap.parse_args()
    db = next(Path(a.session).expanduser().glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?immutable=1", uri=True)
    tid = con.execute("select id from topics where name='/velodyne_points'").fetchone()[0]
    rows = con.execute("select id, substr(data, 1, 16) from messages where topic_id=?", (tid,)).fetchall()
    uniq = {}
    for mid, h in rows:
        uniq.setdefault(header_stamp(h), mid)          # drop the byte-identical duplicate recordings
    stamps = np.array(sorted(uniq))
    # the two publishers strictly interleave (verified: both sub-streams are a clean 100.865 ms
    # sequence, their offset is 6.6 ms on eightcircle, 15.9 ms on distance, ~50 ms on dense), so the
    # alternating split is the robust one -- a fixed time window is not.
    k = 0 if a.stream == "a" else 1
    picked = [int(t) for t in stamps[k::2]]
    other = [int(t) for t in stamps[1 - k::2]]
    groups = [len(picked), len(other)]
    dt = np.diff(picked) / 1e6
    stats = {"messages": len(rows), "unique_header_stamps": int(len(stamps)),
             "duplicate_messages": len(rows) - int(len(stamps)),
             "substream_sizes": groups, "stream": a.stream, "scans_used": len(picked),
             "publisher_offset_ms": round(float(np.median(np.array(other[:min(len(other), len(picked))])
                                                          - np.array(picked[:min(len(other), len(picked))])) / 1e6), 3),
             "dt_ms": {"median": round(float(np.median(dt)), 3), "std": round(float(dt.std()), 3),
                       "min": round(float(dt.min()), 2), "max": round(float(dt.max()), 2)}}
    print(json.dumps(stats), flush=True)

    scans = []
    for t_ns in picked:
        pts = parse_pc2(con.execute("select data from messages where id=?", (uniq[t_ns],)).fetchone()[0])
        xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
        r = np.linalg.norm(xyz, axis=1)
        ok = np.isfinite(r) & (r > 0.1)
        ts = pts["time"][ok].astype(np.float64)
        scans.append((int(t_ns), xyz[ok], (ts - ts.min()) / max(ts.max() - ts.min(), 1e-9)))

    out = Path(a.out).expanduser(); out.mkdir(parents=True, exist_ok=True)
    odom = make(a.voxel, True)
    t0 = time.perf_counter()
    with open(out / "base_tum.txt", "w") as f:
        for t_ns, xyz, u in scans:
            odom.register_frame(xyz, u)
            T = odom.last_pose
            q = Rotation.from_matrix(T[:3, :3]).as_quat()
            f.write(f"{t_ns/1e9:.9f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")
    stats["ms_per_scan"] = round((time.perf_counter() - t0) * 1e3 / len(scans), 2)
    P = np.loadtxt(out / "base_tum.txt")[:, 1:4]
    stats["path_m"] = round(float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum()), 2)
    stats["return_error_cm"] = round(float(np.linalg.norm(P[-1] - P[0])) * 100, 1)
    (out / "summary.json").write_text(json.dumps(stats, indent=1))
    print(json.dumps(stats), flush=True)


if __name__ == "__main__":
    main()
