#!/usr/bin/env python3
"""kiss_gt.py — KISS-ICP LiDAR odometry of a 260901_cbnu VLP-16 bag, used as the reference ("GT") trajectory
for the ORB-SLAM3 / gyro-aided ORB-SLAM3 comparison (docs/orb_imu_260918.md).

Same odometry settings as rt/260915/kiss_260915.py / lidar_stream.py (kiss-icp 1.3.0, min_range 0.5 m,
max_range 30 m, voxel 0.10 m) plus the streamer's second deskew pass (re-deskew with the scan's own motion
and register again), which removed the start/stop/turn spikes on 260910. The 260901 bags are a clean 10 Hz
stream (no duplicated messages, unlike 260915), so every scan is used.

Poses are T_world_velodyne (world = first scan), stamped with the velodyne header stamp. The per-point
`time` field runs from about -0.0995 to 0 s, i.e. the header stamp is the END of the sweep, which is also
the instant KISS-ICP deskews to — so stamp and pose agree.

    ~/objpose_kiss_venv/bin/python objpose/rt/260901/bigeight/kiss_gt.py --session <lidar dir> --out <dir>
"""
from __future__ import annotations
import argparse, json, sqlite3, sys, time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "objpose" / "lidar"))
from lidar_stream import header_stamp, parse_pc2, register  # noqa: E402


def make(voxel, deskew):
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
    ap.add_argument("--voxel", type=float, default=0.10)
    ap.add_argument("--deskew-passes", type=int, default=2)
    ap.add_argument("--map-stride", type=int, default=10, help="keep every Nth deskewed scan for the map preview")
    a = ap.parse_args()
    db = next(Path(a.session).expanduser().glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?immutable=1", uri=True)
    tid = con.execute("select id from topics where name='/velodyne_points'").fetchone()[0]
    rows = con.execute("select id, substr(data, 1, 16) from messages where topic_id=? order by timestamp", (tid,)).fetchall()
    uniq = {}
    for mid, h in rows:
        uniq.setdefault(header_stamp(h), mid)
    stamps = sorted(uniq)
    dt = np.diff(stamps) / 1e6
    stats = {"messages": len(rows), "unique_header_stamps": len(stamps), "scans_used": len(stamps),
             "dt_ms": {"median": round(float(np.median(dt)), 3), "std": round(float(dt.std()), 3),
                       "min": round(float(dt.min()), 2), "max": round(float(dt.max()), 2)}}
    print(json.dumps(stats), flush=True)

    out = Path(a.out).expanduser(); out.mkdir(parents=True, exist_ok=True)
    odom = make(a.voxel, True)
    t0 = time.perf_counter()
    poses, cloud = [], []
    with open(out / "kiss_tum.txt", "w") as f:
        for k, t_ns in enumerate(stamps):
            pts = parse_pc2(con.execute("select data from messages where id=?", (uniq[t_ns],)).fetchone()[0])
            xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
            r = np.linalg.norm(xyz, axis=1)
            ok = np.isfinite(r) & (r > 0.1)
            ts = pts["time"][ok].astype(np.float64)
            u = (ts - ts.min()) / max(ts.max() - ts.min(), 1e-9)
            register(odom, xyz[ok], u, a.deskew_passes)
            T = np.array(odom.last_pose)
            poses.append(T)
            q = Rotation.from_matrix(T[:3, :3]).as_quat()
            f.write(f"{t_ns/1e9:.9f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")
            if k % a.map_stride == 0:
                p = xyz[ok][::6]
                p = p[(np.linalg.norm(p, axis=1) < 20.0)]
                cloud.append((T[:3, :3] @ p.T).T + T[:3, 3])
            if k % 500 == 0:
                print(f"  {k}/{len(stamps)}", flush=True)
    stats["ms_per_scan"] = round((time.perf_counter() - t0) * 1e3 / len(stamps), 2)
    P = np.array([T[:3, 3] for T in poses])
    stats["path_m"] = round(float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum()), 2)
    stats["return_error_cm"] = round(float(np.linalg.norm(P[-1] - P[0])) * 100, 1)
    yaw = np.unwrap([np.arctan2(T[1, 0], T[0, 0]) for T in poses])
    stats["total_abs_yaw_deg"] = round(float(np.degrees(np.abs(np.diff(yaw)).sum())), 1)
    stats["net_yaw_deg"] = round(float(np.degrees(yaw[-1] - yaw[0])), 1)
    C = np.concatenate(cloud)
    # voxel-thin the preview map to 0.15 m
    key = np.unique(np.floor(C / 0.15).astype(np.int64), axis=0, return_index=True)[1]
    np.save(out / "kiss_map_preview.npy", C[key].astype(np.float32))
    stats["map_preview_points"] = int(len(key))
    (out / "kiss_summary.json").write_text(json.dumps(stats, indent=1))
    print(json.dumps(stats), flush=True)


if __name__ == "__main__":
    main()
