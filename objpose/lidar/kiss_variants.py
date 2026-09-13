#!/usr/bin/env python3
"""kiss_variants.py — KISS-ICP variants on the 260910_object VLP-16 bag (runs on the Mac, offline, back-to-back).

  base        as streamed live: deskew with the previous scan's motion (constant-velocity model)
  nodeskew    no motion compensation
  iterdeskew  after ICP, deskew the raw scan again with the motion just estimated for *this* scan and
              re-run ICP from that pose (N passes); the map is updated with the final deskewed scan
  iter2       iterdeskew with 2 passes
  iterdamp    iterdeskew, deskew motion = halfway between the previous and the current scan's motion
  voxel05     base with voxel 0.05 m (finer map / more correspondences)

Writes <out>/<variant>_tum.txt stamped with the velodyne header stamp (same as lidar_stream.py).
"""
from __future__ import annotations

import argparse
import json
import sqlite3
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lidar_stream import header_stamp, parse_pc2  # noqa: E402


def make(voxel, deskew):
    from kiss_icp.config import load_config
    from kiss_icp.kiss_icp import KissICP
    cfg = load_config(None)
    cfg.data.min_range, cfg.data.max_range, cfg.data.deskew = 0.5, 30.0, deskew
    cfg.mapping.voxel_size = voxel
    return KissICP(cfg)


def blend(Ta, Tb, u):
    from scipy.spatial.transform import Slerp
    R = Slerp([0, 1], Rotation.from_matrix([Ta[:3, :3], Tb[:3, :3]]))([u]).as_matrix()[0]
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = (1 - u) * Ta[:3, 3] + u * Tb[:3, 3]
    return T


def register_iterative(odom, frame, timestamps, passes, damp=1.0):
    """KissICP.register_frame, with the deskew re-done using the current scan's own estimated motion
    (damp < 1 blends that motion with the previous scan's, so a single noisy registration cannot flip it)"""
    guess = odom.last_pose @ odom.last_delta
    delta = odom.last_delta
    sigma = odom.adaptive_threshold.get_threshold()
    pose = guess
    for _ in range(passes):
        f = odom.preprocessor.preprocess(frame, timestamps, delta)
        source, frame_down = odom.voxelize(f)
        pose = odom.registration.align_points_to_map(points=source, voxel_map=odom.local_map, initial_guess=pose,
                                                     max_correspondance_distance=3 * sigma, kernel=sigma)
        delta = blend(odom.last_delta, np.linalg.inv(odom.last_pose) @ pose, damp)
    odom.adaptive_threshold.update_model_deviation(np.linalg.inv(guess) @ pose)
    odom.local_map.update(frame_down, pose)
    odom.last_delta = np.linalg.inv(odom.last_pose) @ pose
    odom.last_pose = pose


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--variants", default="base,nodeskew,iterdeskew,voxel05")
    ap.add_argument("--passes", type=int, default=3)
    a = ap.parse_args()
    db = next(Path(a.session).expanduser().glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    tid = con.execute("select id from topics where name='/velodyne_points'").fetchone()[0]
    rows = con.execute("select id, substr(data, 1, 16) from messages where topic_id=?", (tid,)).fetchall()
    frames = sorted((header_stamp(h), mid) for mid, h in rows)
    scans = []
    for t_ns, mid in frames:
        pts = parse_pc2(con.execute("select data from messages where id=?", (mid,)).fetchone()[0])
        xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
        r = np.linalg.norm(xyz, axis=1)
        ok = np.isfinite(r) & (r > 0.1)
        ts = pts["time"][ok].astype(np.float64)
        scans.append((t_ns, xyz[ok], (ts - ts.min()) / max(ts.max() - ts.min(), 1e-9)))
    out = Path(a.out).expanduser(); out.mkdir(parents=True, exist_ok=True)

    # which instant does the deskewed scan refer to? (point with timestamp u is left unmoved)
    probe = make(0.1, True)
    d = np.eye(4); d[0, 3] = 1.0
    moved = probe.preprocessor.preprocess(np.tile([5.0, 0.0, 0.0], (3, 1)), np.array([0.0, 0.5, 1.0]), d)   # x = 4 / 4.5 / 5: referenced to the scan end
    ref = {"deskew_probe_x_for_u_0_05_1": [round(float(v), 3) for v in np.asarray(moved)[:, 0]]}
    print(json.dumps(ref), flush=True)

    summary = {"deskew_reference": ref}
    for v in a.variants.split(","):
        odom = make(0.05 if v == "voxel05" else 0.10, v != "nodeskew")
        t0 = time.perf_counter()
        with open(out / f"{v}_tum.txt", "w") as f:
            for t_ns, xyz, u in scans:
                if v == "iterdeskew":
                    register_iterative(odom, xyz, u, a.passes)
                elif v == "iter2":
                    register_iterative(odom, xyz, u, 2)
                elif v == "iterdamp":
                    register_iterative(odom, xyz, u, a.passes, damp=0.5)
                else:
                    odom.register_frame(xyz, u)
                T = odom.last_pose
                q = Rotation.from_matrix(T[:3, :3]).as_quat()
                f.write(f"{t_ns / 1e9:.9f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")
        ms = (time.perf_counter() - t0) * 1e3 / len(scans)
        summary[v] = {"ms_per_scan": round(ms, 2)}
        print(v, summary[v], flush=True)
    (out / "summary.json").write_text(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
