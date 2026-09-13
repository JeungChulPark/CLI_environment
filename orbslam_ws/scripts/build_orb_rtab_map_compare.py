#!/usr/bin/env python3
"""
Pack one ORB-SLAM3 live run and one RTAB-Map live run (orbslam_ws/mac_harness) into a self-contained
comparison page: floor-plan views of each map with its trajectory, the two trajectories overlaid,
and how far apart the two position estimates are over time.

    build_orb_rtab_map_compare.py --orb output/<orb run> --rtab output/<rtab run> --out <dir>/index.html

Both runs share one world frame: the first camera's optical frame (x right, y down, z forward), so
nothing is aligned or scaled here. There is no ground truth; a gap between the two is disagreement,
not error.

Poses per system:
  ORB-SLAM3  CameraTrajectory.txt, every frame, final (each frame hangs off its optimised keyframe)
  RTAB-Map   CameraTrajectoryLive.txt (older runs: CameraTrajectory.txt), every frame as tracked live, and
             the graph nodes' final optimised camera poses exported from rtabmap.db
Maps: ORB-SLAM3 map points; RTAB-Map's per-node RGB cloud (the last map in stream_maps.bin).
A height band (between the floor and ceiling) is kept so the top view reads as a floor plan.
"""

import argparse
import json
import math
import random
import re
import struct
import subprocess
import tempfile
from pathlib import Path

HDR = 11  # PCD ascii header lines
MAX_MAP_POINTS = 60000


def tum(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        v = line.split()
        if len(v) >= 8 and not line.startswith("#"):  # rtabmap-export appends the node id
            rows.append(tuple(map(float, v[:4])))
    return rows  # (stamp, x, y, z)


def pcd_xyz(path):
    pts = []
    for line in Path(path).read_text().splitlines()[HDR:]:
        v = line.split()
        if len(v) >= 3:
            pts.append((float(v[0]), float(v[1]), float(v[2])))
    return pts


def last_map(path):
    """Last map payload in stream_maps.bin: [u8 2, u8 has_rgb, pad2][u32 n] xyz f32*3n (rgb u8*3n)."""
    data = Path(path).read_bytes()
    off, last = 0, None
    while off + 8 <= len(data):
        typ, has_rgb = data[off], data[off + 1]
        n = struct.unpack_from("<I", data, off + 4)[0]
        size = 8 + n * 12 + (n * 3 if has_rgb else 0)
        if typ != 2 or off + size > len(data):
            break
        last = (off, n)
        off += size
    if last is None:
        return []
    o, n = last
    xyz = struct.unpack_from(f"<{n * 3}f", data, o + 8)
    return [xyz[i:i + 3] for i in range(0, n * 3, 3)]


def rtab_optimized_nodes(db, export):
    """Final optimised camera poses of the graph nodes, converted to the first camera's optical frame."""
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run([export, "--poses_camera", "--poses_format", "11", "--output_dir", tmp, str(db)],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        files = sorted(Path(tmp).glob("*camera_poses*.txt")) or sorted(Path(tmp).glob("*poses*.txt"))
        rows = tum(files[0])
    # world is RTAB-Map's base frame (x forward, y left, z up) at the first pose -> optical (x right, y down, z fwd)
    return [(t, -y, -z, x) for t, x, y, z in rows]


def pct(vals, p):
    s = sorted(vals)
    return s[min(len(s) - 1, max(0, int(round(p / 100 * (len(s) - 1)))))]


def floor_band(points):
    """Keep a height band between the floor and the ceiling, so the top view reads as a floor plan."""
    ys = [p[1] for p in points]
    if not ys:
        return points, (0, 0)
    ceiling, floor = pct(ys, 3), pct(ys, 97)  # y grows downwards
    lo, hi = ceiling + 0.35 * (floor - ceiling), floor - 0.2 * (floor - ceiling)
    kept = [p for p in points if lo <= p[1] <= hi]
    return kept, (lo, hi)


def path_len(xyz):
    return sum(math.dist(a, b) for a, b in zip(xyz, xyz[1:]))


def summary(path):
    s = {}
    for line in Path(path).read_text().splitlines():
        k, _, v = line.partition(" ")
        s[k] = v
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--orb", required=True)
    ap.add_argument("--rtab", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--export", default="/opt/homebrew/opt/rtabmap/bin/rtabmap-export")
    ap.add_argument("--dataset", default="260826_etri_eightcircle_dark")
    args = ap.parse_args()
    orb, rtab = Path(args.orb), Path(args.rtab)
    random.seed(7)

    orb_traj = tum(orb / "CameraTrajectory.txt")
    # live_rtabmap now writes the final trajectory to CameraTrajectory.txt and the live one beside it
    live = rtab / "CameraTrajectoryLive.txt"
    rtab_traj = tum(live if live.exists() else rtab / "CameraTrajectory.txt")
    nodes = rtab_optimized_nodes(rtab / "rtabmap.db", args.export)

    orb_all = pcd_xyz(orb / "map_points.pcd")
    n_orb_all = len(orb_all)
    orb_map, orb_band = floor_band(orb_all)
    rtab_all = last_map(rtab / "stream_maps.bin")
    rtab_map, rtab_band = floor_band(rtab_all)
    n_rtab_band = len(rtab_map)
    if len(rtab_map) > MAX_MAP_POINTS:
        rtab_map = random.sample(rtab_map, MAX_MAP_POINTS)

    # disagreement between the two estimates at the same frame (both stamp frame i as 1000 + i/30)
    rt = {round(t, 3): (x, y, z) for t, x, y, z in rtab_traj}
    gap = []
    for t, x, y, z in orb_traj:
        q = rt.get(round(t, 3))
        if q:
            gap.append((round(t - 1000, 3), round(math.dist((x, y, z), q), 4)))
    orb_by_t = {round(t, 3): (x, y, z) for t, x, y, z in orb_traj}
    node_gap = []
    for t, x, y, z in nodes:
        # nodes carry the frame stamp they were made from; take the nearest ORB frame
        k = round(round((t - 1000) * 30) / 30 + 1000, 3)
        q = orb_by_t.get(k)
        if q:
            node_gap.append((round(t - 1000, 3), round(math.dist((x, y, z), q), 4)))

    so, sr = summary(orb / "summary.txt"), summary(rtab / "summary.txt")

    def ms(s):
        m = re.search(r"mean ([\d.]+) p50 [\d.]+ p95 [\d.]+ p99 ([\d.]+)", s.get("track_ms", ""))
        return (float(m.group(1)), float(m.group(2))) if m else (None, None)

    def xz(rows, i0=1):
        return [[round(r[i0], 3), round(r[i0 + 2], 3)] for r in rows]

    orb_xyz = [p[1:] for p in orb_traj]
    rtab_xyz = [p[1:] for p in rtab_traj]
    node_xyz = [p[1:] for p in nodes]
    out = {
        "dataset": args.dataset,
        "orb": {
            "run": orb.name, "traj": xz(orb_traj), "t": [round(p[0] - 1000, 3) for p in orb_traj],
            "map": [[round(p[0], 3), round(p[2], 3)] for p in orb_map],
            "stats": {"frames": len(orb_traj), "path_m": path_len(orb_xyz), "closure_m": math.dist(orb_xyz[0], orb_xyz[-1]),
                      "map_points": n_orb_all,
                      "track": ms(so), "over": int(so.get("over_33.33ms", "0")), "drops": int(so.get("queue_dropped", "0"))},
        },
        "rtab": {
            "run": rtab.name, "traj": xz(rtab_traj), "t": [round(p[0] - 1000, 3) for p in rtab_traj],
            "nodes": xz(nodes), "map": [[round(p[0], 3), round(p[2], 3)] for p in rtab_map],
            "stats": {"frames": len(rtab_traj), "path_m": path_len(rtab_xyz), "closure_m": math.dist(rtab_xyz[0], rtab_xyz[-1]),
                      "node_path_m": path_len(node_xyz),
                      "node_closure_m": math.dist(node_xyz[0], node_xyz[-1]) if node_xyz else None,
                      "map_points": len(rtab_all), "map_band_points": n_rtab_band, "track": ms(sr),
                      "over": int(sr.get("over_33.33ms", "0")), "drops": int(sr.get("queue_dropped", "0")),
                      "nodes": len(nodes), "loops": int(sr.get("loop_closures", "0"))},
        },
        "gap": gap, "node_gap": node_gap,
        "band": {"orb": orb_band, "rtab": rtab_band},
    }
    html = (Path(__file__).parent / "orb_rtab_map_compare.template.html").read_text()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(html.replace("/*__DATA__*/null", json.dumps(out, separators=(",", ":"))))
    print(f"orb: {len(orb_traj)} poses, {len(orb_map)} map points in band; rtab: {len(rtab_traj)} poses, "
          f"{len(nodes)} nodes, {n_rtab_band} cloud points in band (kept {len(rtab_map)})")
    print(f"gap ORB vs RTAB live: median {pct([g[1] for g in gap], 50):.3f} m, max {max(g[1] for g in gap):.3f} m")
    print(f"wrote {args.out} ({Path(args.out).stat().st_size / 1e6:.2f} MB)")


if __name__ == "__main__":
    main()
