#!/usr/bin/env python3
"""Run RTAB-Map RGB-D on the four data_slam bags and normalise outputs.

Must run inside `conda activate rtabmap`. Adapted from run_four_slam_bags.py:
- inputs come from PROJECT_ROOT/data_slam/<dataset>/ (dir with metadata.yaml)
- paths are project-root absolute (no /home/etri hard-coding)
- visualisation PNGs are intentionally NOT produced here (deferred work);
  this script only executes SLAM and extracts trajectory/keyframe/dense map.
"""
from __future__ import annotations

import os
import signal
import sqlite3
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

# Bag playback rate. Lower (e.g. 0.5) gives the rtabmap node more time per frame
# so it never falls behind / drops frames on slower or loaded machines — which
# otherwise degrades RGB-D odometry and causes trajectory drift. Override via
# env RTAB_BAG_RATE.
BAG_RATE = os.environ.get("RTAB_BAG_RATE", "1.0")

# RTAB-Map node args. DEFAULT = RTAB-Map's native real-time config: just delete
# the old DB on start ("-d") and let RTAB use its real-time defaults
# (Rtabmap/DetectionRate=1Hz, RGBD/LinearUpdate=0.1m, RGBD/AngularUpdate=0.1rad).
# The previous aggressive args (DetectionRate 0 / LinearUpdate 0 / AngularUpdate 0)
# forced per-frame loop-closure + per-frame node creation, which is NOT real-time
# and caused backlog/drift on a loaded machine. Override via env if needed.
RTAB_ARGS = os.environ.get("RTAB_ARGS", "-d")
RTAB_ODOM_ARGS = os.environ.get("RTAB_ODOM_ARGS", "")

PROJECT_ROOT = Path(__file__).resolve().parents[2]   # .../CLI_environment
RTABMAP_WS = PROJECT_ROOT / "rtabmap_ws"
DATA_ROOT = PROJECT_ROOT / "data_slam"
OUTPUT_ROOT = RTABMAP_WS / "output" / os.environ.get("RTAB_OUTPUT_SUBDIR", "")
# stats mode: export poses only (skip heavy cloud assembly) for speed
_EXPORT_CLOUD = "" if os.environ.get("RTAB_POSES_ONLY") else "--cloud "

ENV_PREFIX = f"source {RTABMAP_WS}/install/setup.bash && "

# Hard wall-clock ceiling for a single bag playback (sec). Bags are ~35-130s at
# rate 1.0; this is only a watchdog so a stalled rosbag2 player cannot hang the
# whole batch indefinitely.
BAG_TIMEOUT_SEC = 900

DATASETS = [
    "SLAM_forward_backward_repeat",
    "SLAM_one_lap",
    "SLAM_one_lap_back_and_forth",
    "SLAM_three_laps",
]


def shell(command: str) -> list[str]:
    return ["bash", "-lc", ENV_PREFIX + command]


def run_dataset(name: str) -> dict:
    bag = DATA_ROOT / name              # directory containing metadata.yaml + bag_0.db3
    out = OUTPUT_ROOT / name
    out.mkdir(parents=True, exist_ok=True)

    for pattern in [
        "rtabmap.db", "rtabmap.db-*", "rtabmap_3d_map*",
        "rtabmap_launch.log", "bag_play.log", "rtabmap_export.log",
        "trajectory.txt", "keyframes.txt", "dense_map.pcd",
        # stale artifacts from prior sessions (e.g. old viz PNGs) — remove so a
        # later visualisation step never picks up a stale image by accident
        "rtabmap_2d_top_map*.png", "run_summary.md",
    ]:
        for path in out.glob(pattern):
            if path.is_file():
                path.unlink()

    db = out / "rtabmap.db"
    launch_cmd = (
        "ros2 launch rtabmap_launch rtabmap.launch.py "
        "rtabmap_viz:=false rviz:=false use_sim_time:=true "
        "frame_id:=camera_color_optical_frame "
        "rgb_topic:=/camera/camera/color/image_raw "
        "depth_topic:=/camera/camera/aligned_depth_to_color/image_raw "
        "camera_info_topic:=/camera/camera/color/camera_info "
        "approx_sync:=true approx_sync_max_interval:=0.05 "
        "qos:=2 qos_image:=2 qos_camera_info:=2 "
        "topic_queue_size:=100 queue_size:=100 "
        f"database_path:={db} "
        f'args:="{RTAB_ARGS}"'
    )
    if RTAB_ODOM_ARGS.strip():
        launch_cmd += f' odom_args:="{RTAB_ODOM_ARGS}"'
    bag_cmd = f"ros2 bag play {bag} --clock --rate {BAG_RATE}"

    with (out / "rtabmap_launch.log").open("wb") as launch_log, \
         (out / "bag_play.log").open("wb") as bag_log:
        launch_proc = subprocess.Popen(
            shell(launch_cmd), cwd=str(RTABMAP_WS), stdout=launch_log,
            stderr=subprocess.STDOUT, preexec_fn=os.setsid,
        )
        time.sleep(8.0)
        bag_proc = subprocess.Popen(
            shell(bag_cmd), cwd=str(RTABMAP_WS), stdout=bag_log,
            stderr=subprocess.STDOUT, preexec_fn=os.setsid,
        )
        try:
            bag_rc = bag_proc.wait(timeout=BAG_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(bag_proc.pid), signal.SIGINT)
            try:
                bag_rc = bag_proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(bag_proc.pid), signal.SIGKILL)
                bag_rc = bag_proc.wait()
            print(f"    WARN: bag playback for {name} exceeded {BAG_TIMEOUT_SEC}s; killed", flush=True)
        time.sleep(5.0)
        if launch_proc.poll() is None:
            os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
            try:
                launch_rc = launch_proc.wait(timeout=120)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(launch_proc.pid), signal.SIGTERM)
                try:
                    launch_rc = launch_proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(launch_proc.pid), signal.SIGKILL)
                    launch_rc = launch_proc.wait()
        else:
            launch_rc = launch_proc.returncode

    export_cmd = (
        f"rtabmap-export {_EXPORT_CLOUD}--poses --poses_format 11 --ascii "
        f"--output rtabmap_3d_map --output_dir {out} {db}"
    )
    with (out / "rtabmap_export.log").open("wb") as export_log:
        export_rc = subprocess.run(
            shell(export_cmd), cwd=str(RTABMAP_WS), stdout=export_log,
            stderr=subprocess.STDOUT,
        ).returncode

    # --- normalise outputs to spec names ---
    ply = out / "rtabmap_3d_map_cloud.ply"
    poses = out / "rtabmap_3d_map_poses.txt"
    n_points = 0
    if ply.exists() and ply.stat().st_size > 0:
        points, colors = read_ascii_ply(ply)
        if points.size:                       # drop non-finite (nan/inf) coords
            mask = np.isfinite(points).all(axis=1)
            points = points[mask]
            if colors is not None:
                colors = colors[mask]
        write_ascii_pcd(out / "dense_map.pcd", points, colors)
        n_points = int(points.shape[0])
    n_traj = normalise_poses(poses, out / "trajectory.txt")
    # RTAB-Map graph nodes ARE keyframes -> keyframes.txt == trajectory.txt
    normalise_poses(poses, out / "keyframes.txt")

    counts = db_counts(db)
    return {
        "name": name, "bag_rc": bag_rc, "launch_rc": launch_rc,
        "export_rc": export_rc, "node_count": counts.get("Node", 0),
        "link_count": counts.get("Link", 0), "traj_rows": n_traj,
        "dense_points": n_points, "output": str(out),
    }


def normalise_poses(src: Path, dst: Path) -> int:
    """rtabmap poses_format 11 (#ts x y z qx qy qz qw id) -> ts x y z qx qy qz qw."""
    if not src.exists():
        dst.write_text("", encoding="utf-8")
        return 0
    rows = []
    with src.open("r", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 8:
                rows.append(" ".join(parts[:8]))
    dst.write_text(("\n".join(rows) + "\n") if rows else "", encoding="utf-8")
    return len(rows)


def db_counts(db: Path) -> dict:
    counts = {}
    if not db.exists():
        return counts
    con = sqlite3.connect(str(db))
    try:
        for table in ("Node", "Link", "Data"):
            try:
                counts[table] = int(con.execute(f"select count(*) from {table}").fetchone()[0])
            except sqlite3.Error:
                counts[table] = 0
    finally:
        con.close()
    return counts


def read_ascii_ply(path: Path):
    properties, vertex_count, in_vertex = [], 0, False
    with path.open("r", encoding="ascii", errors="replace") as f:
        for line in f:
            line = line.strip()
            if line.startswith("element vertex"):
                vertex_count = int(line.split()[-1]); in_vertex = True
            elif line.startswith("element "):
                in_vertex = False
            elif in_vertex and line.startswith("property"):
                properties.append(line.split()[-1])
            elif line == "end_header":
                break
        ncol = len(properties)
        rows = []
        for _ in range(vertex_count):
            line = f.readline()
            if not line:
                break
            parts = line.split()
            if len(parts) == ncol:        # skip ragged/truncated rows
                rows.append(parts)
    if not rows:
        return np.empty((0, 3), dtype=np.float32), None
    try:
        data = np.asarray(rows, dtype=np.float32)
    except ValueError:
        return np.empty((0, 3), dtype=np.float32), None
    ix, iy, iz = (properties.index(a) for a in ("x", "y", "z"))
    points = data[:, [ix, iy, iz]]
    colors = None
    if all(n in properties for n in ("red", "green", "blue")):
        ir, ig, ib = (properties.index(n) for n in ("red", "green", "blue"))
        colors = data[:, [ir, ig, ib]].clip(0, 255).astype(np.uint8)
    return points, colors


def write_ascii_pcd(path: Path, points: np.ndarray, colors) -> None:
    with path.open("w", encoding="ascii") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n")
        if colors is None:
            f.write("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
            f.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n")
            np.savetxt(f, points, fmt="%.6f %.6f %.6f")
            return
        f.write("FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n")
        f.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n")
        rgb = ((colors[:, 0].astype(np.uint32) << 16)
               | (colors[:, 1].astype(np.uint32) << 8) | colors[:, 2].astype(np.uint32))
        for p, v in zip(points, rgb):
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {int(v)}\n")


def main() -> int:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    targets = [a for a in sys.argv[1:] if not a.startswith("-")] or DATASETS
    print(f"(bag rate={BAG_RATE}, datasets={targets})", flush=True)
    results = []
    for name in targets:
        print(f"=== RTAB-Map: {name} ===", flush=True)
        r = run_dataset(name)
        print(f"    nodes={r['node_count']} links={r['link_count']} "
              f"traj_rows={r['traj_rows']} dense_points={r['dense_points']}", flush=True)
        results.append(r)
    print("\n=== RTAB-Map summary ===")
    all_ok = True
    for r in results:
        ok = r["traj_rows"] > 1 and r["dense_points"] > 0 and r["export_rc"] == 0
        all_ok = all_ok and ok
        print(f"- [{'PASS' if ok else 'FAIL'}] {r['name']}: nodes={r['node_count']} "
              f"traj={r['traj_rows']} dense={r['dense_points']} export_rc={r['export_rc']} "
              f"-> {r['output']}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
