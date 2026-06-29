#!/usr/bin/env python3
from __future__ import annotations

import os
import signal
import sqlite3
import subprocess
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


DATASETS = [
    "SLAM_forward_backward_repeat",
    "SLAM_one_lap",
    "SLAM_one_lap_back_and_forth",
    "SLAM_three_laps",
]

HOME = Path.home()
RTABMAP_WS = HOME / "CLI_environment" / "rtabmap_ws"
ORB_DATA = HOME / "CLI_environment" / "orbslam_ws" / "data"
OUTPUT_ROOT = RTABMAP_WS / "output"

ENV_PREFIX = "source /home/etri/CLI_environment/rtabmap_ws/install/setup.bash && "


def shell(command: str) -> list[str]:
    return ["bash", "-lc", ENV_PREFIX + command]


def run_dataset(name: str) -> dict[str, object]:
    bag = ORB_DATA / name / "bag"
    out = OUTPUT_ROOT / name
    out.mkdir(parents=True, exist_ok=True)

    for pattern in [
        "rtabmap.db",
        "rtabmap.db-*",
        "rtabmap_3d_map*",
        "rtabmap_2d_top_map*",
        "rtabmap_launch.log",
        "bag_play.log",
        "rtabmap_export.log",
        "run_summary.md",
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
        'args:="-d --Rtabmap/DetectionRate 0 --RGBD/LinearUpdate 0 '
        '--RGBD/AngularUpdate 0 --RGBD/ProximityBySpace true '
        '--RGBD/ProximityByTime true --Vis/MinInliers 10 --Kp/MaxFeatures 1000" '
        'odom_args:="--Odom/ResetCountdown 1 --Vis/MinInliers 10 '
        '--OdomF2M/MaxSize 2000 --Odom/KeyFrameThr 0.3"'
    )
    bag_cmd = f"ros2 bag play {bag} --clock --rate 1.0"

    with (out / "rtabmap_launch.log").open("wb") as launch_log, (out / "bag_play.log").open("wb") as bag_log:
        launch_proc = subprocess.Popen(
            shell(launch_cmd),
            cwd=str(RTABMAP_WS),
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        time.sleep(8.0)
        bag_proc = subprocess.Popen(
            shell(bag_cmd),
            cwd=str(RTABMAP_WS),
            stdout=bag_log,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        bag_rc = bag_proc.wait()
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
        "rtabmap-export --cloud --poses --poses_format 11 --ascii "
        f"--output rtabmap_3d_map --output_dir {out} {db}"
    )
    with (out / "rtabmap_export.log").open("wb") as export_log:
        export_rc = subprocess.run(
            shell(export_cmd),
            cwd=str(RTABMAP_WS),
            stdout=export_log,
            stderr=subprocess.STDOUT,
        ).returncode

    ply = out / "rtabmap_3d_map_cloud.ply"
    pcd = out / "rtabmap_3d_map.pcd"
    poses = out / "rtabmap_3d_map_poses.txt"
    points = np.empty((0, 3), dtype=np.float32)
    if ply.exists() and ply.stat().st_size > 0:
        points, colors = read_ascii_ply(ply)
        write_ascii_pcd(pcd, points, colors)
        plot_top(out / "rtabmap_2d_top_map.png", points, None)
        trajectory = read_poses(poses)
        plot_top(out / "rtabmap_2d_top_map_trajectory_start_end.png", points, trajectory)

    counts = db_counts(db)
    summary = {
        "name": name,
        "bag": str(bag),
        "output": str(out),
        "bag_returncode": bag_rc,
        "launch_returncode": launch_rc,
        "export_returncode": export_rc,
        "db_size_bytes": db.stat().st_size if db.exists() else 0,
        "node_count": counts.get("Node", 0),
        "link_count": counts.get("Link", 0),
        "data_count": counts.get("Data", 0),
        "cloud_points": int(points.shape[0]),
    }
    write_summary(out / "run_summary.md", summary, launch_cmd, bag_cmd, export_cmd)
    return summary


def db_counts(db: Path) -> dict[str, int]:
    counts: dict[str, int] = {}
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


def read_ascii_ply(path: Path) -> tuple[np.ndarray, np.ndarray | None]:
    properties: list[str] = []
    vertex_count = 0
    in_vertex = False
    with path.open("r", encoding="ascii", errors="replace") as f:
        for line in f:
            line = line.strip()
            if line.startswith("element vertex"):
                vertex_count = int(line.split()[-1])
                in_vertex = True
            elif line.startswith("element ") and not line.startswith("element vertex"):
                in_vertex = False
            elif in_vertex and line.startswith("property"):
                properties.append(line.split()[-1])
            elif line == "end_header":
                break
        rows = []
        for _ in range(vertex_count):
            line = f.readline()
            if not line:
                break
            rows.append(line.split())

    if not rows:
        return np.empty((0, 3), dtype=np.float32), None

    data = np.asarray(rows, dtype=np.float32)
    ix, iy, iz = (properties.index(axis) for axis in ("x", "y", "z"))
    points = data[:, [ix, iy, iz]]
    colors = None
    if all(name in properties for name in ("red", "green", "blue")):
        ir, ig, ib = (properties.index(name) for name in ("red", "green", "blue"))
        colors = data[:, [ir, ig, ib]].clip(0, 255).astype(np.uint8)
    return points, colors


def write_ascii_pcd(path: Path, points: np.ndarray, colors: np.ndarray | None) -> None:
    with path.open("w", encoding="ascii") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        if colors is None:
            f.write("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
            f.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n")
            np.savetxt(f, points, fmt="%.6f %.6f %.6f")
            return
        f.write("FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n")
        f.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n")
        rgb = (
            (colors[:, 0].astype(np.uint32) << 16)
            | (colors[:, 1].astype(np.uint32) << 8)
            | colors[:, 2].astype(np.uint32)
        )
        for point, value in zip(points, rgb):
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {int(value)}\n")


def read_poses(path: Path) -> np.ndarray:
    if not path.exists():
        return np.empty((0, 3), dtype=np.float32)
    rows: list[list[float]] = []
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            parts = line.split()
            if len(parts) >= 4:
                try:
                    rows.append([float(parts[1]), float(parts[2]), float(parts[3])])
                except ValueError:
                    pass
    return np.asarray(rows, dtype=np.float32) if rows else np.empty((0, 3), dtype=np.float32)


def plot_top(path: Path, points: np.ndarray, trajectory: np.ndarray | None) -> None:
    if points.size == 0:
        return
    xy = points[:, [0, 2]]
    xy = xy[np.isfinite(xy).all(axis=1)]
    if len(xy) > 300000:
        xy = xy[:: max(1, len(xy) // 300000)]
    fig, ax = plt.subplots(figsize=(10, 10), dpi=180)
    ax.scatter(xy[:, 0], xy[:, 1], s=0.12, c="#222222", alpha=0.5, linewidths=0)
    if trajectory is not None and len(trajectory):
        txy = trajectory[:, [0, 2]]
        txy = txy[np.isfinite(txy).all(axis=1)]
        if len(txy):
            ax.plot(txy[:, 0], txy[:, 1], c="#0077ff", linewidth=2.0)
            ax.scatter(txy[0, 0], txy[0, 1], c="#1a9c45", s=90, edgecolors="white")
            ax.scatter(txy[-1, 0], txy[-1, 1], c="#d62728", s=90, edgecolors="white")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.grid(True, linewidth=0.4, color="#d8d8d8")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def write_summary(path: Path, summary: dict[str, object], launch_cmd: str, bag_cmd: str, export_cmd: str) -> None:
    lines = [
        f"# RTAB-Map Run Summary: {summary['name']}",
        "",
        f"- bag: `{summary['bag']}`",
        f"- output: `{summary['output']}`",
        f"- bag_returncode: `{summary['bag_returncode']}`",
        f"- launch_returncode: `{summary['launch_returncode']}`",
        f"- export_returncode: `{summary['export_returncode']}`",
        f"- db_size_bytes: `{summary['db_size_bytes']}`",
        f"- node_count: `{summary['node_count']}`",
        f"- link_count: `{summary['link_count']}`",
        f"- data_count: `{summary['data_count']}`",
        f"- cloud_points: `{summary['cloud_points']}`",
        "",
        "## Commands",
        "",
        f"- launch: `{launch_cmd}`",
        f"- bag: `{bag_cmd}`",
        f"- export: `{export_cmd}`",
        "",
        "## Files",
        "",
        "- `rtabmap.db`",
        "- `rtabmap_3d_map_cloud.ply`",
        "- `rtabmap_3d_map.pcd`",
        "- `rtabmap_3d_map_poses.txt`",
        "- `rtabmap_2d_top_map.png`",
        "- `rtabmap_2d_top_map_trajectory_start_end.png`",
        "- `rtabmap_launch.log`",
        "- `bag_play.log`",
        "- `rtabmap_export.log`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    summaries = []
    for name in DATASETS:
        print(f"=== Running {name} ===", flush=True)
        summaries.append(run_dataset(name))

    lines = ["# RTAB-Map Four Sequence Summary", ""]
    for s in summaries:
        lines.append(
            f"- {s['name']}: nodes={s['node_count']}, links={s['link_count']}, "
            f"points={s['cloud_points']}, output=`{s['output']}`"
        )
    (OUTPUT_ROOT / "rtabmap_four_slam_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(OUTPUT_ROOT / "rtabmap_four_slam_summary.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
