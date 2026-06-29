#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import sqlite3
import struct
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class RtabmapOutputExporter(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rtabmap_output_exporter")
        self.output_dir = Path(args.output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.map_topic = args.map_topic
        self.pose_topic = args.pose_topic
        self.trajectory_db = Path(args.trajectory_db).expanduser() if args.trajectory_db else None
        self.top_plane = args.top_plane
        self.idle_sec = args.idle_sec
        self.min_run_sec = args.min_run_sec
        self.started_at = time.monotonic()
        self.last_update = time.monotonic()
        self.latest_points: np.ndarray | None = None
        self.latest_rgb: np.ndarray | None = None
        self.trajectory: list[tuple[float, float, float]] = (
            _load_db_trajectory(self.trajectory_db) if self.trajectory_db else []
        )
        self.saved = False

        self.create_subscription(PointCloud2, self.map_topic, self._cloud_cb, 1)
        self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, 1000)
        self.create_timer(1.0, self._check_idle)
        self.get_logger().info(f"Saving RTAB-Map outputs under {self.output_dir}")
        self.get_logger().info(f"Subscribing cloud={self.map_topic}, pose={self.pose_topic}")
        if self.trajectory_db:
            self.get_logger().info(
                f"Loaded {len(self.trajectory)} trajectory poses from {self.trajectory_db}"
            )

    def _cloud_cb(self, msg: PointCloud2) -> None:
        field_names = [field.name for field in msg.fields]
        read_fields = ["x", "y", "z"]
        rgb_field = None
        if "rgb" in field_names:
            read_fields.append("rgb")
            rgb_field = "rgb"
        elif "rgba" in field_names:
            read_fields.append("rgba")
            rgb_field = "rgba"

        rows = list(point_cloud2.read_points(msg, field_names=read_fields, skip_nans=True))
        if not rows:
            return

        points = np.empty((len(rows), 3), dtype=np.float32)
        colors = np.empty((len(rows), 3), dtype=np.uint8) if rgb_field else None
        for i, row in enumerate(rows):
            if hasattr(row, "dtype") and getattr(row.dtype, "names", None):
                x, y, z = float(row["x"]), float(row["y"]), float(row["z"])
                rgb_value = row[rgb_field] if rgb_field else None
            else:
                x, y, z = float(row[0]), float(row[1]), float(row[2])
                rgb_value = row[3] if rgb_field else None
            points[i] = (x, y, z)
            if colors is not None:
                colors[i] = _decode_rgb(rgb_value)

        self.latest_points = points
        self.latest_rgb = colors
        self.last_update = time.monotonic()
        self.get_logger().info(f"Received cloud with {len(points)} points")

    def _pose_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.trajectory.append((float(p.x), float(p.y), float(p.z)))
            self.last_update = time.monotonic()

    def _check_idle(self) -> None:
        ran_long_enough = time.monotonic() - self.started_at >= self.min_run_sec
        idle_long_enough = time.monotonic() - self.last_update >= self.idle_sec
        if ran_long_enough and idle_long_enough and self.latest_points is not None:
            self.save_outputs()
            rclpy.shutdown()

    def save_outputs(self) -> None:
        if self.saved or self.latest_points is None:
            return
        self.saved = True
        points = self.latest_points
        colors = self.latest_rgb
        trajectory = np.asarray(self.trajectory, dtype=np.float32)

        pcd_path = self.output_dir / "rtabmap_3d_map.pcd"
        top_path = self.output_dir / "rtabmap_2d_top_map.png"
        trajectory_path = self.output_dir / "rtabmap_2d_top_map_trajectory_start_end.png"

        _write_pcd(pcd_path, points, colors)
        _plot_top_map(top_path, points, None, self.top_plane)
        _plot_top_map(trajectory_path, points, trajectory, self.top_plane)

        self.get_logger().info(f"Wrote {pcd_path}")
        self.get_logger().info(f"Wrote {top_path}")
        self.get_logger().info(f"Wrote {trajectory_path}")


def _decode_rgb(value) -> np.ndarray:
    if isinstance(value, np.ndarray):
        value = value.item()
    if value is None:
        return np.array([160, 160, 160], dtype=np.uint8)
    try:
        if not math.isfinite(float(value)):
            return np.array([160, 160, 160], dtype=np.uint8)
    except (TypeError, ValueError):
        pass
    if isinstance(value, float):
        packed = struct.unpack("I", struct.pack("f", value))[0]
    else:
        packed = int(value)
    r = (packed >> 16) & 0xFF
    g = (packed >> 8) & 0xFF
    b = packed & 0xFF
    return np.array([r, g, b], dtype=np.uint8)


def _write_pcd(path: Path, points: np.ndarray, colors: np.ndarray | None) -> None:
    if colors is None:
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            f"WIDTH {len(points)}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {len(points)}\n"
            "DATA ascii\n"
        )
        with path.open("w", encoding="ascii") as f:
            f.write(header)
            np.savetxt(f, points, fmt="%.5f %.5f %.5f")
        return

    rgb = (
        (colors[:, 0].astype(np.uint32) << 16)
        | (colors[:, 1].astype(np.uint32) << 8)
        | colors[:, 2].astype(np.uint32)
    )
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z rgb\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F U\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA ascii\n"
    )
    with path.open("w", encoding="ascii") as f:
        f.write(header)
        for point, rgb_value in zip(points, rgb):
            f.write(f"{point[0]:.5f} {point[1]:.5f} {point[2]:.5f} {int(rgb_value)}\n")


def _plane_indices(top_plane: str) -> tuple[int, int, str, str]:
    planes = {
        "xy": (0, 1, "x", "y"),
        "xz": (0, 2, "x", "z"),
        "yz": (1, 2, "y", "z"),
    }
    if top_plane not in planes:
        raise RuntimeError(f"Unsupported top plane: {top_plane}")
    return planes[top_plane]


def _plot_top_map(
    path: Path, points: np.ndarray, trajectory: np.ndarray | None, top_plane: str
) -> None:
    axis_a, axis_b, label_a, label_b = _plane_indices(top_plane)
    xy = points[:, [axis_a, axis_b]]
    finite = np.isfinite(xy).all(axis=1)
    xy = xy[finite]
    if len(xy) == 0:
        raise RuntimeError(f"No finite {top_plane.upper()} points in cloud")

    if len(xy) > 250000:
        step = max(1, len(xy) // 250000)
        xy = xy[::step]

    fig, ax = plt.subplots(figsize=(10, 10), dpi=180)
    ax.scatter(xy[:, 0], xy[:, 1], s=0.15, c="#222222", alpha=0.55, linewidths=0)

    if trajectory is not None and len(trajectory) > 0:
        traj_xy = trajectory[:, [axis_a, axis_b]]
        traj_xy = traj_xy[np.isfinite(traj_xy).all(axis=1)]
        if len(traj_xy) > 0:
            ax.plot(traj_xy[:, 0], traj_xy[:, 1], c="#0077ff", linewidth=2.0, alpha=0.95)
            ax.scatter(
                traj_xy[0, 0],
                traj_xy[0, 1],
                s=90,
                c="#1a9c45",
                edgecolors="white",
                linewidths=1.4,
                label="Start",
                zorder=4,
            )
            ax.scatter(
                traj_xy[-1, 0],
                traj_xy[-1, 1],
                s=90,
                c="#d62728",
                edgecolors="white",
                linewidths=1.4,
                label="End",
                zorder=4,
            )
            ax.legend(loc="upper right", frameon=True)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel(f"{label_a} [m]")
    ax.set_ylabel(f"{label_b} [m]")
    ax.grid(True, color="#d8d8d8", linewidth=0.5)
    ax.set_title(f"{path.stem} ({top_plane.upper()} top-view)")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _read_ascii_pcd(path: Path) -> tuple[np.ndarray, np.ndarray | None]:
    fields: list[str] = []
    rows: list[list[float]] = []
    with path.open("r", encoding="ascii") as f:
        in_data = False
        for line in f:
            line = line.strip()
            if not line:
                continue
            if in_data:
                rows.append([float(value) for value in line.split()])
                continue
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("DATA"):
                if line != "DATA ascii":
                    raise RuntimeError(f"Only ASCII PCD is supported for plot-only mode: {path}")
                in_data = True
    if not rows:
        raise RuntimeError(f"No point rows found in {path}")
    data = np.asarray(rows, dtype=np.float32)
    x_idx, y_idx, z_idx = (fields.index(name) for name in ("x", "y", "z"))
    points = data[:, [x_idx, y_idx, z_idx]]
    return points, None


def _load_db_trajectory(db_path: Path) -> list[tuple[float, float, float]]:
    if not db_path.exists():
        raise RuntimeError(f"trajectory DB does not exist: {db_path}")
    trajectory: list[tuple[float, float, float]] = []
    con = sqlite3.connect(str(db_path))
    try:
        for _, pose_blob in con.execute(
            "select id, pose from Node where pose is not null order by id"
        ):
            if pose_blob is None or len(pose_blob) < 48:
                continue
            pose = struct.unpack("12f", pose_blob[:48])
            trajectory.append((float(pose[3]), float(pose[7]), float(pose[11])))
    finally:
        con.close()
    return trajectory


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--map-topic", default="/rtabmap/cloud_map")
    parser.add_argument("--pose-topic", default="/rtabmap/pose")
    parser.add_argument("--trajectory-db", default="")
    parser.add_argument("--input-pcd", default="")
    parser.add_argument("--top-plane", choices=("xy", "xz", "yz"), default="xz")
    parser.add_argument("--idle-sec", type=float, default=20.0)
    parser.add_argument("--min-run-sec", type=float, default=30.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.input_pcd:
        output_dir = Path(args.output_dir).expanduser()
        output_dir.mkdir(parents=True, exist_ok=True)
        points, _ = _read_ascii_pcd(Path(args.input_pcd).expanduser())
        trajectory = (
            np.asarray(_load_db_trajectory(Path(args.trajectory_db).expanduser()), dtype=np.float32)
            if args.trajectory_db
            else np.empty((0, 3), dtype=np.float32)
        )
        _plot_top_map(output_dir / "rtabmap_2d_top_map.png", points, None, args.top_plane)
        _plot_top_map(
            output_dir / "rtabmap_2d_top_map_trajectory_start_end.png",
            points,
            trajectory,
            args.top_plane,
        )
        return

    rclpy.init()
    node = RtabmapOutputExporter(args)
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.save_outputs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
