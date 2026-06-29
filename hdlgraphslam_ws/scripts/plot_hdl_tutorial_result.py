#!/usr/bin/env python3
"""Render HDL Graph SLAM tutorial bag results as top-down PNGs."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from matplotlib.collections import LineCollection
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2


@dataclass
class TimedPose:
    stamp_ns: int
    matrix: np.ndarray


@dataclass
class TimedCloud:
    stamp_ns: int
    points: np.ndarray


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def msg_stamp_ns(msg, fallback_ns: int) -> int:
    header = getattr(msg, "header", None)
    if header is None:
        return fallback_ns
    return stamp_to_ns(header.stamp)


def normalize_quat(q: Sequence[float]) -> np.ndarray:
    quat = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quat / norm


def quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    x, y, z, w = normalize_quat((qx, qy, qz, qw))
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def pose_to_matrix(position, orientation) -> np.ndarray:
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = quat_to_matrix(
        orientation.x, orientation.y, orientation.z, orientation.w
    )
    mat[:3, 3] = [position.x, position.y, position.z]
    return mat


def transform_to_matrix(transform) -> np.ndarray:
    return pose_to_matrix(transform.translation, transform.rotation)


def matrix_to_xyz_quat(mat: np.ndarray) -> Tuple[float, float, float, float, float, float, float]:
    rot = mat[:3, :3]
    trace = np.trace(rot)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rot[2, 1] - rot[1, 2]) / s
        qy = (rot[0, 2] - rot[2, 0]) / s
        qz = (rot[1, 0] - rot[0, 1]) / s
    else:
        idx = int(np.argmax(np.diag(rot)))
        if idx == 0:
            s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
            qw = (rot[2, 1] - rot[1, 2]) / s
            qx = 0.25 * s
            qy = (rot[0, 1] + rot[1, 0]) / s
            qz = (rot[0, 2] + rot[2, 0]) / s
        elif idx == 1:
            s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
            qw = (rot[0, 2] - rot[2, 0]) / s
            qx = (rot[0, 1] + rot[1, 0]) / s
            qy = 0.25 * s
            qz = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
            qw = (rot[1, 0] - rot[0, 1]) / s
            qx = (rot[0, 2] + rot[2, 0]) / s
            qy = (rot[1, 2] + rot[2, 1]) / s
            qz = 0.25 * s
    qx, qy, qz, qw = normalize_quat((qx, qy, qz, qw))
    x, y, z = mat[:3, 3]
    return float(x), float(y), float(z), float(qx), float(qy), float(qz), float(qw)


def open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    if not (bag_path / "metadata.yaml").exists():
        raise FileNotFoundError(f"missing ROS 2 bag metadata: {bag_path / 'metadata.yaml'}")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def read_cloud_points(msg, max_points: int, point_step: int) -> np.ndarray:
    pts = point_cloud2.read_points(
        msg, field_names=("x", "y", "z"), skip_nans=True
    )
    arr = np.asarray(pts)
    if arr.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    if arr.dtype.fields:
        arr = np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float32)
    else:
        arr = np.asarray(list(pts), dtype=np.float32).reshape((-1, 3))
    if point_step > 1:
        arr = arr[::point_step]
    if max_points > 0 and len(arr) > max_points:
        idx = np.linspace(0, len(arr) - 1, max_points, dtype=np.int64)
        arr = arr[idx]
    return arr


def read_bag(
    bag_path: Path,
    odom_topic: str,
    tf_topic: str,
    lidar_topic: str,
    native_map_topic: str,
    gt_pose_topic: str,
    max_points_per_scan: int,
    point_step: int,
) -> Tuple[List[TimedPose], List[TimedPose], List[Tuple[int, np.ndarray]], List[TimedCloud], List[TimedCloud], Dict[str, int]]:
    reader = open_reader(bag_path)
    type_map = {item.name: item.type for item in reader.get_all_topics_and_types()}
    type_cache: Dict[str, object] = {}
    wanted = {odom_topic, tf_topic, lidar_topic, native_map_topic, gt_pose_topic}
    counts = {
        odom_topic: 0,
        tf_topic: 0,
        lidar_topic: 0,
        native_map_topic: 0,
        gt_pose_topic: 0,
    }
    odom_poses: List[TimedPose] = []
    gt_poses: List[TimedPose] = []
    map_odom_tfs: List[Tuple[int, np.ndarray]] = []
    lidar_clouds: List[TimedCloud] = []
    native_clouds: List[TimedCloud] = []

    while reader.has_next():
        topic_name, data, bag_stamp_ns = reader.read_next()
        if topic_name not in wanted:
            continue
        if topic_name not in type_cache:
            type_cache[topic_name] = get_message(type_map[topic_name])
        msg = deserialize_message(data, type_cache[topic_name])
        counts[topic_name] += 1

        if topic_name == odom_topic:
            stamp_ns = msg_stamp_ns(msg, bag_stamp_ns)
            odom_poses.append(
                TimedPose(
                    stamp_ns,
                    pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation),
                )
            )
        elif topic_name == tf_topic:
            for transform in msg.transforms:
                parent = transform.header.frame_id.lstrip("/")
                child = transform.child_frame_id.lstrip("/")
                if parent == "map" and child == "odom":
                    map_odom_tfs.append(
                        (stamp_to_ns(transform.header.stamp), transform_to_matrix(transform.transform))
                    )
                elif parent == "odom" and child == "map":
                    map_odom_tfs.append(
                        (stamp_to_ns(transform.header.stamp), np.linalg.inv(transform_to_matrix(transform.transform)))
                    )
        elif topic_name == gt_pose_topic:
            gt_poses.append(
                TimedPose(
                    msg_stamp_ns(msg, bag_stamp_ns),
                    pose_to_matrix(msg.pose.position, msg.pose.orientation),
                )
            )
        elif topic_name == lidar_topic:
            points = read_cloud_points(msg, max_points_per_scan, point_step)
            if len(points):
                lidar_clouds.append(TimedCloud(msg_stamp_ns(msg, bag_stamp_ns), points))
        elif topic_name == native_map_topic:
            points = read_cloud_points(msg, max_points_per_scan, point_step)
            if len(points):
                native_clouds.append(TimedCloud(msg_stamp_ns(msg, bag_stamp_ns), points))

    odom_poses.sort(key=lambda item: item.stamp_ns)
    gt_poses.sort(key=lambda item: item.stamp_ns)
    map_odom_tfs.sort(key=lambda item: item[0])
    lidar_clouds.sort(key=lambda item: item.stamp_ns)
    native_clouds.sort(key=lambda item: item.stamp_ns)
    return odom_poses, gt_poses, map_odom_tfs, native_clouds, lidar_clouds, counts


def nearest_index(stamps: Sequence[int], stamp_ns: int) -> Tuple[int, int]:
    idx = bisect.bisect_left(stamps, stamp_ns)
    if idx == 0:
        best = 0
    elif idx == len(stamps):
        best = len(stamps) - 1
    else:
        before = stamps[idx - 1]
        after = stamps[idx]
        best = idx - 1 if abs(stamp_ns - before) <= abs(after - stamp_ns) else idx
    return best, abs(stamps[best] - stamp_ns)


def apply_map_to_odom(
    odom_poses: Sequence[TimedPose], map_odom_tfs: Sequence[Tuple[int, np.ndarray]]
) -> Tuple[List[TimedPose], float | None]:
    if not odom_poses:
        return [], None
    if not map_odom_tfs:
        return list(odom_poses), None
    tf_stamps = [item[0] for item in map_odom_tfs]
    poses: List[TimedPose] = []
    max_dt_ns = 0
    for pose in odom_poses:
        idx, dt_ns = nearest_index(tf_stamps, pose.stamp_ns)
        max_dt_ns = max(max_dt_ns, dt_ns)
        poses.append(TimedPose(pose.stamp_ns, map_odom_tfs[idx][1] @ pose.matrix))
    return poses, max_dt_ns / 1e9


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if len(points) == 0 or voxel_size <= 0.0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(unique_idx)]


def accumulate_lidar_map(
    clouds: Sequence[TimedCloud],
    poses: Sequence[TimedPose],
    voxel_size: float,
    max_match_dt: float,
) -> Tuple[np.ndarray, Dict[str, float]]:
    if not clouds or not poses:
        return np.empty((0, 3), dtype=np.float32), {
            "used_scans": 0,
            "dropped_scans": float(len(clouds)),
        }
    pose_stamps = [pose.stamp_ns for pose in poses]
    max_dt_allowed_ns = int(max_match_dt * 1e9) if max_match_dt > 0 else None
    max_seen_dt_ns = 0
    dropped = 0
    acc: List[np.ndarray] = []
    for cloud in clouds:
        idx, dt_ns = nearest_index(pose_stamps, cloud.stamp_ns)
        max_seen_dt_ns = max(max_seen_dt_ns, dt_ns)
        if max_dt_allowed_ns is not None and dt_ns > max_dt_allowed_ns:
            dropped += 1
            continue
        pose = poses[idx].matrix
        xyz = (cloud.points.astype(np.float64) @ pose[:3, :3].T) + pose[:3, 3]
        acc.append(xyz.astype(np.float32))
    points = np.concatenate(acc, axis=0) if acc else np.empty((0, 3), dtype=np.float32)
    return voxel_downsample(points, voxel_size), {
        "used_scans": float(len(acc)),
        "dropped_scans": float(dropped),
        "max_lidar_pose_dt_sec": max_seen_dt_ns / 1e9,
    }


def write_pcd(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for x, y, z in points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def write_trajectory_csv(path: Path, poses: Sequence[TimedPose]) -> None:
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_ns", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for pose in poses:
            writer.writerow([pose.stamp_ns, *matrix_to_xyz_quat(pose.matrix)])


def trajectory_positions(poses: Sequence[TimedPose]) -> np.ndarray:
    if not poses:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray([pose.matrix[:3, 3] for pose in poses], dtype=np.float32)


def add_gradient_trajectory(ax, positions: np.ndarray, label: str) -> None:
    if len(positions) < 2:
        return
    xy = positions[:, :2]
    segments = np.stack([xy[:-1], xy[1:]], axis=1)
    collection = LineCollection(segments, cmap="coolwarm_r", linewidths=1.8)
    collection.set_array(np.linspace(0.0, 1.0, len(segments)))
    ax.add_collection(collection)
    ax.scatter(xy[0, 0], xy[0, 1], c="red", s=28, label=f"{label} start", zorder=3)
    ax.scatter(xy[-1, 0], xy[-1, 1], c="blue", s=28, label=f"{label} end", zorder=3)


def plot_plain_trajectory(ax, poses: Sequence[TimedPose], color: str, label: str) -> np.ndarray:
    positions = trajectory_positions(poses)
    if len(positions) >= 2:
        ax.plot(positions[:, 0], positions[:, 1], color=color, linewidth=1.8, label=label)
        ax.scatter(positions[0, 0], positions[0, 1], c=color, s=24, marker="o", label=f"{label} start")
        ax.scatter(positions[-1, 0], positions[-1, 1], c=color, s=32, marker="x", label=f"{label} end")
    return positions


def align_gt_to_hdl(
    gt_poses: Sequence[TimedPose],
    hdl_poses: Sequence[TimedPose],
    max_match_dt: float,
) -> Tuple[List[TimedPose], dict[str, float]]:
    if len(gt_poses) < 2 or len(hdl_poses) < 2:
        return [], {"matched_pairs": 0.0}
    gt_stamps = [pose.stamp_ns for pose in gt_poses]
    max_dt_ns = int(max_match_dt * 1e9)
    src = []
    dst = []
    max_seen_dt_ns = 0
    for hdl_pose in hdl_poses:
        idx, dt_ns = nearest_index(gt_stamps, hdl_pose.stamp_ns)
        max_seen_dt_ns = max(max_seen_dt_ns, dt_ns)
        if max_match_dt > 0 and dt_ns > max_dt_ns:
            continue
        src.append(gt_poses[idx].matrix[:2, 3])
        dst.append(hdl_pose.matrix[:2, 3])
    if len(src) < 2:
        return [], {"matched_pairs": float(len(src)), "max_seen_dt_sec": max_seen_dt_ns / 1e9}

    src_xy = np.asarray(src, dtype=np.float64)
    dst_xy = np.asarray(dst, dtype=np.float64)
    src_center = src_xy.mean(axis=0)
    dst_center = dst_xy.mean(axis=0)
    src_centered = src_xy - src_center
    dst_centered = dst_xy - dst_center
    u, _, vt = np.linalg.svd(src_centered.T @ dst_centered)
    rot = u @ vt
    if np.linalg.det(rot) < 0:
        u[:, -1] *= -1
        rot = u @ vt
    trans = dst_center - src_center @ rot

    aligned = []
    for pose in gt_poses:
        mat = pose.matrix.copy()
        mat[:2, 3] = pose.matrix[:2, 3] @ rot + trans
        aligned.append(TimedPose(pose.stamp_ns, mat))

    residual = src_xy @ rot + trans - dst_xy
    rmse = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    return aligned, {
        "matched_pairs": float(len(src)),
        "max_seen_dt_sec": max_seen_dt_ns / 1e9,
        "xy_rmse_after_alignment_m": rmse,
    }


def set_equal_limits(ax, arrays: Iterable[np.ndarray], percentile: float | None = None) -> None:
    pts = [arr[:, :2] for arr in arrays if len(arr)]
    if not pts:
        return
    xy = np.concatenate(pts, axis=0)
    if percentile is not None and 0.0 < percentile < 100.0 and len(xy) >= 20:
        tail = (100.0 - percentile) / 2.0
        mins = np.percentile(xy, tail, axis=0)
        maxs = np.percentile(xy, 100.0 - tail, axis=0)
    else:
        mins = xy.min(axis=0)
        maxs = xy.max(axis=0)
    center = (mins + maxs) / 2.0
    span = max(float(np.max(maxs - mins)), 1.0)
    pad = span * 0.06
    ax.set_xlim(center[0] - span / 2.0 - pad, center[0] + span / 2.0 + pad)
    ax.set_ylim(center[1] - span / 2.0 - pad, center[1] + span / 2.0 + pad)


def plot_map_overlay(
    path: Path,
    points: np.ndarray,
    poses: Sequence[TimedPose],
    title: str,
    limit_percentile: float | None = None,
) -> None:
    fig, ax = plt.subplots(figsize=(10, 10), dpi=180)
    if len(points):
        stride = max(1, len(points) // 350_000)
        ax.scatter(
            points[::stride, 0],
            points[::stride, 1],
            s=0.08,
            c="0.25",
            alpha=0.5,
            rasterized=True,
        )
    traj = trajectory_positions(poses)
    add_gradient_trajectory(ax, traj, "trajectory")
    set_equal_limits(ax, [points, traj], percentile=limit_percentile)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, linewidth=0.3, alpha=0.25)
    ax.legend(loc="best", markerscale=1.5)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_map_with_gt_overlay(
    path: Path,
    points: np.ndarray,
    hdl_poses: Sequence[TimedPose],
    gt_poses: Sequence[TimedPose],
    title: str,
    limit_percentile: float | None = None,
) -> None:
    fig, ax = plt.subplots(figsize=(10, 10), dpi=180)
    if len(points):
        stride = max(1, len(points) // 350_000)
        ax.scatter(
            points[::stride, 0],
            points[::stride, 1],
            s=0.08,
            c="0.25",
            alpha=0.5,
            rasterized=True,
            label="map",
        )
    hdl = plot_plain_trajectory(ax, hdl_poses, "#1f77b4", "HDL")
    gt = plot_plain_trajectory(ax, gt_poses, "#d62728", "GT aligned")
    set_equal_limits(ax, [points, hdl, gt], percentile=limit_percentile)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, linewidth=0.3, alpha=0.25)
    ax.legend(loc="best", markerscale=1.5)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def select_final_map_points(
    map_source: str,
    native_clouds: Sequence[TimedCloud],
    lidar_clouds: Sequence[TimedCloud],
    poses: Sequence[TimedPose],
    voxel_size: float,
    max_pose_match_dt: float,
) -> Tuple[np.ndarray, Dict[str, object]]:
    if map_source in {"native", "auto"} and native_clouds:
        points = voxel_downsample(native_clouds[-1].points, voxel_size)
        return points, {
            "source": "native",
            "point_count": int(len(points)),
            "source_stamp_ns": native_clouds[-1].stamp_ns,
        }
    if map_source in {"lidar", "auto"} and lidar_clouds:
        points, stats = accumulate_lidar_map(
            lidar_clouds, poses, voxel_size, max_pose_match_dt
        )
        return points, {
            "source": "lidar",
            "point_count": int(len(points)),
            **stats,
        }
    return np.empty((0, 3), dtype=np.float32), {
        "source": "none",
        "point_count": 0,
    }


def write_final_png(
    path: Path,
    points: np.ndarray,
    poses: Sequence[TimedPose],
    aligned_gt_poses: Sequence[TimedPose],
    missing_gt_policy: str,
    limit_percentile: float | None = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if aligned_gt_poses:
        plot_map_with_gt_overlay(
            path,
            points,
            poses,
            aligned_gt_poses,
            "HDL map with HDL and GT trajectories",
            limit_percentile=limit_percentile,
        )
    elif missing_gt_policy == "fail":
        raise RuntimeError("GT poses were requested but no aligned GT trajectory is available")
    else:
        plot_map_overlay(
            path,
            points,
            poses,
            "HDL map with HDL trajectory",
            limit_percentile=limit_percentile,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="Recorded ROS 2 bag directory")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--lidar-topic", default="/filtered_points")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--native-map-topic", default="/hdl_graph_slam/map_points")
    parser.add_argument("--gt-pose-topic", default="/vrpn_client_node/UWBTest/pose")
    parser.add_argument("--voxel-size", type=float, default=0.05)
    parser.add_argument("--point-step", type=int, default=5)
    parser.add_argument("--max-points-per-scan", type=int, default=20_000)
    parser.add_argument("--max-pose-match-dt", type=float, default=1.0)
    parser.add_argument(
        "--zoom-percentile",
        type=float,
        default=99.0,
        help="Percentile limits for additional zoom PNGs; use <=0 to disable.",
    )
    parser.add_argument("--max-gt-match-dt", type=float, default=0.2)
    parser.add_argument(
        "--final-png",
        default="",
        help="Write this final overlay PNG and skip the legacy diagnostic PNG set.",
    )
    parser.add_argument(
        "--final-zoom-png",
        default="",
        help="Optional zoomed final overlay PNG written from the same data as --final-png.",
    )
    parser.add_argument(
        "--final-zoom-percentile",
        type=float,
        default=99.0,
        help="Percentile limits for --final-zoom-png; use <=0 to disable zoom limiting.",
    )
    parser.add_argument(
        "--map-source",
        choices=("native", "lidar", "auto"),
        default="auto",
        help="Map source for --final-png: native map_points, accumulated lidar, or auto fallback.",
    )
    parser.add_argument(
        "--missing-gt-policy",
        choices=("hdl_only", "fail"),
        default="hdl_only",
        help="Behavior when --final-png is requested and GT is missing or cannot be aligned.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bag_path = Path(args.bag).expanduser()
    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    odom_poses, gt_poses, map_odom_tfs, native_clouds, lidar_clouds, counts = read_bag(
        bag_path,
        args.odom_topic,
        args.tf_topic,
        args.lidar_topic,
        args.native_map_topic,
        args.gt_pose_topic,
        args.max_points_per_scan,
        max(1, args.point_step),
    )
    poses, max_odom_tf_dt = apply_map_to_odom(odom_poses, map_odom_tfs)
    if not poses:
        raise RuntimeError(f"missing required odom poses: {args.odom_topic}")

    write_trajectory_csv(output_dir / "hdl_estimated_trajectory.csv", poses)
    summary = {
        "bag": str(bag_path),
        "topic_counts": counts,
        "pose_count": len(poses),
        "map_to_odom_tf_count": len(map_odom_tfs),
        "max_odom_tf_dt_sec": max_odom_tf_dt,
        "used_identity_map_to_odom": len(map_odom_tfs) == 0,
        "voxel_size": args.voxel_size,
        "point_step": max(1, args.point_step),
        "max_points_per_scan": args.max_points_per_scan,
    }

    aligned_gt_poses, gt_alignment = align_gt_to_hdl(gt_poses, poses, args.max_gt_match_dt)
    if aligned_gt_poses:
        write_trajectory_csv(output_dir / "gt_aligned_trajectory.csv", aligned_gt_poses)
        summary["gt_alignment"] = gt_alignment

    if args.final_png:
        final_points, final_map_stats = select_final_map_points(
            args.map_source,
            native_clouds,
            lidar_clouds,
            poses,
            args.voxel_size,
            args.max_pose_match_dt,
        )
        final_png = Path(args.final_png).expanduser()
        write_final_png(
            final_png,
            final_points,
            poses,
            aligned_gt_poses,
            args.missing_gt_policy,
        )
        summary["final_png"] = str(final_png)
        if args.final_zoom_png:
            final_zoom_png = Path(args.final_zoom_png).expanduser()
            zoom_percentile = args.final_zoom_percentile
            write_final_png(
                final_zoom_png,
                final_points,
                poses,
                aligned_gt_poses,
                args.missing_gt_policy,
                limit_percentile=zoom_percentile if zoom_percentile > 0 else None,
            )
            summary["final_zoom_png"] = str(final_zoom_png)
            summary["final_zoom_percentile"] = zoom_percentile
        summary["final_map"] = final_map_stats
        summary["gt_available"] = bool(aligned_gt_poses)
        with (output_dir / "tutorial_result_summary.json").open("w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, sort_keys=True)
            f.write("\n")
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0

    if native_clouds:
        native_points = voxel_downsample(native_clouds[-1].points, args.voxel_size)
        write_pcd(output_dir / "hdl_native_map_points.pcd", native_points)
        plot_map_overlay(
            output_dir / "hdl_native_map_points_overlay.png",
            native_points,
            poses,
            "HDL native map_points with HDL trajectory",
        )
        if args.zoom_percentile > 0:
            plot_map_overlay(
                output_dir / "hdl_native_map_points_overlay_zoom.png",
                native_points,
                poses,
                "HDL native map_points with HDL trajectory (zoom)",
                limit_percentile=args.zoom_percentile,
            )
            if aligned_gt_poses:
                plot_map_with_gt_overlay(
                    output_dir / "hdl_native_map_points_hdl_gt_overlay_zoom.png",
                    native_points,
                    poses,
                    aligned_gt_poses,
                    "HDL native map_points with HDL and GT trajectories (zoom)",
                    limit_percentile=args.zoom_percentile,
                )
        summary["hdl_native_map_points"] = {
            "point_count": int(len(native_points)),
            "source_stamp_ns": native_clouds[-1].stamp_ns,
        }

    if lidar_clouds:
        estimated_map, map_stats = accumulate_lidar_map(
            lidar_clouds, poses, args.voxel_size, args.max_pose_match_dt
        )
        write_pcd(output_dir / "hdl_estimated_lidar_map.pcd", estimated_map)
        plot_map_overlay(
            output_dir / "hdl_estimated_lidar_map_trajectory_overlay.png",
            estimated_map,
            poses,
            "HDL estimated LiDAR map with trajectory",
        )
        if args.zoom_percentile > 0:
            plot_map_overlay(
                output_dir / "hdl_estimated_lidar_map_trajectory_overlay_zoom.png",
                estimated_map,
                poses,
                "HDL estimated LiDAR map with trajectory (zoom)",
                limit_percentile=args.zoom_percentile,
            )
            if aligned_gt_poses:
                plot_map_with_gt_overlay(
                    output_dir / "hdl_estimated_lidar_map_hdl_gt_overlay_zoom.png",
                    estimated_map,
                    poses,
                    aligned_gt_poses,
                    "HDL estimated LiDAR map with HDL and GT trajectories (zoom)",
                    limit_percentile=args.zoom_percentile,
                )
        summary["hdl_estimated_lidar_map"] = {
            "point_count": int(len(estimated_map)),
            **map_stats,
        }

    with (output_dir / "tutorial_result_summary.json").open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, sort_keys=True)
        f.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
