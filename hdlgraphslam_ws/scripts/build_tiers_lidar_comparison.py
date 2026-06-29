#!/usr/bin/env python3
"""Build TIERS HDL-vs-GT LiDAR comparison maps from ROS 2 bags.

The comparison keeps the PointCloud2 input fixed and changes only the pose
stream used to accumulate the scans:
  * HDL map: /filtered_points accumulated with /odom transformed by map->odom.
  * GT map:  /filtered_points accumulated with /vrpn_client_node/UWBTest/pose.

The script accepts one combined recorded bag or multiple bags. Multiple bags are
useful when a prior HDL run recorded /odom and /tf separately from the source
bag containing LiDAR and GT pose.
"""

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


DEFAULT_BAG = (
    "/home/etri/hdl_graph_slam_humble_clone/output/tiers/indoor03/hdl_recorded_bag"
)
DEFAULT_OUTPUT_DIR = "/home/etri/hdl_graph_slam_humble_clone/output/tiers/indoor03"


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
    if header is not None:
        return stamp_to_ns(header.stamp)
    return fallback_ns


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
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)
    return reader


def read_cloud_points(msg, max_points_per_scan: int, point_step: int) -> np.ndarray:
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
    if max_points_per_scan > 0 and len(arr) > max_points_per_scan:
        idx = np.linspace(0, len(arr) - 1, max_points_per_scan, dtype=np.int64)
        arr = arr[idx]
    return arr


def read_bags(
    bag_paths: Sequence[Path],
    topics: Dict[str, str],
    max_points_per_scan: int,
    point_step: int,
) -> Tuple[List[TimedPose], List[TimedPose], List[Tuple[int, np.ndarray]], List[TimedCloud], Dict[str, int]]:
    type_cache: Dict[str, object] = {}
    topic_counts = {name: 0 for name in topics}
    odom_poses: List[TimedPose] = []
    gt_poses: List[TimedPose] = []
    map_odom_tfs: List[Tuple[int, np.ndarray]] = []
    lidar_clouds: List[TimedCloud] = []
    native_clouds: List[TimedCloud] = []

    wanted = set(topics.values())
    for bag_path in bag_paths:
        reader = open_reader(bag_path)
        type_map = {
            item.name: item.type for item in reader.get_all_topics_and_types()
        }
        while reader.has_next():
            topic_name, data, bag_stamp_ns = reader.read_next()
            if topic_name not in wanted:
                continue
            if topic_name not in type_cache:
                type_cache[topic_name] = get_message(type_map[topic_name])
            msg = deserialize_message(data, type_cache[topic_name])

            if topic_name == topics["odom"]:
                stamp_ns = msg_stamp_ns(msg, bag_stamp_ns)
                odom_poses.append(TimedPose(stamp_ns, pose_to_matrix(
                    msg.pose.pose.position, msg.pose.pose.orientation
                )))
                topic_counts["odom"] += 1
            elif topic_name == topics["gt_pose"]:
                stamp_ns = msg_stamp_ns(msg, bag_stamp_ns)
                gt_poses.append(TimedPose(stamp_ns, pose_to_matrix(
                    msg.pose.position, msg.pose.orientation
                )))
                topic_counts["gt_pose"] += 1
            elif topic_name == topics["tf"]:
                for transform in msg.transforms:
                    parent = transform.header.frame_id.lstrip("/")
                    child = transform.child_frame_id.lstrip("/")
                    if parent == "map" and child == "odom":
                        map_odom_tfs.append(
                            (stamp_to_ns(transform.header.stamp), transform_to_matrix(transform.transform))
                        )
                        topic_counts["tf"] += 1
                    elif parent == "odom" and child == "map":
                        map_odom_tfs.append(
                            (stamp_to_ns(transform.header.stamp), np.linalg.inv(transform_to_matrix(transform.transform)))
                        )
                        topic_counts["tf"] += 1
            elif topic_name == topics["lidar"]:
                stamp_ns = msg_stamp_ns(msg, bag_stamp_ns)
                points = read_cloud_points(msg, max_points_per_scan, point_step)
                if len(points):
                    lidar_clouds.append(TimedCloud(stamp_ns, points))
                topic_counts["lidar"] += 1
            elif topic_name == topics["native_map"]:
                stamp_ns = msg_stamp_ns(msg, bag_stamp_ns)
                points = read_cloud_points(msg, max_points_per_scan, point_step)
                if len(points):
                    native_clouds.append(TimedCloud(stamp_ns, points))
                topic_counts["native_map"] += 1

    odom_poses.sort(key=lambda item: item.stamp_ns)
    gt_poses.sort(key=lambda item: item.stamp_ns)
    map_odom_tfs.sort(key=lambda item: item[0])
    lidar_clouds.sort(key=lambda item: item.stamp_ns)
    native_clouds.sort(key=lambda item: item.stamp_ns)
    return odom_poses, gt_poses, map_odom_tfs, native_clouds, lidar_clouds, topic_counts


def nearest_index(stamps: Sequence[int], stamp_ns: int) -> Tuple[int, int]:
    if not stamps:
        raise ValueError("empty stamp sequence")
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
) -> Tuple[List[TimedPose], Optional[float]]:
    if not odom_poses:
        return [], None
    if not map_odom_tfs:
        return list(odom_poses), None
    tf_stamps = [item[0] for item in map_odom_tfs]
    hdl_map_poses: List[TimedPose] = []
    max_dt_ns = 0
    for pose in odom_poses:
        idx, dt_ns = nearest_index(tf_stamps, pose.stamp_ns)
        max_dt_ns = max(max_dt_ns, dt_ns)
        hdl_map_poses.append(
            TimedPose(pose.stamp_ns, map_odom_tfs[idx][1] @ pose.matrix)
        )
    return hdl_map_poses, max_dt_ns / 1e9


def accumulate_lidar_map(
    clouds: Sequence[TimedCloud],
    poses: Sequence[TimedPose],
    voxel_size: float,
) -> Tuple[np.ndarray, float]:
    if not clouds or not poses:
        return np.empty((0, 3), dtype=np.float32), 0.0
    pose_stamps = [pose.stamp_ns for pose in poses]
    acc: List[np.ndarray] = []
    max_dt_ns = 0
    for cloud in clouds:
        idx, dt_ns = nearest_index(pose_stamps, cloud.stamp_ns)
        max_dt_ns = max(max_dt_ns, dt_ns)
        pose = poses[idx].matrix
        xyz = (cloud.points.astype(np.float64) @ pose[:3, :3].T) + pose[:3, 3]
        acc.append(xyz.astype(np.float32))
    points = np.concatenate(acc, axis=0) if acc else np.empty((0, 3), dtype=np.float32)
    return voxel_downsample(points, voxel_size), max_dt_ns / 1e9


def filter_clouds_by_pose_match(
    clouds: Sequence[TimedCloud],
    pose_sets: Sequence[Sequence[TimedPose]],
    max_match_dt_sec: float,
) -> Tuple[List[TimedCloud], Dict[str, float]]:
    if max_match_dt_sec <= 0.0:
        return list(clouds), {"kept": float(len(clouds)), "dropped": 0.0}
    stamp_sets = [[pose.stamp_ns for pose in poses] for poses in pose_sets]
    max_dt_ns_allowed = int(max_match_dt_sec * 1e9)
    kept: List[TimedCloud] = []
    max_seen_dt_ns = 0
    dropped = 0
    for cloud in clouds:
        keep = True
        for stamps in stamp_sets:
            if not stamps:
                keep = False
                break
            _, dt_ns = nearest_index(stamps, cloud.stamp_ns)
            max_seen_dt_ns = max(max_seen_dt_ns, dt_ns)
            if dt_ns > max_dt_ns_allowed:
                keep = False
                break
        if keep:
            kept.append(cloud)
        else:
            dropped += 1
    return kept, {
        "kept": float(len(kept)),
        "dropped": float(dropped),
        "max_seen_dt_sec": max_seen_dt_ns / 1e9,
        "threshold_sec": max_match_dt_sec,
    }


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if len(points) == 0 or voxel_size <= 0.0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(unique_idx)]


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
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_ns", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for pose in poses:
            writer.writerow([pose.stamp_ns, *matrix_to_xyz_quat(pose.matrix)])


def trajectory_positions(poses: Sequence[TimedPose]) -> np.ndarray:
    if not poses:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray([pose.matrix[:3, 3] for pose in poses], dtype=np.float32)


def add_gradient_trajectory(ax, positions: np.ndarray, label: str, linewidth: float = 1.8) -> None:
    if len(positions) < 2:
        return
    xy = positions[:, :2]
    segments = np.stack([xy[:-1], xy[1:]], axis=1)
    colors = np.linspace(0.0, 1.0, len(segments))
    collection = LineCollection(segments, cmap="coolwarm_r", linewidths=linewidth)
    collection.set_array(colors)
    ax.add_collection(collection)
    ax.scatter(xy[0, 0], xy[0, 1], c="red", s=28, label=f"{label} start", zorder=3)
    ax.scatter(xy[-1, 0], xy[-1, 1], c="blue", s=28, label=f"{label} end", zorder=3)


def set_equal_limits(ax, arrays: Iterable[np.ndarray]) -> None:
    pts = [arr[:, :2] for arr in arrays if len(arr)]
    if not pts:
        return
    xy = np.concatenate(pts, axis=0)
    mins = xy.min(axis=0)
    maxs = xy.max(axis=0)
    center = (mins + maxs) / 2.0
    span = max(float(np.max(maxs - mins)), 1.0)
    pad = span * 0.06
    ax.set_xlim(center[0] - span / 2.0 - pad, center[0] + span / 2.0 + pad)
    ax.set_ylim(center[1] - span / 2.0 - pad, center[1] + span / 2.0 + pad)


def plot_map_overlay(path: Path, points: np.ndarray, poses: Sequence[TimedPose], title: str) -> None:
    fig, ax = plt.subplots(figsize=(10, 10), dpi=180)
    if len(points):
        stride = max(1, len(points) // 350_000)
        ax.scatter(points[::stride, 0], points[::stride, 1], s=0.08, c="0.25", alpha=0.5, rasterized=True)
    traj = trajectory_positions(poses)
    add_gradient_trajectory(ax, traj, "trajectory")
    set_equal_limits(ax, [points, traj])
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, linewidth=0.3, alpha=0.25)
    ax.legend(loc="best", markerscale=1.5)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_trajectory_overlay(
    path: Path, hdl_poses: Sequence[TimedPose], gt_poses: Sequence[TimedPose]
) -> None:
    fig, ax = plt.subplots(figsize=(10, 10), dpi=180)
    hdl = trajectory_positions(hdl_poses)
    gt = trajectory_positions(gt_poses)
    if len(hdl) >= 2:
        ax.plot(hdl[:, 0], hdl[:, 1], color="#d95f02", linewidth=1.8, label="HDL")
        ax.scatter(hdl[0, 0], hdl[0, 1], c="red", s=24, label="HDL start")
        ax.scatter(hdl[-1, 0], hdl[-1, 1], c="blue", s=24, label="HDL end")
    if len(gt) >= 2:
        ax.plot(gt[:, 0], gt[:, 1], color="#1b9e77", linewidth=1.8, label="GT")
        ax.scatter(gt[0, 0], gt[0, 1], c="red", marker="x", s=30, label="GT start")
        ax.scatter(gt[-1, 0], gt[-1, 1], c="blue", marker="x", s=30, label="GT end")
    set_equal_limits(ax, [hdl, gt])
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("HDL vs GT trajectory overlay")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, linewidth=0.3, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build TIERS HDL and GT LiDAR accumulation maps."
    )
    parser.add_argument(
        "--bag",
        action="append",
        default=[],
        help=(
            "ROS 2 bag directory to read. Repeat to merge topics from multiple "
            f"bags. Defaults to {DEFAULT_BAG}."
        ),
    )
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--lidar-topic", default="/filtered_points")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--gt-pose-topic", default="/vrpn_client_node/UWBTest/pose")
    parser.add_argument("--native-map-topic", default="/hdl_graph_slam/map_points")
    parser.add_argument("--voxel-size", type=float, default=0.05)
    parser.add_argument("--point-step", type=int, default=5)
    parser.add_argument("--max-points-per-scan", type=int, default=20_000)
    parser.add_argument(
        "--max-pose-match-dt",
        type=float,
        default=1.0,
        help=(
            "Only accumulate LiDAR scans whose nearest HDL and GT poses are "
            "within this many seconds. Use <=0 to disable filtering."
        ),
    )
    parser.add_argument(
        "--allow-missing-hdl",
        action="store_true",
        help="Generate GT/native outputs when HDL odom is absent instead of failing.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bag_paths = [Path(item).expanduser() for item in args.bag] or [Path(DEFAULT_BAG)]
    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    topics = {
        "lidar": args.lidar_topic,
        "odom": args.odom_topic,
        "tf": args.tf_topic,
        "gt_pose": args.gt_pose_topic,
        "native_map": args.native_map_topic,
    }
    odom_poses, gt_poses, map_odom_tfs, native_clouds, lidar_clouds, topic_counts = read_bags(
        bag_paths, topics, args.max_points_per_scan, max(1, args.point_step)
    )
    hdl_poses, tf_match_max_dt = apply_map_to_odom(odom_poses, map_odom_tfs)

    missing = []
    if not lidar_clouds:
        missing.append(args.lidar_topic)
    if not gt_poses:
        missing.append(args.gt_pose_topic)
    if not hdl_poses and not args.allow_missing_hdl:
        missing.append(args.odom_topic)
    if missing:
        raise RuntimeError(f"missing required topic data: {', '.join(missing)}")

    summary = {
        "bags": [str(path) for path in bag_paths],
        "topics": topics,
        "topic_counts": topic_counts,
        "voxel_size": args.voxel_size,
        "point_step": max(1, args.point_step),
        "max_points_per_scan": args.max_points_per_scan,
        "max_pose_match_dt": args.max_pose_match_dt,
        "assumption": "LiDAR-to-pose extrinsic is identity unless the input poses already include it.",
    }

    if hdl_poses:
        comparison_clouds, cloud_filter = filter_clouds_by_pose_match(
            lidar_clouds, [hdl_poses, gt_poses], args.max_pose_match_dt
        )
    else:
        comparison_clouds, cloud_filter = filter_clouds_by_pose_match(
            lidar_clouds, [gt_poses], args.max_pose_match_dt
        )
    if not comparison_clouds:
        raise RuntimeError(
            "no LiDAR scans survived pose timestamp filtering; increase --max-pose-match-dt"
        )
    summary["comparison_lidar_scans"] = {
        "input_count": len(lidar_clouds),
        **cloud_filter,
    }

    gt_map, gt_match_max_dt = accumulate_lidar_map(comparison_clouds, gt_poses, args.voxel_size)
    write_pcd(output_dir / "gt_pose_lidar_map.pcd", gt_map)
    write_trajectory_csv(output_dir / "gt_pose_trajectory.csv", gt_poses)
    plot_map_overlay(
        output_dir / "gt_pose_lidar_map_trajectory_overlay.png",
        gt_map,
        gt_poses,
        "GT pose LiDAR map with trajectory",
    )
    summary["gt_pose_lidar_map"] = {
        "point_count": int(len(gt_map)),
        "max_lidar_pose_dt_sec": gt_match_max_dt,
    }

    if hdl_poses:
        hdl_map, hdl_match_max_dt = accumulate_lidar_map(comparison_clouds, hdl_poses, args.voxel_size)
        write_pcd(output_dir / "hdl_estimated_lidar_map.pcd", hdl_map)
        write_trajectory_csv(output_dir / "hdl_estimated_trajectory.csv", hdl_poses)
        plot_map_overlay(
            output_dir / "hdl_estimated_lidar_map_trajectory_overlay.png",
            hdl_map,
            hdl_poses,
            "HDL estimated LiDAR map with trajectory",
        )
        plot_trajectory_overlay(
            output_dir / "hdl_vs_gt_trajectory_overlay.png", hdl_poses, gt_poses
        )
        summary["hdl_estimated_lidar_map"] = {
            "point_count": int(len(hdl_map)),
            "max_lidar_pose_dt_sec": hdl_match_max_dt,
            "max_odom_tf_dt_sec": tf_match_max_dt,
            "map_to_odom_tf_count": len(map_odom_tfs),
            "used_identity_map_to_odom": len(map_odom_tfs) == 0,
        }

    if native_clouds:
        native_points = voxel_downsample(native_clouds[-1].points, args.voxel_size)
        write_pcd(output_dir / "hdl_native_map_points.pcd", native_points)
        plot_map_overlay(
            output_dir / "hdl_native_map_points_overlay.png",
            native_points,
            hdl_poses,
            "HDL native map_points with HDL trajectory",
        )
        summary["hdl_native_map_points"] = {
            "point_count": int(len(native_points)),
            "source_stamp_ns": native_clouds[-1].stamp_ns,
        }

    with (output_dir / "comparison_summary.json").open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, sort_keys=True)
        f.write("\n")

    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
