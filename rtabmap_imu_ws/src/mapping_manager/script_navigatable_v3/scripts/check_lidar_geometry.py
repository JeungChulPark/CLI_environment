#!/usr/bin/env python3

"""Reject VLP-16 data that is not local to the robot or exceeds its range cap."""

import argparse
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
import yaml


ROOT = Path(__file__).resolve().parents[1]


class LidarGeometryCheck(Node):
    def __init__(self, sample_count: int) -> None:
        super().__init__("lidar_geometry_check")
        self.sample_count = sample_count
        self.cloud_frames: list[str] = []
        self.scan_samples: list[tuple[str, list[float]]] = []
        # The Velodyne cloud is reliable, while pointcloud_to_laserscan uses
        # sensor-data (best-effort) QoS. Match each publisher explicitly.
        cloud_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(
            PointCloud2,
            "/velodyne_points",
            self._cloud_callback,
            cloud_qos,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            scan_qos,
        )

    def _cloud_callback(self, message: PointCloud2) -> None:
        if len(self.cloud_frames) < self.sample_count:
            self.cloud_frames.append(message.header.frame_id)

    def _scan_callback(self, message: LaserScan) -> None:
        if len(self.scan_samples) >= self.sample_count:
            return
        finite = [float(value) for value in message.ranges if math.isfinite(value)]
        self.scan_samples.append((message.header.frame_id, finite))

    @property
    def complete(self) -> bool:
        return (
            len(self.cloud_frames) >= self.sample_count
            and len(self.scan_samples) >= self.sample_count
        )


def normalized_frame(frame_id: str) -> str:
    return frame_id.lstrip("/")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "config" / "vlp16.yaml",
    )
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--min-returns", type=int, default=25)
    parser.add_argument("--range-tolerance", type=float, default=0.05)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if args.samples < 1 or args.timeout <= 0.0 or args.min_returns < 1:
        raise SystemExit("samples, timeout and min-returns must be positive")

    with args.config.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    driver_frame = normalized_frame(str(config["driver"]["frame_id"]))
    transform = config["transform"]
    laser_scan = config["laser_scan"]
    fixed_frame = normalized_frame(str(transform.get("fixed_frame", "")))
    target_frame = normalized_frame(str(transform.get("target_frame", "")))
    min_range = float(transform["min_range"])
    max_range = float(transform["max_range"])
    expected_scan_frame = normalized_frame(
        str(laser_scan.get("target_frame", driver_frame))
        if laser_scan.get("mode") == "all_rings"
        else driver_frame
    )

    # This repository deliberately uses the version-independent sensor-frame
    # path. Non-empty values can reintroduce the fixed/target reversal present
    # in velodyne_pointcloud 2.5.1, so fail before accepting misleading data.
    if fixed_frame or target_frame:
        print(
            "FAIL: fixed_frame and target_frame must both be empty; "
            f"got fixed={fixed_frame!r}, target={target_frame!r}"
        )
        return 2

    rclpy.init(args=[])
    node = LidarGeometryCheck(args.samples)
    deadline = time.monotonic() + args.timeout
    try:
        while not node.complete and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(node.cloud_frames) < args.samples or len(node.scan_samples) < args.samples:
        print(
            "FAIL: insufficient LiDAR data "
            f"(cloud={len(node.cloud_frames)}/{args.samples}, "
            f"scan={len(node.scan_samples)}/{args.samples})"
        )
        return 2

    wrong_cloud_frames = sorted(
        {
            normalized_frame(frame)
            for frame in node.cloud_frames
            if normalized_frame(frame) != driver_frame
        }
    )
    scan_frames = [normalized_frame(frame) for frame, _ in node.scan_samples]
    wrong_scan_frames = sorted(
        {frame for frame in scan_frames if frame != expected_scan_frame}
    )
    if wrong_cloud_frames or wrong_scan_frames:
        print(
            "FAIL: LiDAR data is not body-fixed "
            f"(cloud expected={driver_frame!r}, "
            f"cloud={wrong_cloud_frames or [driver_frame]}, "
            f"scan expected={expected_scan_frame!r}, "
            f"scan={wrong_scan_frames or [expected_scan_frame]})"
        )
        return 2

    counts = [len(finite) for _, finite in node.scan_samples]
    if any(count < args.min_returns for count in counts):
        print(
            "FAIL: sparse /scan "
            f"(finite returns min/avg/max={min(counts)}/"
            f"{sum(counts) / len(counts):.1f}/{max(counts)})"
        )
        return 2

    finite_ranges = [
        value
        for _, finite in node.scan_samples
        for value in finite
    ]
    nearest = min(finite_ranges)
    farthest = max(finite_ranges)
    if nearest < min_range - args.range_tolerance:
        print(
            f"FAIL: nearest finite return {nearest:.3f} m is below "
            f"configured minimum {min_range:.3f} m"
        )
        return 2
    if farthest > max_range + args.range_tolerance:
        print(
            f"FAIL: farthest finite return {farthest:.3f} m exceeds "
            f"configured maximum {max_range:.3f} m"
        )
        return 2

    print(
        f"PASS: cloud_frame={driver_frame}, scan_frame={expected_scan_frame}, "
        f"samples={args.samples}, "
        f"finite returns min/avg/max={min(counts)}/"
        f"{sum(counts) / len(counts):.1f}/{max(counts)}, "
        f"nearest={nearest:.2f} m, farthest={farthest:.2f} m, "
        f"configured range={min_range:.2f}-{max_range:.2f} m"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
