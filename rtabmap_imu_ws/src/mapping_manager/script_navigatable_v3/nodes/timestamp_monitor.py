#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Type

import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2

try:
    from rtabmap_msgs.msg import RGBDImage
except Exception:  # pragma: no cover - package may be absent in reduced installs
    RGBDImage = None

try:
    from velodyne_msgs.msg import VelodyneScan
except Exception:  # pragma: no cover
    VelodyneScan = None


@dataclass
class Sample:
    stamp_s: float
    arrival_s: float
    frame_id: str


def time_to_seconds(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class TimestampMonitor(Node):
    def __init__(self) -> None:
        super().__init__("timestamp_monitor")
        self.declare_parameter("reference_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("report_period", 2.0)
        self.declare_parameter("stale_after", 5.0)
        self.declare_parameter("max_offset_ms", 80.0)
        self.declare_parameter("max_age_ms", 500.0)

        self.reference_topic = str(self.get_parameter("reference_topic").value)
        self.report_period = float(self.get_parameter("report_period").value)
        self.stale_after = float(self.get_parameter("stale_after").value)
        self.max_offset_ms = float(self.get_parameter("max_offset_ms").value)
        self.max_age_ms = float(self.get_parameter("max_age_ms").value)
        self.samples: Dict[str, Sample] = {}

        specs = [
            ("/camera/camera/color/image_raw", Image),
            ("/camera/camera/aligned_depth_to_color/image_raw", Image),
            ("/rtabmap/rgbd_image", RGBDImage),
            ("/imu/data", Imu),
            ("/a200_0881/platform/odom", Odometry),
            ("/odometry/filtered", Odometry),
            ("/velodyne_packets", VelodyneScan),
            ("/velodyne_points", PointCloud2),
            ("/scan", LaserScan),
            ("/camera/depth_scan", LaserScan),
        ]
        for topic, msg_type in specs:
            if msg_type is not None:
                self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, name=topic: self.record(name, msg),
                    qos_profile_sensor_data,
                )

        self.create_timer(self.report_period, self.report)
        self.get_logger().info(
            "Monitoring sensor header timestamps. This reports stamp alignment; "
            "it does not rewrite sensor messages."
        )

    def record(self, topic: str, message) -> None:
        header = getattr(message, "header", None)
        if header is None:
            return
        now_s = self.get_clock().now().nanoseconds * 1e-9
        self.samples[topic] = Sample(
            stamp_s=time_to_seconds(header.stamp),
            arrival_s=now_s,
            frame_id=str(getattr(header, "frame_id", "")),
        )

    def report(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        active = {
            topic: sample
            for topic, sample in self.samples.items()
            if now_s - sample.arrival_s <= self.stale_after
        }
        if not active:
            self.get_logger().warn("No active stamped sensor topics yet.")
            return

        ref = active.get(self.reference_topic)
        if ref is None:
            ref_topic, ref = max(active.items(), key=lambda item: item[1].arrival_s)
        else:
            ref_topic = self.reference_topic

        lines = [f"reference={ref_topic} stamp={ref.stamp_s:.6f}"]
        worst_offset = 0.0
        worst_age = 0.0

        for topic in sorted(active):
            sample = active[topic]
            offset_ms = (sample.stamp_s - ref.stamp_s) * 1000.0
            age_ms = (now_s - sample.stamp_s) * 1000.0
            arrival_skew_ms = (sample.arrival_s - ref.arrival_s) * 1000.0
            worst_offset = max(worst_offset, abs(offset_ms))
            worst_age = max(worst_age, abs(age_ms))
            state = "OK"
            if abs(offset_ms) > self.max_offset_ms or abs(age_ms) > self.max_age_ms:
                state = "WARN"
            lines.append(
                f"{state:4} {topic:48} "
                f"stamp_delta={offset_ms:8.1f} ms "
                f"age={age_ms:8.1f} ms "
                f"arrival_delta={arrival_skew_ms:8.1f} ms "
                f"frame={sample.frame_id}"
            )

        message = "\n".join(lines)
        if worst_offset > self.max_offset_ms or worst_age > self.max_age_ms:
            self.get_logger().warn("\n" + message)
        else:
            self.get_logger().info("\n" + message)


def main() -> None:
    rclpy.init()
    node = TimestampMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
