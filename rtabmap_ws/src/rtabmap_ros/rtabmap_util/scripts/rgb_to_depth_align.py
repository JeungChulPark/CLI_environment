#!/usr/bin/env python3

from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def _camera_model(config, name):
    section = config[name]
    return {
        "width": int(section["width"]),
        "height": int(section["height"]),
        "fx": float(section["fx"]),
        "fy": float(section["fy"]),
        "cx": float(section["cx"]),
        "cy": float(section["cy"]),
    }


def _scaled_intrinsics(model, width, height):
    sx = float(width) / float(model["width"])
    sy = float(height) / float(model["height"])
    return (
        model["fx"] * sx,
        model["fy"] * sy,
        model["cx"] * sx,
        model["cy"] * sy,
    )


class RgbToDepthAlign(Node):
    def __init__(self):
        super().__init__("rgb_to_depth_align")

        self.declare_parameter("color_topic", "/cam_1/color/image_raw")
        self.declare_parameter("depth_topic", "/cam_1/depth/image_rect_raw")
        self.declare_parameter("aligned_color_topic", "/cam_1/aligned/color/image_raw")
        self.declare_parameter("aligned_depth_topic", "/cam_1/aligned/depth/image_rect_raw")
        self.declare_parameter(
            "calibration_yaml",
            "/home/etri/convert_ros1bag_to_ros2bag/tiers_l515_oneshot/l515_calibration.yaml",
        )
        self.declare_parameter("output_frame_id", "cam_1_depth_optical_frame")
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("queue_size", 10)

        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        aligned_color_topic = self.get_parameter("aligned_color_topic").value
        aligned_depth_topic = self.get_parameter("aligned_depth_topic").value
        calibration_yaml = self.get_parameter("calibration_yaml").value
        self.output_frame_id = self.get_parameter("output_frame_id").value
        sync_slop = float(self.get_parameter("sync_slop").value)
        queue_size = int(self.get_parameter("queue_size").value)

        with Path(calibration_yaml).expanduser().open("r", encoding="utf-8") as stream:
            calibration = yaml.safe_load(stream) or {}

        self.color_model = _camera_model(calibration, "color")
        self.depth_model = _camera_model(calibration, "depth")
        self.depth_scale = float(calibration["depth"].get("scale_mm_to_m", 0.001))
        self.r_depth_to_color = np.asarray(calibration["depth_to_color"]["R"], dtype=np.float32)
        self.t_depth_to_color = np.asarray(calibration["depth_to_color"]["t"], dtype=np.float32)

        self.bridge = CvBridge()
        raw_qos = QoSProfile(depth=queue_size, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(depth=queue_size, reliability=ReliabilityPolicy.RELIABLE)

        self.color_pub = self.create_publisher(Image, aligned_color_topic, pub_qos)
        self.depth_pub = self.create_publisher(Image, aligned_depth_topic, pub_qos)

        self.color_sub = message_filters.Subscriber(
            self, Image, self.color_topic, qos_profile=raw_qos
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, self.depth_topic, qos_profile=raw_qos
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=queue_size,
            slop=sync_slop,
        )
        self.sync.registerCallback(self.callback)
        self._logged_shape = None

        self.get_logger().info(
            "Aligning RGB %s to depth %s, publishing %s and %s in %s"
            % (
                self.color_topic,
                self.depth_topic,
                aligned_color_topic,
                aligned_depth_topic,
                self.output_frame_id,
            )
        )

    def _depth_to_meters(self, depth):
        if depth.dtype == np.uint16:
            return depth.astype(np.float32) * self.depth_scale
        return depth.astype(np.float32)

    def _aligned_color(self, color_msg, depth_msg):
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        if depth.ndim != 2:
            raise ValueError("Expected a single-channel depth image")

        depth_h, depth_w = depth.shape[:2]
        color_h, color_w = color.shape[:2]
        d_fx, d_fy, d_cx, d_cy = _scaled_intrinsics(self.depth_model, depth_w, depth_h)
        c_fx, c_fy, c_cx, c_cy = _scaled_intrinsics(self.color_model, color_w, color_h)

        grid_y, grid_x = np.indices((depth_h, depth_w), dtype=np.float32)
        z = self._depth_to_meters(depth)
        x = (grid_x - d_cx) * z / d_fx
        y = (grid_y - d_cy) * z / d_fy

        r = self.r_depth_to_color
        t = self.t_depth_to_color
        x_color = r[0, 0] * x + r[0, 1] * y + r[0, 2] * z + t[0]
        y_color = r[1, 0] * x + r[1, 1] * y + r[1, 2] * z + t[1]
        z_color = r[2, 0] * x + r[2, 1] * y + r[2, 2] * z + t[2]

        with np.errstate(divide="ignore", invalid="ignore"):
            map_x = c_fx * x_color / z_color + c_cx
            map_y = c_fy * y_color / z_color + c_cy

        valid = (
            np.isfinite(z)
            & (z > 0.0)
            & np.isfinite(z_color)
            & (z_color > 0.0)
            & (map_x >= 0.0)
            & (map_x <= float(color_w - 1))
            & (map_y >= 0.0)
            & (map_y <= float(color_h - 1))
        )
        map_x = np.where(valid, map_x, -1.0).astype(np.float32)
        map_y = np.where(valid, map_y, -1.0).astype(np.float32)

        return cv2.remap(
            color,
            map_x,
            map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )

    def _copy_depth_with_aligned_header(self, depth_msg):
        aligned_depth = Image()
        aligned_depth.header.stamp = depth_msg.header.stamp
        aligned_depth.header.frame_id = self.output_frame_id
        aligned_depth.height = depth_msg.height
        aligned_depth.width = depth_msg.width
        aligned_depth.encoding = depth_msg.encoding
        aligned_depth.is_bigendian = depth_msg.is_bigendian
        aligned_depth.step = depth_msg.step
        aligned_depth.data = depth_msg.data
        return aligned_depth

    def callback(self, color_msg, depth_msg):
        try:
            aligned = self._aligned_color(color_msg, depth_msg)
        except Exception as exc:
            self.get_logger().error(f"Failed to align RGB to depth: {exc}")
            return

        header = depth_msg.header
        aligned_color = self.bridge.cv2_to_imgmsg(aligned, encoding="bgr8")
        aligned_color.header.stamp = header.stamp
        aligned_color.header.frame_id = self.output_frame_id

        aligned_depth = self._copy_depth_with_aligned_header(depth_msg)

        self.color_pub.publish(aligned_color)
        self.depth_pub.publish(aligned_depth)

        shape = (aligned_color.width, aligned_color.height, aligned_depth.width, aligned_depth.height)
        if shape != self._logged_shape:
            self._logged_shape = shape
            self.get_logger().info(
                "Aligned RGB %dx%d and depth %dx%d with stamp/frame from depth"
                % shape
            )


def main(args=None):
    rclpy.init(args=args)
    node = RgbToDepthAlign()
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
