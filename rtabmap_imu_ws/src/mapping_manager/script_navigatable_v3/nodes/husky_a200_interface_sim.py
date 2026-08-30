#!/usr/bin/env python3

import math
import struct
from typing import Iterable, Tuple

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_from_rpy(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class HuskyA200InterfaceSim(Node):
    """Small ROS interface simulator for the V3 Husky scripts."""

    def __init__(self) -> None:
        super().__init__("husky_a200_interface_sim")

        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("image_width", 320)
        self.declare_parameter("image_height", 240)
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("cmd_timeout", 0.6)

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.width = int(self.get_parameter("image_width").value)
        self.height = int(self.get_parameter("image_height").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_step_time = self.get_clock().now()
        self.frame_index = 0

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.platform_odom_pub = self.create_publisher(
            Odometry, "/a200_0881/platform/odom", sensor_qos
        )
        self.clearpath_filtered_pub = self.create_publisher(
            Odometry, "/a200_0881/odometry/filtered", 10
        )
        self.filtered_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.color_pub = self.create_publisher(
            Image, "/camera/camera/color/image_raw", 10
        )
        self.depth_pub = self.create_publisher(
            Image, "/camera/camera/aligned_depth_to_color/image_raw", 10
        )
        self.color_info_pub = self.create_publisher(
            CameraInfo, "/camera/camera/color/camera_info", 10
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo, "/camera/camera/depth/camera_info", 10
        )

        self.create_subscription(
            TwistStamped, "/a200_0881/cmd_vel", self.stamped_cmd_callback, 10
        )
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_camera_static_tf()

        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self.step)
        self.get_logger().info(
            "Publishing simulated A200 odom, IMU and RGB-D camera topics for V3."
        )

    def stamped_cmd_callback(self, message: TwistStamped) -> None:
        self.set_cmd(message.twist)

    def cmd_callback(self, message: Twist) -> None:
        self.set_cmd(message)

    def set_cmd(self, twist: Twist) -> None:
        self.linear_x = max(min(float(twist.linear.x), 1.0), -0.4)
        self.angular_z = max(min(float(twist.angular.z), 1.5), -1.5)
        self.last_cmd_time = self.get_clock().now()

    def step(self) -> None:
        now = self.get_clock().now()
        dt = max((now - self.last_step_time).nanoseconds / 1e9, 0.0)
        self.last_step_time = now

        if (now - self.last_cmd_time).nanoseconds / 1e9 > self.cmd_timeout:
            self.linear_x = 0.0
            self.angular_z = 0.0

        self.yaw += self.angular_z * dt
        self.x += math.cos(self.yaw) * self.linear_x * dt
        self.y += math.sin(self.yaw) * self.linear_x * dt

        stamp = now.to_msg()
        self.publish_odom(stamp)
        self.publish_imu(stamp)
        self.publish_camera(stamp)
        self.frame_index += 1

    def publish_odom(self, stamp: Time) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.linear_x
        odom.twist.twist.angular.z = self.angular_z
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.02
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.03

        self.platform_odom_pub.publish(odom)
        self.clearpath_filtered_pub.publish(odom)
        self.filtered_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def publish_imu(self, stamp: Time) -> None:
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        # Match the real mounting: IMU x points forward, y right and z down.
        # Its pose in the world is therefore robot yaw followed by roll=pi.
        qx, qy, qz, qw = quaternion_from_rpy(math.pi, 0.0, self.yaw)
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        # Yaw about base_link +z and gravity-specific force along base_link +z
        # are both negative on the sensor's downward-pointing z axis.
        imu.angular_velocity.z = -self.angular_z
        imu.linear_acceleration.z = -9.80665
        imu.orientation_covariance[0] = 0.05
        imu.orientation_covariance[4] = 0.05
        imu.orientation_covariance[8] = 0.05
        imu.angular_velocity_covariance[8] = 0.01
        imu.linear_acceleration_covariance[8] = 0.02
        self.imu_pub.publish(imu)

    def publish_camera(self, stamp: Time) -> None:
        color = Image()
        color.header.stamp = stamp
        color.header.frame_id = "camera_color_optical_frame"
        color.height = self.height
        color.width = self.width
        color.encoding = "rgb8"
        color.is_bigendian = False
        color.step = self.width * 3
        color.data = self.color_bytes()
        self.color_pub.publish(color)

        depth = Image()
        depth.header.stamp = stamp
        depth.header.frame_id = "camera_depth_optical_frame"
        depth.height = self.height
        depth.width = self.width
        depth.encoding = "16UC1"
        depth.is_bigendian = False
        depth.step = self.width * 2
        depth.data = self.depth_bytes()
        self.depth_pub.publish(depth)

        info = self.camera_info(stamp, "camera_color_optical_frame")
        self.color_info_pub.publish(info)
        depth_info = self.camera_info(stamp, "camera_depth_optical_frame")
        self.depth_info_pub.publish(depth_info)

    def camera_info(self, stamp: Time, frame_id: str) -> CameraInfo:
        fx = fy = float(self.width) * 0.9
        cx = (float(self.width) - 1.0) * 0.5
        cy = (float(self.height) - 1.0) * 0.5

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.height = self.height
        info.width = self.width
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def color_bytes(self) -> bytes:
        data = bytearray(self.width * self.height * 3)
        phase = (self.frame_index * 3) % 255
        for row in range(self.height):
            for col in range(self.width):
                i = (row * self.width + col) * 3
                data[i] = (col + phase) % 255
                data[i + 1] = (row * 2) % 255
                data[i + 2] = (120 + int(40.0 * math.sin((col + phase) / 30.0))) % 255
        return bytes(data)

    def depth_bytes(self) -> bytes:
        data = bytearray(self.width * self.height * 2)
        obstacle_center = int(
            self.width * (0.5 + 0.2 * math.sin(self.frame_index * 0.05))
        )
        floor_start = int(self.height * 0.62)
        for row in range(self.height):
            for col in range(self.width):
                distance_mm = 3500 + int(700.0 * math.sin(col / 35.0))
                if row > floor_start:
                    distance_mm = 1600 + (self.height - row) * 35
                if abs(col - obstacle_center) < 24 and 80 < row < 175:
                    distance_mm = 1200
                struct.pack_into("<H", data, (row * self.width + col) * 2, distance_mm)
        return bytes(data)

    def publish_camera_static_tf(self) -> None:
        transforms = [
            self.static_transform(self.camera_frame, "camera_color_frame"),
            self.static_transform(self.camera_frame, "camera_depth_frame"),
            self.static_transform(
                "camera_color_frame",
                "camera_color_optical_frame",
                quaternion_from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0),
            ),
            self.static_transform(
                "camera_depth_frame",
                "camera_depth_optical_frame",
                quaternion_from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0),
            ),
        ]
        self.static_tf_broadcaster.sendTransform(transforms)

    def static_transform(
        self,
        parent: str,
        child: str,
        quaternion: Iterable[float] = (0.0, 0.0, 0.0, 1.0),
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        qx, qy, qz, qw = quaternion
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform


def main() -> None:
    rclpy.init()
    node = HuskyA200InterfaceSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
