#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class SafeCmdVelBridge(Node):
    """Bridge collision-checked commands only after RTAB-Map is localized."""

    def __init__(self):
        super().__init__("safe_cmd_vel_bridge")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/a200_0881/cmd_vel")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("require_localization", True)
        self.declare_parameter(
            "localization_topic", "/rtabmap/localization_pose"
        )
        self.declare_parameter("localization_timeout", 1.5)
        self.declare_parameter("minimum_valid_samples", 3)
        self.declare_parameter("max_xy_variance", 0.10)
        self.declare_parameter("max_yaw_variance", 0.10)
        self.declare_parameter("max_pose_jump", 0.35)
        self.declare_parameter("max_yaw_jump", 0.35)
        self.declare_parameter("plausible_linear_speed", 0.8)
        self.declare_parameter("plausible_angular_speed", 1.5)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.allow_reverse = self.get_parameter("allow_reverse").value
        self.require_localization = self.get_parameter(
            "require_localization"
        ).value
        localization_topic = self.get_parameter("localization_topic").value
        self.localization_timeout = float(
            self.get_parameter("localization_timeout").value
        )
        self.minimum_valid_samples = int(
            self.get_parameter("minimum_valid_samples").value
        )
        self.max_xy_variance = float(
            self.get_parameter("max_xy_variance").value
        )
        self.max_yaw_variance = float(
            self.get_parameter("max_yaw_variance").value
        )
        self.max_pose_jump = float(self.get_parameter("max_pose_jump").value)
        self.max_yaw_jump = float(self.get_parameter("max_yaw_jump").value)
        self.plausible_linear_speed = float(
            self.get_parameter("plausible_linear_speed").value
        )
        self.plausible_angular_speed = float(
            self.get_parameter("plausible_angular_speed").value
        )

        if self.localization_timeout <= 0.0:
            raise ValueError("localization_timeout must be positive")
        if self.minimum_valid_samples < 1:
            raise ValueError("minimum_valid_samples must be at least one")

        self.publisher = self.create_publisher(TwistStamped, output_topic, 10)
        self.subscription = self.create_subscription(
            Twist, input_topic, self.callback, 10
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_publisher = self.create_publisher(
            Bool, "/navigation/localization_ready", state_qos
        )

        self.localization_subscription = None
        self.localization_ready = not self.require_localization
        self.valid_sample_count = 0
        self.last_localization_time = None
        self.last_localization_pose = None
        self.block_reason = "waiting for RTAB-Map localization"
        if self.require_localization:
            self.localization_subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                localization_topic,
                self.localization_callback,
                10,
            )
        self.watchdog = self.create_timer(0.1, self.watchdog_callback)
        self.publish_readiness()

        self.get_logger().info(
            f"Safety bridge: {input_topic} (Twist) -> "
            f"{output_topic} (TwistStamped)"
        )
        if self.require_localization:
            self.get_logger().warn(
                "Navigation motion is LOCKED until RTAB-Map reports "
                "three stable localization poses. Publish /initialpose "
                "from RViz if the robot is not at the last mapped pose."
            )

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def publish_readiness(self) -> None:
        message = Bool()
        message.data = bool(self.localization_ready)
        self.state_publisher.publish(message)

    def set_localization_ready(self, ready: bool, reason: str) -> None:
        changed = ready != self.localization_ready
        self.localization_ready = ready
        self.block_reason = reason
        if changed:
            self.publish_readiness()
            if ready:
                self.get_logger().info(
                    "Navigation motion UNLOCKED: RTAB-Map localization is "
                    "valid and stable."
                )
            else:
                self.get_logger().error(
                    f"Navigation motion LOCKED: {reason}"
                )

    @staticmethod
    def quaternion_yaw(message: PoseWithCovarianceStamped) -> float | None:
        quaternion = message.pose.pose.orientation
        norm = math.sqrt(
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w
        )
        if not math.isfinite(norm) or norm < 1.0e-6:
            return None
        x = quaternion.x / norm
        y = quaternion.y / norm
        z = quaternion.z / norm
        w = quaternion.w / norm
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    @staticmethod
    def angle_difference(left: float, right: float) -> float:
        return math.atan2(math.sin(left - right), math.cos(left - right))

    def localization_callback(
        self, message: PoseWithCovarianceStamped
    ) -> None:
        now = self.now_seconds()
        covariance = message.pose.covariance
        x_variance = float(covariance[0])
        y_variance = float(covariance[7])
        yaw_variance = float(covariance[35])
        yaw = self.quaternion_yaw(message)
        position = message.pose.pose.position

        covariance_valid = (
            all(
                math.isfinite(value) and value >= 0.0
                for value in (x_variance, y_variance, yaw_variance)
            )
            and x_variance <= self.max_xy_variance
            and y_variance <= self.max_xy_variance
            and yaw_variance <= self.max_yaw_variance
        )
        pose_valid = (
            message.header.frame_id.lstrip("/") == "map"
            and yaw is not None
            and math.isfinite(position.x)
            and math.isfinite(position.y)
        )
        if not covariance_valid or not pose_valid:
            self.valid_sample_count = 0
            self.last_localization_time = now
            self.last_localization_pose = None
            self.set_localization_ready(
                False,
                "RTAB-Map has no bounded map-frame localization "
                f"(variance x={x_variance:.3g}, y={y_variance:.3g}, "
                f"yaw={yaw_variance:.3g})",
            )
            return

        current_pose = (float(position.x), float(position.y), float(yaw))
        if self.last_localization_pose is not None:
            previous_x, previous_y, previous_yaw, previous_time = (
                self.last_localization_pose
            )
            elapsed = max(0.0, now - previous_time)
            translation = math.hypot(
                current_pose[0] - previous_x,
                current_pose[1] - previous_y,
            )
            rotation = abs(
                self.angle_difference(current_pose[2], previous_yaw)
            )
            translation_limit = (
                self.max_pose_jump + self.plausible_linear_speed * elapsed
            )
            rotation_limit = (
                self.max_yaw_jump + self.plausible_angular_speed * elapsed
            )
            if translation > translation_limit or rotation > rotation_limit:
                self.valid_sample_count = 0
                self.last_localization_time = now
                self.last_localization_pose = (*current_pose, now)
                self.set_localization_ready(
                    False,
                    "implausible RTAB-Map pose jump "
                    f"({translation:.3f} m, {rotation:.3f} rad)",
                )
                return

        self.last_localization_time = now
        self.last_localization_pose = (*current_pose, now)
        self.valid_sample_count += 1
        if self.valid_sample_count >= self.minimum_valid_samples:
            self.set_localization_ready(True, "")

    def localization_is_fresh(self) -> bool:
        if not self.require_localization:
            return True
        if self.last_localization_time is None:
            return False
        age = self.now_seconds() - self.last_localization_time
        if age > self.localization_timeout:
            self.valid_sample_count = 0
            self.last_localization_pose = None
            self.set_localization_ready(
                False,
                f"localization pose is stale by {age:.2f} seconds",
            )
            return False
        return self.localization_ready

    def make_command(self, message: Twist | None = None) -> TwistStamped:
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        if message is not None:
            stamped.twist.linear.x = message.linear.x
            stamped.twist.linear.y = message.linear.y
            stamped.twist.linear.z = message.linear.z
            stamped.twist.angular.x = message.angular.x
            stamped.twist.angular.y = message.angular.y
            stamped.twist.angular.z = message.angular.z
        return stamped

    def callback(self, msg: Twist) -> None:
        stamped = self.make_command(
            msg if self.localization_is_fresh() else None
        )
        if not self.allow_reverse and stamped.twist.linear.x < 0.0:
            stamped.twist.linear.x = 0.0
        self.publisher.publish(stamped)

    def watchdog_callback(self) -> None:
        if not self.localization_is_fresh():
            # Continue sending zero so a previously forwarded command cannot
            # survive a camera/RTAB-Map failure until a base-side timeout.
            self.publisher.publish(self.make_command())

    def stop(self) -> None:
        self.publisher.publish(self.make_command())


def main() -> None:
    rclpy.init()
    node = SafeCmdVelBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        # ROS launch may already have shut the shared context down on SIGINT.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
