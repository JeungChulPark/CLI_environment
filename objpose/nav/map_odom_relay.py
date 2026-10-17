#!/usr/bin/env python3
"""map_odom_relay.py — hdl_graph_slam map->hdl_odom (stamped with the last keyframe, so it goes stale while the
robot stands still) -> map->odom re-broadcast at a fixed rate with the current stamp, as Nav2 expects (like AMCL).
Bag mode: hdl scan-matching odometry is the `odom` frame itself, so map->odom = map->hdl_odom.
  python3 map_odom_relay.py --ros-args -p use_sim_time:=true [-p rate:=20.0]"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class Relay(Node):
    def __init__(self):
        super().__init__("map_odom_relay")
        self.src = self.declare_parameter("source_child", "hdl_odom").value
        self.parent = self.declare_parameter("map_frame", "map").value
        self.child = self.declare_parameter("odom_frame", "odom").value
        self.future = self.declare_parameter("future_dating_s", 0.1).value
        rate = self.declare_parameter("rate", 20.0).value
        self.last = None
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        self.br = TransformBroadcaster(self)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"relay {self.parent}->{self.src} as {self.parent}->{self.child} @ {rate} Hz")

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == self.parent and t.child_frame_id == self.src:
                if self.last is None:
                    self.get_logger().info("first map->hdl_odom received")
                self.last = t

    def tick(self):
        if self.last is None:
            return
        o = TransformStamped()
        o.header.stamp = (self.get_clock().now() + Duration(seconds=self.future)).to_msg()
        o.header.frame_id = self.parent
        o.child_frame_id = self.child
        o.transform = self.last.transform
        self.br.sendTransform(o)


def main():
    rclpy.init()
    rclpy.spin(Relay())


if __name__ == "__main__":
    main()
