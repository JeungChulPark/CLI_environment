#!/usr/bin/env python3
"""cmd_lag_rec.py — record the velocity command chain and the wheel odometry with receive times, to measure how long a
command takes to show up at the wheels (laptop, next to real_loc_up.sh / real_up.sh; real_down.sh stops it).
  controller (MPPI) -> cmd_vel_nav -> velocity_smoother -> cmd_vel -> twist_mux -> platform/cmd_vel -> base -> platform/odom
Rows (gzip CSV, logs/cmd_lag_<time>.csv.gz): t_recv_s (monotonic), topic, stamp_s (header), vx, wz
Analysis: objpose/nav/real/cmd_lag_eval.py <csv.gz>
"""
import gzip, os, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

NS = os.environ.get("NS", "/a200_0881")
here = os.path.dirname(os.path.abspath(__file__))
out = os.path.join(here, "logs", time.strftime("cmd_lag_%y%m%d_%H%M%S.csv.gz"))
rclpy.init(); n = Node("cmd_lag_rec"); f = gzip.open(out, "wt"); f.write("t_recv_s,topic,stamp_s,vx,wz\n"); cnt = {"n": 0}


def put(topic, stamp, vx, wz):
    f.write(f"{time.monotonic():.4f},{topic},{stamp.sec + stamp.nanosec * 1e-9:.4f},{vx:.4f},{wz:.4f}\n"); cnt["n"] += 1


for t in ("cmd_vel_nav", "cmd_vel", "platform/cmd_vel", "platform/cmd_vel_out"):
    n.create_subscription(TwistStamped, f"{NS}/{t}", lambda m, t=t: put(t, m.header.stamp, m.twist.linear.x, m.twist.angular.z), 50)
n.create_subscription(Odometry, f"{NS}/platform/odom", lambda m: put("odom", m.header.stamp, m.twist.twist.linear.x, m.twist.twist.angular.z),
                      qos_profile_sensor_data)
n.create_timer(5.0, f.flush)
n.get_logger().info(f"recording the command chain -> {out}")
try:
    rclpy.spin(n)
finally:
    f.close()
