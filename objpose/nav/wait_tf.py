#!/usr/bin/env python3
"""wait_tf.py <target> <source> [timeout_s] — exit 0 once TF target<-source is available (sim time aware)."""
import sys, time, rclpy
from rclpy.parameter import Parameter
import tf2_ros
rclpy.init()
n = rclpy.create_node("wait_tf", parameter_overrides=[Parameter("use_sim_time", value=True)])
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
t_end = time.time() + float(sys.argv[3] if len(sys.argv) > 3 else 120)
while time.time() < t_end:
    rclpy.spin_once(n, timeout_sec=0.2)
    if buf.can_transform(sys.argv[1], sys.argv[2], rclpy.time.Time()):
        print(f"tf {sys.argv[1]} <- {sys.argv[2]} ready"); sys.exit(0)
print("timeout waiting for tf", sys.argv[1:3]); sys.exit(1)
