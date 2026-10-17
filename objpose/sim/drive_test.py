# Drive the simulated Husky forward at 0.5 m/s for 4 sim-seconds and report odometry displacement.
import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
ns = '/a200_0881'
rclpy.init(); n = Node('drive_test', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
pub = n.create_publisher(TwistStamped, ns + '/cmd_vel', 10); od = {}
n.create_subscription(Odometry, ns + '/platform/odom', lambda m: od.update(p=m.pose.pose.position), qos_profile_sensor_data)
while 'p' not in od: rclpy.spin_once(n, timeout_sec=0.1)
p0 = od['p']; t0 = n.get_clock().now().nanoseconds
while (n.get_clock().now().nanoseconds - t0) < 4e9:
    m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg(); m.twist.linear.x = 0.5; pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.05)
pub.publish(TwistStamped())
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.05)
p1 = od['p']; print(f"moved {math.hypot(p1.x-p0.x, p1.y-p0.y):.2f} m in 4 sim-s (expected ~2.0)")
