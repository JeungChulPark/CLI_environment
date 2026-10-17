# Age of sensor messages (sim clock now - header stamp) on the laptop, to spot backlog.
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry
rclpy.init(); n = Node('latency', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)]); ages = {}
def cb(k):
    def f(m):
        now = n.get_clock().now().nanoseconds * 1e-9
        ages.setdefault(k, []).append(now - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
    return f
for k, t, T in [('points', '/a200_0881/sensors/lidar3d_0/points', PointCloud2), ('scan', '/a200_0881/sensors/lidar3d_0/scan', LaserScan),
                ('filtered', '/filtered_points', PointCloud2), ('hdl_odom', '/odom', Odometry), ('ekf', '/a200_0881/platform/odom/filtered', Odometry)]:
    n.create_subscription(T, t, cb(k), qos_profile_sensor_data)
while n.get_clock().now().nanoseconds == 0: rclpy.spin_once(n, timeout_sec=0.05)  # wait for /clock
ages.clear(); t0 = time.time()
while time.time() - t0 < 6: rclpy.spin_once(n, timeout_sec=0.05)
for k, a in ages.items(): print(f"{k:9s} n={len(a):3d} age min {min(a):6.2f} max {max(a):6.2f} s")
