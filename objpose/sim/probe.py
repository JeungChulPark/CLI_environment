# Measure sim real-time factor and sensor rates over N wall seconds. usage: python3 probe.py [secs]
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry
ns = '/a200_0881'
rclpy.init(); n = Node('sim_probe'); clk = []; cnt = {'points': 0, 'scan': 0, 'odom': 0}
def inc(k): cnt[k] += 1
n.create_subscription(Clock, '/clock', lambda m: clk.append((time.time(), m.clock.sec + m.clock.nanosec * 1e-9)), 10)
n.create_subscription(PointCloud2, ns + '/sensors/lidar3d_0/points', lambda m: inc('points'), qos_profile_sensor_data)
n.create_subscription(LaserScan, ns + '/sensors/lidar3d_0/scan', lambda m: inc('scan'), qos_profile_sensor_data)
n.create_subscription(Odometry, ns + '/platform/odom', lambda m: inc('odom'), qos_profile_sensor_data)
T = float(sys.argv[1]) if len(sys.argv) > 1 else 10
t0 = time.time()
while time.time() - t0 < T: rclpy.spin_once(n, timeout_sec=0.05)
rtf = (clk[-1][1] - clk[0][1]) / (clk[-1][0] - clk[0][0]) if len(clk) > 1 else 0
print(f"RTF={rtf:.2f} sim_t={clk[-1][1] if clk else 0:.1f}s " + ' '.join(f"{k}={v/T:.1f}Hz" for k, v in cnt.items()))
