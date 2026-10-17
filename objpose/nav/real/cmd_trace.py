# Where do Nav2's velocity commands get lost? Records, for N seconds, every stage of the command chain plus the wheels:
#   controller (cmd_vel_nav) -> velocity smoother (cmd_vel_smoothed) -> collision monitor (cmd_vel)
#   -> twist_mux output (platform/cmd_vel) -> wheel odometry (platform/odom), and the collision monitor state.
# usage: python3 cmd_trace.py [seconds]   (send a goal from RViz while it runs)
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
NS = '/a200_0881'; T = float(sys.argv[1]) if len(sys.argv) > 1 else 40
rclpy.init(); n = Node('cmd_trace'); rec = {}
def tw(k):
    def f(m): rec.setdefault(k, []).append((time.time(), m.twist.linear.x, m.twist.angular.z))
    return f
rel = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
for k in ('cmd_vel_nav', 'cmd_vel_smoothed', 'cmd_vel', 'platform/cmd_vel', 'twist_marker_server/cmd_vel'):
    n.create_subscription(TwistStamped, f'{NS}/{k}', tw(k), rel)
n.create_subscription(Odometry, NS + '/platform/odom', lambda m: rec.setdefault('wheels', []).append(
    (time.time(), m.twist.twist.linear.x, m.twist.twist.angular.z)), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < T: rclpy.spin_once(n, timeout_sec=0.05)
print(f'{"stage":30s} {"msgs":>5s} {"nonzero":>8s} {"max|v|":>7s} {"max|w|":>7s}  first nonzero at')
for k in ('cmd_vel_nav', 'cmd_vel_smoothed', 'cmd_vel', 'twist_marker_server/cmd_vel', 'platform/cmd_vel', 'wheels'):
    a = rec.get(k, [])
    nz = [x for x in a if abs(x[1]) > 0.01 or abs(x[2]) > 0.02]
    print(f'{k:30s} {len(a):5d} {len(nz):8d} {max([abs(x[1]) for x in a] or [0]):7.3f} {max([abs(x[2]) for x in a] or [0]):7.3f}  '
          f'{(nz[0][0] - t0) if nz else float("nan"):.1f} s')
