# One-shot comparison of the pose sources on the real robot: wheel/EKF odom, hdl scan matching, map->base_link
# (what Nav2 uses), the relay correction map->odom, and hdl keyframes.
import time, math, rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int32
import tf2_ros
NS = '/a200_0881'
rclpy.init(args=['--ros-args', '-r', f'/tf:={NS}/tf', '-r', f'/tf_static:={NS}/tf_static'])
n = Node('pose_snapshot'); buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n); g = {}
yaw = lambda q: math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
rel = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
lat = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(Odometry, NS + '/platform/odom/filtered', lambda m: g.update(w=m), rel)
n.create_subscription(Odometry, '/odom', lambda m: g.update(h=m), rel)
n.create_subscription(Path, '/hdl_graph_slam/keyframes', lambda m: g.update(k=m), rel)
n.create_subscription(Int32, '/hdl_graph_slam/debug/keyframe_count', lambda m: g.update(kc=m.data), lat)
t0 = time.time()
while time.time() - t0 < 4: rclpy.spin_once(n, timeout_sec=0.05)
def show(lab, x, y, z, yw): print(f'{lab:34s} x {x:+.3f}  y {y:+.3f}  z {z:+.3f}  yaw {yw:+6.1f} deg')
for k, lab in (('w', 'wheel/EKF  odom->base_link'), ('h', 'hdl scan-matching odom->base_link')):
    if k in g: p = g[k].pose.pose; show(lab, p.position.x, p.position.y, p.position.z, yaw(p.orientation))
for a, b, lab in (('map', 'base_link', 'map->base_link (Nav2 uses)'), ('map', 'odom', 'map->odom (relay correction)')):
    try:
        r = buf.lookup_transform(a, b, Time()).transform; show(lab, r.translation.x, r.translation.y, r.translation.z, yaw(r.rotation))
    except Exception as e: print(lab, 'n/a', e)
print('keyframes:', g.get('kc'))
for ps in (g['k'].poses if 'k' in g else []):
    p = ps.pose.position; print(f'   kf x {p.x:+.2f}  y {p.y:+.2f}  z {p.z:+.2f}  yaw {yaw(ps.pose.orientation):+.1f}')
