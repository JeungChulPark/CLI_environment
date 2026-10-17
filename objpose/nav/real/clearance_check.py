# Situation check before sending a goal: what the robot's sensors and costmaps say about the space around it.
#  - scan (base_link frame): clearance from the footprint on each side, free straight distance forward/backward,
#    whether turning in place is possible (no point inside the footprint's corner circle)
#  - local / global costmap cost under the robot centre (100 lethal, 99 inscribed)
import math, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros
NS = '/a200_0881'; HL, HW = 0.52, 0.38       # footprint half length / half width (nav2_real.yaml)
BODY_L, BODY_W = 0.50, 0.335                  # bare Husky A200 half dims (~1.0 x 0.67 m)
rclpy.init(args=['--ros-args', '-r', f'/tf:={NS}/tf', '-r', f'/tf_static:={NS}/tf_static'])
n = Node('clearance_check'); g = {}; buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
n.create_subscription(LaserScan, NS + '/scan', lambda m: g.update(scan=m), qos_profile_sensor_data)
n.create_subscription(OccupancyGrid, NS + '/local_costmap/costmap', lambda m: g.update(local=m), QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
n.create_subscription(OccupancyGrid, NS + '/global_costmap/costmap', lambda m: g.update(glob=m), QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
t0 = time.time()
while time.time() - t0 < 8 and not all(k in g for k in ('scan', 'local', 'glob')): rclpy.spin_once(n, timeout_sec=0.1)
s = g['scan']; r = np.array(s.ranges); a = s.angle_min + np.arange(len(r)) * s.angle_increment; ok = np.isfinite(r)
x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
def box_gap(px, py, hl, hw):   # distance from points to an axis-aligned box centred at the robot
    dx = np.maximum(np.abs(px) - hl, 0); dy = np.maximum(np.abs(py) - hw, 0); return np.hypot(dx, dy)
gap = box_gap(x, y, BODY_L, BODY_W)
print(f'scan points {ok.sum()}, nearest to the bare body: {gap.min():.2f} m')
for name, m in (('front', x > BODY_L), ('back', x < -BODY_L), ('left', y > BODY_W), ('right', y < -BODY_W)):
    if m.any(): i = np.argmin(gap[m]); print(f'  {name:5s}: nearest {gap[m][i]:.2f} m from the body (point at x {x[m][i]:+.2f}, y {y[m][i]:+.2f})')
fw = x[(np.abs(y) < HW) & (x > 0)]; bw = x[(np.abs(y) < HW) & (x < 0)]
print(f'free straight ahead (footprint corridor): {fw.min() - HL if len(fw) else 9:.2f} m;  straight back: {-bw.max() - HL if len(bw) else 9:.2f} m')
cr = math.hypot(HL, HW); inside = np.hypot(x, y) < cr
print(f'turn in place: {"BLOCKED" if inside.any() else "possible"} ({inside.sum()} scan points inside the {cr:.2f} m corner circle)')
for k in ('local', 'glob'):
    m = g[k]; frame = m.header.frame_id
    try:
        t = buf.lookup_transform(frame, 'base_link', Time()).transform.translation
        arr = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
        i, j = int((t.y - m.info.origin.position.y) / m.info.resolution), int((t.x - m.info.origin.position.x) / m.info.resolution)
        win = arr[max(0, i - 2):i + 3, max(0, j - 2):j + 3]
        print(f'{k} costmap ({frame}): cost under robot centre {arr[i, j]} (max in 25 cm window {win.max()})')
    except Exception as e: print(k, 'n/a', e)
