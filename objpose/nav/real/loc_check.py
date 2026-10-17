# Localization sanity check on the real robot while someone drives it (joystick): records the wheel/EKF pose
# (platform/odom/filtered), the hdl scan-matching pose (/odom, hdl frame base_link) and map->base_link (robot tree,
# what Nav2 uses) for N s, then compares travelled distance, heading change and motion direction.
# A consistent angle between the hdl and wheel motion directions = lidar mounting yaw error (LIDAR_YAW).
# usage: python3 loc_check.py [seconds]
import sys, time, math, rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
import tf2_ros
NS = '/a200_0881'
rclpy.init(args=['--ros-args', '-r', f'/tf:={NS}/tf', '-r', f'/tf_static:={NS}/tf_static'])
n = Node('loc_check'); buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
tr = {'wheel': [], 'hdl': [], 'map': []}
rel = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(Odometry, NS + '/platform/odom/filtered', lambda m: tr['wheel'].append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))), rel)
n.create_subscription(Odometry, '/odom', lambda m: tr['hdl'].append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))), rel)
T = float(sys.argv[1]) if len(sys.argv) > 1 else 60; t0 = time.time(); last = 0
while time.time() - t0 < T:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - last > 0.1:
        last = time.time()
        try:
            t = buf.lookup_transform('map', 'base_link', Time()); r = t.transform
            tr['map'].append((last, r.translation.x, r.translation.y, yaw(r.rotation)))
        except Exception: pass
def summary(k):
    a = tr[k]
    if len(a) < 2: return None
    d = sum(math.hypot(b[1] - c[1], b[2] - c[2]) for b, c in zip(a, a[1:]))
    dh = sum((b[3] - c[3] + math.pi) % (2 * math.pi) - math.pi for c, b in zip(a, a[1:]))
    return a, d, dh
S = {k: summary(k) for k in tr}
for k, s in S.items():
    if s: print(f"{k:5s} samples {len(s[0]):4d}  path {s[1]:6.2f} m  net heading change {math.degrees(s[2]):+7.1f} deg  "
                f"start ({s[0][0][1]:+.2f},{s[0][0][2]:+.2f}) end ({s[0][-1][1]:+.2f},{s[0][-1][2]:+.2f})")
    else: print(f"{k:5s} no data")
# motion direction in each frame's own start-heading coordinates
def body_disp(a):
    x0, y0, h0 = a[0][1:]; x1, y1 = a[-1][1:3]; dx, dy = x1 - x0, y1 - y0
    return dx * math.cos(-h0) - dy * math.sin(-h0), dx * math.sin(-h0) + dy * math.cos(-h0)
if S['wheel'] and S['hdl'] and S['wheel'][1] > 0.3:
    w, h = body_disp(S['wheel'][0]), body_disp(S['hdl'][0])
    off = math.degrees(math.atan2(h[1], h[0]) - math.atan2(w[1], w[0])); off = (off + 180) % 360 - 180
    print(f"displacement in start frame: wheel ({w[0]:+.2f},{w[1]:+.2f})  hdl ({h[0]:+.2f},{h[1]:+.2f})")
    print(f"=> hdl vs wheel motion direction differs by {off:+.1f} deg; distance ratio hdl/wheel {math.hypot(*h)/max(math.hypot(*w),1e-6):.2f}")
else:
    print("robot moved less than 0.3 m by wheel odometry: drive it a bit more for the direction check")
