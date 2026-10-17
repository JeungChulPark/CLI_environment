# Log wheel/EKF odom and hdl scan-matching odom side by side while someone drives, to find when hdl diverges.
# Each source is expressed relative to its own start pose (start-heading frame), so the two should match.
# usage: python3 drive_log.py seconds out.csv
import sys, time, math, csv, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
NS = '/a200_0881'
rclpy.init(); n = Node('drive_log'); last = {}
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
def put(k):
    def f(m):
        p = m.pose.pose; last[k] = (p.position.x, p.position.y, p.position.z, yaw(p.orientation))
    return f
rel = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(Odometry, NS + '/platform/odom/filtered', put('w'), rel)
n.create_subscription(Odometry, '/odom', put('h'), rel)
T = float(sys.argv[1]); rows = []; t0 = time.time(); nxt = t0; origin = {}
def rel_pose(k):
    x, y, z, h = last[k]; x0, y0, z0, h0 = origin[k]; dx, dy = x - x0, y - y0
    c, s = math.cos(-h0), math.sin(-h0)
    return dx * c - dy * s, dx * s + dy * c, z - z0, math.degrees((h - h0 + math.pi) % (2 * math.pi) - math.pi)
while time.time() - t0 < T:
    rclpy.spin_once(n, timeout_sec=0.02)
    if time.time() >= nxt and 'w' in last and 'h' in last:
        nxt += 0.1
        if not origin: origin = dict(last)
        w, h = rel_pose('w'), rel_pose('h')
        rows.append([round(time.time() - t0, 2)] + [round(v, 3) for v in w + h] + [round(math.hypot(w[0] - h[0], w[1] - h[1]), 3), round(abs(w[3] - h[3]), 2)])
with open(sys.argv[2], 'w', newline='') as f:
    wr = csv.writer(f); wr.writerow(['t', 'wx', 'wy', 'wz', 'wyaw', 'hx', 'hy', 'hz', 'hyaw', 'pos_diff', 'yaw_diff']); wr.writerows(rows)
moved = [r for r in rows if math.hypot(r[1], r[2]) > 0.02 or abs(r[4]) > 2]
first_bad = next((r for r in rows if r[9] > 0.15 or r[10] > 8), None)
print(f"{len(rows)} samples; wheel path end ({rows[-1][1]:+.2f},{rows[-1][2]:+.2f}) yaw {rows[-1][4]:+.1f}; hdl end ({rows[-1][5]:+.2f},{rows[-1][6]:+.2f}) z {rows[-1][7]:+.2f} yaw {rows[-1][8]:+.1f}")
print(f"max pos diff {max(r[9] for r in rows):.2f} m, max yaw diff {max(r[10] for r in rows):.1f} deg; motion samples {len(moved)}")
print('first divergence (>15 cm or >8 deg):', first_bad if first_bad else 'none')
for r in rows[::10]: print(r)
