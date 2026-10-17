# Navigation test on the Gazebo sim: send a sequence of NavigateToPose goals (map frame) and, per goal, record
# result, time, travelled distance, arrival error against Gazebo ground truth, and SLAM localization error
# (map->base_link vs ground truth; the robot spawns at the world origin so map == world at start).
# Ground truth: the robot model's world pose, read from `gz topic -e -t /world/<world>/dynamic_pose/info` (the ROS
# bridge drops entity names from Pose_V). usage: python3 nav_test.py out.csv world x1,y1 x2,y2 ...
import sys, time, math, csv, re, subprocess, threading, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from tf2_msgs.msg import TFMessage
import tf2_ros

NS = '/a200_0881'
rclpy.init(args=['--ros-args', '-r', f'/tf:={NS}/tf', '-r', f'/tf_static:={NS}/tf_static', '-p', 'use_sim_time:=true'])
n = Node('nav_test')
gt = {}
def read_gt(world):
    # text stream of Pose_V; take the pose block whose name is the robot model
    proc = subprocess.Popen(['gz', 'topic', '-e', '-t', f'/world/{world}/dynamic_pose/info'],
                            stdout=subprocess.PIPE, text=True, bufsize=1)
    cur, sect, v = None, None, {}
    for line in proc.stdout:
        line = line.strip()
        m = re.match(r'name: "(.*)"', line)
        if m: cur, v = m.group(1), {}; continue
        if cur != 'a200_0881/robot': continue
        if line in ('position {', 'orientation {'): sect = line[0]; continue
        m = re.match(r'([xyzw]): (\S+)', line)
        if m and sect: v[sect + m.group(1)] = float(m.group(2))
        if sect == 'o' and line == '}':
            qx, qy, qz, qw = (v.get('o' + k, 0.0) for k in 'xyzw')
            gt['p'] = (v.get('px', 0.0), v.get('py', 0.0),
                       math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
            cur = sect = None
threading.Thread(target=read_gt, args=(sys.argv[2],), daemon=True).start()
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
ac = ActionClient(n, NavigateToPose, NS + '/navigate_to_pose')
assert ac.wait_for_server(timeout_sec=30), 'navigate_to_pose not available'
t0 = time.time()
while 'p' not in gt and time.time() - t0 < 20: rclpy.spin_once(n, timeout_sec=0.1)
assert 'p' in gt, 'no ground truth from gz dynamic_pose/info'
while n.get_clock().now().nanoseconds == 0: rclpy.spin_once(n, timeout_sec=0.1)  # wait for /clock

def slam_pose():
    try:
        t = buf.lookup_transform('map', 'base_link', Time())
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    except Exception:
        return None

rows = []
for i, arg in enumerate(sys.argv[3:], 1):
    x, y = map(float, arg.split(','))
    g = NavigateToPose.Goal(); g.pose.header.frame_id = 'map'
    g.pose.pose.position.x, g.pose.pose.position.y, g.pose.pose.orientation.w = x, y, 1.0
    fut = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, fut); h = fut.result()
    ts, last, dist = n.get_clock().now().nanoseconds * 1e-9, gt['p'], 0.0
    tw = time.time(); res = h.get_result_async() if h.accepted else None
    while res and not res.done() and time.time() - tw < 180:
        rclpy.spin_once(n, timeout_sec=0.1)
        p = gt['p']; dist += math.hypot(p[0] - last[0], p[1] - last[1]); last = p
    if res is None: status = 'REJECTED'
    elif not res.done(): h.cancel_goal_async(); status = 'TIMEOUT'
    else: status = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(res.result().status, str(res.result().status))
    for _ in range(10): rclpy.spin_once(n, timeout_sec=0.05)
    p, s = gt['p'], slam_pose()
    loc_err = math.hypot(s[0] - p[0], s[1] - p[1]) if s else float('nan')
    yaw_err = abs((s[2] - p[2] + math.pi) % (2 * math.pi) - math.pi) * 180 / math.pi if s else float('nan')
    r = dict(goal=i, x=x, y=y, status=status, sim_s=round(n.get_clock().now().nanoseconds * 1e-9 - ts, 1),
             dist_m=round(dist, 2), goal_err_m=round(math.hypot(p[0] - x, p[1] - y), 2),
             loc_err_m=round(loc_err, 3), loc_yaw_err_deg=round(yaw_err, 2), gt_x=round(p[0], 2), gt_y=round(p[1], 2))
    rows.append(r); print(r, flush=True)
with open(sys.argv[1], 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
ok = sum(r['status'] == 'SUCCEEDED' for r in rows)
print(f"SUMMARY {ok}/{len(rows)} succeeded, total {sum(r['dist_m'] for r in rows):.1f} m, "
      f"max loc err {max(r['loc_err_m'] for r in rows):.3f} m")
