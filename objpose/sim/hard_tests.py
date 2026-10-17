# Hard navigation scenarios on the Gazebo warehouse with the running Nav2 stack (sim_hdl_nav.sh or sim_nav.sh).
# Each scenario sets up the world (spawned boxes, teleports), sends goals, and records the outcome, timing,
# ground-truth arrival error, localization error (map->base_link vs Gazebo) and the Nav2 warnings/errors logged
# while it ran. usage: python3 hard_tests.py out_prefix [scenario ...]   (default: all)
import sys, time, math, csv, json, re, subprocess, threading, collections, os, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import TwistStamped
import tf2_ros

NS, WORLD, ROBOT = '/a200_0881', 'warehouse', 'a200_0881/robot'
NAV2_LOG = os.path.expanduser('~/objpose/sim/logs/nav2.log')
rclpy.init(args=['--ros-args', '-r', f'/tf:={NS}/tf', '-r', f'/tf_static:={NS}/tf_static', '-p', 'use_sim_time:=true'])
n = Node('hard_tests')
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
ac = ActionClient(n, NavigateToPose, NS + '/navigate_to_pose')
cmd = {'v': 0.0}
n.create_subscription(TwistStamped, NS + '/platform/cmd_vel', lambda m: cmd.update(v=abs(m.twist.linear.x) + abs(m.twist.angular.z)), 10)

gt = {}
def read_gt():
    proc = subprocess.Popen(['gz', 'topic', '-e', '-t', f'/world/{WORLD}/dynamic_pose/info'], stdout=subprocess.PIPE, text=True, bufsize=1)
    cur, sect, v = None, None, {}
    for line in proc.stdout:
        line = line.strip()
        m = re.match(r'name: "(.*)"', line)
        if m: cur, v = m.group(1), {}; continue
        if cur != ROBOT: continue
        if line in ('position {', 'orientation {'): sect = line[0]; continue
        m = re.match(r'([xyzw]): (\S+)', line)
        if m and sect: v[sect + m.group(1)] = float(m.group(2))
        if sect == 'o' and line == '}':
            qx, qy, qz, qw = (v.get('o' + k, 0.0) for k in 'xyzw')
            gt['p'] = (v.get('px', 0.0), v.get('py', 0.0), math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
            cur = sect = None
threading.Thread(target=read_gt, daemon=True).start()
assert ac.wait_for_server(timeout_sec=30)
t0 = time.time()
while ('p' not in gt or n.get_clock().now().nanoseconds == 0) and time.time() - t0 < 30: rclpy.spin_once(n, timeout_sec=0.1)

def spin(sec):
    t = time.time()
    while time.time() - t < sec: rclpy.spin_once(n, timeout_sec=0.05)

def sim_now(): return n.get_clock().now().nanoseconds * 1e-9

def slam_pose():
    try:
        t = buf.lookup_transform('map', 'base_link', Time()); q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    except Exception: return None

def gz_srv(name, reqtype, req):
    r = subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/{name}', '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '5000', '--req', req], capture_output=True, text=True)
    return 'true' in r.stdout

spawned = []
def box(name, x, y, sx, sy, sz=1.2, yaw=0.0):
    sdf = (f"<sdf version='1.9'><model name='{name}'><static>true</static><pose>{x} {y} {sz/2} 0 0 {yaw}</pose><link name='l'>"
           f"<collision name='c'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>"
           f"<visual name='v'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
           f"<material><diffuse>0.9 0.2 0.1 1</diffuse><ambient>0.9 0.2 0.1 1</ambient></material></visual></link></model></sdf>")
    ok = gz_srv('create', 'gz.msgs.EntityFactory', 'sdf: ' + json.dumps(sdf)); spawned.append(name); return ok

def remove_all():
    while spawned: gz_srv('remove', 'gz.msgs.Entity', f'name: "{spawned.pop()}" type: MODEL')

def teleport(x, y, yaw):
    return gz_srv('set_pose', 'gz.msgs.Pose',
                  f'name: "{ROBOT}" position {{ x: {x} y: {y} z: 0.15 }} orientation {{ z: {math.sin(yaw/2)} w: {math.cos(yaw/2)} }}')

def log_mark():
    try: return os.path.getsize(NAV2_LOG)
    except OSError: return 0

def log_issues(mark):
    with open(NAV2_LOG, errors='replace') as f:
        f.seek(mark); txt = re.sub(r'\x1b\[[0-9;]*m', '', f.read())
    c = collections.Counter()
    for line in txt.splitlines():
        if '[WARN]' in line or '[ERROR]' in line:
            msg = re.sub(r'^\[[^\]]+\] ', '', line); msg = re.sub(r'\[\d+\.\d+\] ', '', msg)
            msg = re.sub(r'-?\d+\.\d+', 'N', msg); c[msg[:170]] += 1
    return c

def send(x, y, yaw=0.0):
    g = NavigateToPose.Goal(); g.pose.header.frame_id = 'map'
    g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
    g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f); return f.result()

def wait(h, timeout, during=None):
    """wait for a goal; returns (status, sim_s, dist_m, stuck_s). during(t_wall) is called every loop (for events)."""
    if not h.accepted: return 'REJECTED', 0.0, 0.0, 0.0
    res = h.get_result_async(); ts, tw, last, dist, stuck, moving_t = sim_now(), time.time(), gt['p'], 0.0, 0.0, time.time()
    while not res.done() and time.time() - tw < timeout:
        rclpy.spin_once(n, timeout_sec=0.05)
        if during: during(time.time() - tw)
        p = gt['p']; d = math.hypot(p[0] - last[0], p[1] - last[1]); dist += d; last = p
        if d > 0.002: moving_t = time.time()
        stuck = max(stuck, time.time() - moving_t)
    if not res.done():
        h.cancel_goal_async(); spin(1.0); return 'TIMEOUT', sim_now() - ts, dist, stuck
    st = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(res.result().status, str(res.result().status))
    return st, sim_now() - ts, dist, stuck

def pose_errs(x=None, y=None, yaw=None):
    spin(0.5); p, s = gt['p'], slam_pose()
    e = {'gt_x': round(p[0], 2), 'gt_y': round(p[1], 2)}
    if x is not None: e['goal_err_m'] = round(math.hypot(p[0] - x, p[1] - y), 2)
    if yaw is not None: e['goal_yaw_err_deg'] = round(abs((p[2] - yaw + math.pi) % (2 * math.pi) - math.pi) * 180 / math.pi, 1)
    if s:
        e['loc_err_m'] = round(math.hypot(s[0] - p[0], s[1] - p[1]), 3)
        e['loc_yaw_err_deg'] = round(abs((s[2] - p[2] + math.pi) % (2 * math.pi) - math.pi) * 180 / math.pi, 2)
    return e

def go(x, y, yaw=0.0, timeout=180, during=None):
    h = send(x, y, yaw); st, s, d, stuck = wait(h, timeout, during)
    r = dict(status=st, sim_s=round(s, 1), dist_m=round(d, 2), max_stuck_s=round(stuck, 1)); r.update(pose_errs(x, y, yaw)); return r

# ---------------------------------------------------------------- scenarios
# each returns (expected, list of result dicts); the robot is brought to a start pose with a plain goal first.
def s_far_corner():
    return 'reach a far corner (~25 m, through the right hall)', [go(12, -20, timeout=240)]
def s_long_corridor():
    return 'reach the end of the long south corridor', [go(2, -17, timeout=240)]
def s_narrow_gap():
    # ~0.8 m gap between the two shelf rows at x~5.2 / 6.3 is narrower than the 0.9 m padded footprint
    return 'refuse or fail cleanly (gap narrower than footprint)', [go(5.75, -8, timeout=120)]
def s_goal_in_obstacle():
    return 'reject quickly / abort without driving into the shelf', [go(0.4, -2.0, timeout=90)]
def s_goal_outside():
    return 'fail cleanly (outside the building)', [go(22, 0, timeout=150)]
def s_turn_around():
    p = gt['p']; yaw = p[2] + math.pi
    return 'turn in place and stop 1 m behind with yaw 180 deg', [go(p[0] - math.cos(p[2]), p[1] - math.sin(p[2]), yaw, timeout=90)]
def s_tiny_goal():
    p = gt['p']
    return 'handle a goal 0.15 m away (inside tolerance)', [go(p[0] + 0.15 * math.cos(p[2]), p[1] + 0.15 * math.sin(p[2]), p[2], timeout=60)]
def s_popup_obstacle():
    # start (0,0) -> goal (10,0); a 1.2x1.2 m box appears at (5,0) 4 s after the start (not in the map)
    go(0, 0, 0.0, timeout=120)
    fired = {'b': False}
    def ev(t):
        if not fired['b'] and t > 4: fired['b'] = box('popup', 5.0, 0.0, 1.2, 1.2)
    r = go(10, 0, timeout=180, during=ev); r['note'] = f"box spawned={fired['b']}"; remove_all(); return 'detour around a box that appears on the path', [r]
def s_blocked_corridor():
    # wall across the corridor between the hall and the south corridor entrance -> no way to (2,-12)
    go(2, -3, timeout=150)
    box('wall', 2.0, -6.0, 5.0, 0.3)
    r = go(2, -12, timeout=150); remove_all(); return 'give up (no path) instead of pushing into the wall', [r]
def s_preempt():
    go(0, 0, 0.0, timeout=120)
    h1 = send(10, 0); spin(4.0)
    r = go(-8, 2, timeout=180); r['note'] = 'goal (10,0) preempted after 4 s'
    return 'switch to the new goal immediately', [r]
def s_kidnap():
    go(0, 0, 0.0, timeout=120)
    ok = teleport(4.0, 2.0, 1.2); spin(5.0)
    before = pose_errs(); r = go(0, 0, 0.0, timeout=150); r['note'] = f'teleport ok={ok}, loc err 5 s after teleport={before.get("loc_err_m")}'
    return 'notice or recover from a 4.5 m kidnap', [r]
def s_laps():
    rs = []
    for i in range(3):
        for (x, y) in ((10, 0), (11, -10), (2, -10), (-8, 2), (0, 0)):
            r = go(x, y, timeout=180); r['note'] = f'lap {i+1}'; rs.append(r)
    return '3 laps: no drift growth', rs
def s_clutter():
    return 'reach the cluttered table/chair corner', [go(-13.5, 4.0, timeout=180)]
def s_crowd_boxes():
    # slalom: four boxes staggered across the hall between (2,0) and (9,0)
    go(0, 0, 0.0, timeout=120)
    for i, (x, y) in enumerate(((3, 0.8), (4.5, -1.2), (6, 1.0), (7.5, -0.8))): box(f'slalom{i}', x, y, 0.6, 1.6)
    r = go(10, 0, timeout=200); remove_all(); return 'slalom through staggered boxes', [r]

def _park(x=0.0, y=3.0, yaw=0.0):
    # drive to an open spot first (no teleport: keeps localization valid)
    return go(x, y, yaw, timeout=150)
def s_nose_wall():
    # wide wall 0.3 m in front of the nose; goal straight behind it -> must back off / turn and go around
    _park(); p = gt['p']
    fx, fy = p[0] + 0.95 * math.cos(p[2]), p[1] + 0.95 * math.sin(p[2])
    box('nose_wall', fx, fy, 0.3, 3.0, 1.2, p[2])
    r = go(p[0] + 4.0 * math.cos(p[2]), p[1] + 4.0 * math.sin(p[2]), p[2], timeout=200); remove_all()
    return 'back off from a wall at the nose and go around it', [r]
def s_boxed():
    # U-shaped pocket: boxes in front and on both sides, open behind; goal ahead beyond the front box
    _park(); p = gt['p']; c, s_ = math.cos(p[2]), math.sin(p[2])
    at = lambda f, l: (p[0] + f * c - l * s_, p[1] + f * s_ + l * c)
    x, y = at(0.85, 0.0); box('pocket_front', x, y, 0.3, 1.8, 1.2, p[2])
    x, y = at(0.2, 0.68); box('pocket_left', x, y, 1.6, 0.3, 1.2, p[2])
    x, y = at(0.2, -0.68); box('pocket_right', x, y, 1.6, 0.3, 1.2, p[2])
    gx, gy = at(4.0, 0.0); r = go(gx, gy, p[2], timeout=240); remove_all()
    return 'reverse out of a U pocket (no room to turn) and reach a goal beyond it', [r]
def s_low_box():
    # 0.25 m box on the path: below the horizontal scan ring -> invisible to the costmaps (real-robot blind spot)
    _park(); p = gt['p']
    fx, fy = p[0] + 2.0 * math.cos(p[2]), p[1] + 2.0 * math.sin(p[2])
    box('low_box', fx, fy, 0.6, 1.4, 0.25, p[2])
    r = go(p[0] + 4.0 * math.cos(p[2]), p[1] + 4.0 * math.sin(p[2]), p[2], timeout=200); remove_all()
    return 'low obstacle the scan cannot see (documents the blind spot)', [r]

def _side_pin(goal_fwd, goal_left, label):
    # tall box 0.15 m beside the left side of the body (slightly ahead of centre), as on the real robot 2026-10-02;
    # turning left / in place hits a corner, straight motion keeps the side gap
    _park(); p = gt['p']; c, s_ = math.cos(p[2]), math.sin(p[2])
    at = lambda f, l: (p[0] + f * c - l * s_, p[1] + f * s_ + l * c)
    x, y = at(0.2, 0.335 + 0.15 + 0.15); box('side_pin', x, y, 1.0, 0.3, 1.5, p[2])
    gx, gy = at(goal_fwd, goal_left); r = go(gx, gy, p[2] + math.pi, timeout=200); remove_all()
    return label, [r]
def s_side_pin_backleft():
    return _side_pin(-2.5, 2.0, 'pinned on the left, goal back-left (needs a left turn)')
def s_side_pin_back():
    return _side_pin(-3.0, 0.0, 'pinned on the left, goal straight behind (needs a 180 deg turn)')
def s_side_pin_left():
    return _side_pin(0.5, 2.5, 'pinned on the left, goal to the left beyond the box')

ALL = collections.OrderedDict((k[2:], v) for k, v in globals().items() if k.startswith('s_'))
names = sys.argv[2:] or list(ALL)
rows = []
for name in names:
    mark = log_mark(); tw = time.time()
    try:
        expected, rs = ALL[name]()
    except Exception as e:
        expected, rs = 'error', [dict(status='HARNESS_ERROR', note=repr(e))]
        remove_all()
    issues = log_issues(mark)
    for r in rs:
        r = dict(scenario=name, expected=expected, **r); rows.append(r); print(json.dumps(r, ensure_ascii=False), flush=True)
    top = issues.most_common(6)
    rows[-1]['nav2_issues'] = ' | '.join(f'{c}x {m}' for m, c in top)
    print(f'  [{name}] {time.time()-tw:.0f}s wall, nav2 issues: ' + (rows[-1]['nav2_issues'] or 'none'), flush=True)
keys = []
for r in rows:
    for k in r:
        if k not in keys: keys.append(k)
with open(sys.argv[1] + '.csv', 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(rows)
print('csv:', sys.argv[1] + '.csv')
