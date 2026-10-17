# stuck_guard.py — last-resort escape for Nav2 on the Husky (sim and real).
# While a NavigateToPose goal is executing, if the robot has moved < STUCK_DIST m and turned < STUCK_DEG for
# STUCK_S seconds, it plans a short escape from the live scan and drives it at low speed, then hands control back.
#
# Escape planning (GUARD_ARCS=1, default): motion primitives v in {+V, 0, -V} x w in {-2W..+2W} for up to ESC_T s
# are simulated with the bare Husky outline (+ margin) against the current scan points. Primitives that touch a point
# are dropped; the one that increases the clearance (smallest point-to-body gap) the most wins, ties -> reverse.
# This handles a robot pinned sideways (obstacle beside it: straight back/forward keeps the side gap, turning in
# place hits a corner) by arcing away. GUARD_ARCS=0 = straight forward/backward only (2026-10-01 behaviour).
#
# Commands go to twist_marker_server/cmd_vel, which twist_mux ranks above Nav2 (8 vs 1) and below the joystick (10),
# so the operator can always take over. Events are logged and published on <ns>/stuck_guard/event (std_msgs/String).
#   env: NS (/a200_0881), USE_SIM_TIME (true|false), SCAN_TOPIC (<ns>/scan), STUCK_S (6), STUCK_DIST (0.08),
#        STUCK_DEG (8), MAX_ESCAPES (3), GUARD_ARCS (1), NEAR_GAP (0.6 m: act only when the body is this close)
import math, os, json, numpy as np, rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatusArray, GoalStatus
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
import tf2_ros

NS = os.environ.get('NS', '/a200_0881')
SIM = os.environ.get('USE_SIM_TIME', 'true') == 'true'
SCAN_TOPIC = os.environ.get('SCAN_TOPIC', NS + '/scan')
STUCK_S = float(os.environ.get('STUCK_S', 6)); STUCK_DIST = float(os.environ.get('STUCK_DIST', 0.08))
STUCK_DEG = float(os.environ.get('STUCK_DEG', 8)); MAX_ESCAPES = int(os.environ.get('MAX_ESCAPES', 3))
ARCS = os.environ.get('GUARD_ARCS', '1') == '1'
# only act when something is actually close: in the 2026-10-02 sim A/B half the triggers were Nav2 pausing in open
# space (clearance > 3 m, e.g. during its own spin/wait recovery), where a blind 0.36 m reverse only adds noise
NEAR_GAP = float(os.environ.get('NEAR_GAP', 0.6))
BODY_HL, BODY_HW, MARGIN = 0.50, 0.335, 0.03      # bare Husky A200 half length / half width, contact margin
V, W, ESC_T, ESC_MAX_D = 0.12, 0.25, 3.0, 0.45     # escape speed (m/s), turn-rate step (rad/s), duration, max travel
STOP_GAP = 0.02                                    # stop an escape when the body gets this close to a point


def yaw_of(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi


def body_gap(px, py):
    """distance from points (base_link frame) to the bare body rectangle (0 inside)"""
    dx = np.maximum(np.abs(px) - BODY_HL, 0.0); dy = np.maximum(np.abs(py) - BODY_HW, 0.0)
    return np.hypot(dx, dy)


class StuckGuard(Node):
    def __init__(self):
        super().__init__('stuck_guard', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=SIM)])
        self.buf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.buf, self)
        self.active_goal = None; self.hist = []; self.escapes = 0; self.cooldown_until = 0.0
        self.pts = None      # latest scan as Nx2 points in base_link
        self.esc = None
        self.create_subscription(GoalStatusArray, NS + '/navigate_to_pose/_action/status', self.on_status,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, SCAN_TOPIC, self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(TwistStamped, NS + '/twist_marker_server/cmd_vel', 10)
        self.ev = self.create_publisher(String, NS + '/stuck_guard/event', 10)
        self.create_timer(0.1, self.tick)
        self.log('ready', stuck=f'<{STUCK_DIST} m & <{STUCK_DEG} deg in {STUCK_S} s', arcs=ARCS, scan=SCAN_TOPIC)

    def log(self, msg, **kw):
        self.get_logger().info(msg + (' ' + json.dumps(kw) if kw else ''))
        self.ev.publish(String(data=json.dumps(dict(t=round(self.now(), 2), msg=msg, **kw))))

    def now(self): return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, m):
        run = [s for s in m.status_list if s.status == GoalStatus.STATUS_EXECUTING]
        gid = bytes(run[-1].goal_info.goal_id.uuid).hex() if run else None
        if gid != self.active_goal:
            self.active_goal = gid; self.hist = []; self.escapes = 0
            if gid is None and self.esc: self.stop('goal ended')

    def on_scan(self, m):
        r = np.array(m.ranges); a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < min(m.range_max, 6.0))
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        if m.header.frame_id != 'base_link':      # planar offset of the scan frame (sensor arch in sim)
            try:
                t = self.buf.lookup_transform('base_link', m.header.frame_id, Time()).transform
                th = yaw_of(t.rotation); c, s = math.cos(th), math.sin(th)
                x, y = c * x - s * y + t.translation.x, s * x + c * y + t.translation.y
            except Exception:
                return
        self.pts = np.stack([x, y], 1)

    def pose(self):
        try:
            t = self.buf.lookup_transform('odom', 'base_link', Time()); r = t.transform
            return r.translation.x, r.translation.y, yaw_of(r.rotation)
        except Exception:
            return None

    # ---- escape planning on scan points ----
    def simulate(self, v, w, thr):
        """roll a (v, w) primitive forward; return (feasible, end clearance, travelled distance, duration).
        thr = smallest allowed gap along the way (MARGIN, or a bit below the current gap if already closer)"""
        P = self.pts; x = y = th = 0.0; dt = 0.1; travelled = 0.0; t = 0.0
        while t < ESC_T and travelled < ESC_MAX_D:
            x += v * math.cos(th) * dt; y += v * math.sin(th) * dt; th += w * dt; t += dt; travelled += abs(v) * dt
            c, s = math.cos(th), math.sin(th)
            dx, dy = P[:, 0] - x, P[:, 1] - y
            g = body_gap(c * dx + s * dy, -s * dx + c * dy)   # points in the moved body frame
            if g.min() < thr: return False, 0.0, travelled, t
            if v == 0.0 and abs(th) > math.radians(60): break
        return True, float(g.min()), travelled, t

    def plan_escape(self):
        g0 = float(body_gap(self.pts[:, 0], self.pts[:, 1]).min())
        prims = [(V, 0.0), (-V, 0.0)]
        if ARCS:
            prims += [(v, k * W) for v in (V, -V) for k in (-2, -1, 1, 2)] + [(0.0, k * W) for k in (-2, -1, 1, 2)]
        thr = min(MARGIN, max(g0 - 0.01, 0.0))
        res = []
        for v, w in prims:
            ok, g_end, d, t = self.simulate(v, w, thr)
            if ok and (d >= 0.15 or (v == 0.0 and t >= 1.0)):
                res.append((round(g_end - g0, 3) + (0.02 if v < 0 else 0.0), v, w, g_end, d, t))
        res.sort(reverse=True)
        return g0, res

    def send(self, v, w):
        m = TwistStamped(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.angular.z = v, w; self.pub.publish(m)

    def stop(self, why):
        self.send(0.0, 0.0); self.log('escape done', why=why); self.esc = None; self.hist = []
        self.cooldown_until = self.now() + 8.0

    def tick(self):
        p = self.pose()
        if p is None or self.pts is None: return
        t = self.now()
        if self.esc:
            e = self.esc
            if t > e['until']: return self.stop('primitive finished')
            # live check: predict 0.2 s ahead with the same command
            v, w = e['v'], e['w']; th = w * 0.2; dx = v * 0.2
            P = self.pts; c, s = math.cos(th), math.sin(th)
            g = body_gap(c * (P[:, 0] - dx) + s * P[:, 1], -s * (P[:, 0] - dx) + c * P[:, 1])
            if g.min() < STOP_GAP: return self.stop('too close')
            return self.send(v, w)
        if self.active_goal is None or t < self.cooldown_until: self.hist = []; return
        self.hist.append((t, p)); self.hist = [h for h in self.hist if t - h[0] <= STUCK_S + 0.5]
        if t - self.hist[0][0] < STUCK_S: return
        xs = [h[1] for h in self.hist]
        span = max(math.hypot(q[0] - xs[0][0], q[1] - xs[0][1]) for q in xs)
        turn = max(abs(wrap(q[2] - xs[0][2])) for q in xs)
        if span >= STUCK_DIST or turn >= math.radians(STUCK_DEG): return
        if self.escapes >= MAX_ESCAPES:
            self.log('stuck, escape budget used up for this goal', escapes=self.escapes); self.cooldown_until = t + 30; return
        g0, res = self.plan_escape()
        if g0 > NEAR_GAP:
            self.log('stalled but nothing close, leaving it to Nav2', clearance=round(g0, 2)); self.cooldown_until = t + 10; return
        top = [dict(gain=r[0], v=r[1], w=r[2], d=round(r[4], 2)) for r in res[:4]]
        if not res:
            self.log('stuck, no collision-free escape', clearance=round(g0, 3)); self.cooldown_until = t + 10; return
        gain, v, w, g_end, d, dur = res[0]
        self.escapes += 1
        self.esc = dict(v=v, w=w, until=t + dur)
        self.log('stuck -> escape', clearance_before=round(g0, 3), clearance_after=round(g_end, 3), v=v, w=w,
                 travel_m=round(d, 2), seconds=round(dur, 1), top=top, n=self.escapes)


def main():
    rclpy.init(args=['--ros-args', '-r', f'/tf:={NS}/tf', '-r', f'/tf_static:={NS}/tf_static'])
    rclpy.spin(StuckGuard())


if __name__ == '__main__':
    main()
