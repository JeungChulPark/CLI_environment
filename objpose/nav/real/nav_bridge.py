#!/usr/bin/env python3
"""nav_bridge.py — Nav2 for the web viewer (objpose/pc/web/index.html), on the laptop next to the Nav2 stack.

The object-pose hub proxies /nav/* to this server (hub.py --nav-bridge), so one page shows the camera, the objects and
the navigation map, and sets goals. Plain HTTP + JSON, no ROS in the browser.
    GET  /meta          global map size / resolution / origin and its version
    GET  /map.png       global occupancy grid (row 0 = lowest y), RGBA
    GET  /costmap.png   local costmap (rolling window around the robot), RGBA, transparent where free
    GET  /state         robot pose, scan points, plan, footprint, costmap placement, goal state   (poll ~5 Hz)
    POST /goal    {"x","y","yaw"}    -> <ns>/goal_pose (what RViz "Nav2 Goal" does)
    POST /route   {"points": [{"x","y","yaw"}, ...], "loop": true}   -> drive the points in order, again and again:
                  the next one is sent ROUTE_PAUSE_S after Nav2 reports "succeeded"; a failed point is tried once more,
                  then the route stops. A single goal, a jog or /cancel ends the route.
    POST /cancel  {}                 -> cancel every goal of navigate_to_pose / backup / drive_on_heading / spin (and the route)
    POST /jog     {"kind": "fwd"|"back"|"left"|"right"}   0.3 m at 0.1 m/s or 30 deg, Nav2's own collision-checked behaviours
Road (2026-10-14): with ROAD_TRACK = <map>_track.json (the path driven by hand while mapping, nav/real/road.py) a goal
near that path is driven ALONG it with nav2 FollowPath (controller only; MPPI still avoids live obstacles) instead of a
fresh plan, which cut corners and came out in an S. Goals away from the road use NavigateToPose as before.
Detour: a road goal whose path ahead is blocked by a live obstacle for BLOCK_S (or that ends in "failed to make progress")
is handed to NavigateToPose (the planner sees live obstacles) to go round; the next goal uses the road again.
Curves: the robot answers a command ~0.6 s late (mostly its own acceleration), so ahead of a sharp bend of the path being
followed the controller gets a speed limit (<ns>/speed_limit): radius under 0.6 m -> 0.12 m/s, under 1.0 m -> 0.18 m/s.
Everything is in the map frame. env: NS (/a200_0881), NAV_BRIDGE_PORT (8770), ROUTE_PAUSE_S (1.0), ROAD_TRACK
"""
import json, math, os, struct, threading, time, zlib
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import BackUp, DriveOnHeading, FollowPath, NavigateToPose, Spin
from nav2_msgs.msg import SpeedLimit
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros

NS = os.environ.get("NS", "/a200_0881")
PORT = int(os.environ.get("NAV_BRIDGE_PORT", "8770"))
STATUS = {0: "unknown", 1: "accepted", 2: "executing", 3: "canceling", 4: "succeeded", 5: "canceled", 6: "aborted"}
JOG_M, JOG_SPEED, JOG_RAD = 0.30, 0.10, math.radians(30)
ROUTE_PAUSE_S = float(os.environ.get("ROUTE_PAUSE_S", "1.0"))
ROUTE_NEAR_M = 0.6                   # a route point Nav2 gave up on this close to the robot counts as reached
BLOCK_S, BLOCK_AHEAD_M, BLOCK_HALF_M = 1.0, 2.0, 0.45   # road blocked: lidar point within 0.45 m of the next 0.4-2.0 m of path
CURVE_LIMITS = ((0.6, 0.12), (1.0, 0.18))                 # (radius under m, speed m/s) for the next 0.8 m of path


def png_rgba(a):
    """HxWx4 uint8 -> PNG bytes"""
    h, w = a.shape[:2]
    raw = np.concatenate([np.zeros((h, 1), np.uint8), a.reshape(h, w * 4)], 1).tobytes()
    chunk = lambda t, d: struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 3)) + chunk(b"IEND", b""))


def se2_of(tr):
    q = tr.rotation
    return tr.translation.x, tr.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def apply(T, x, y):
    tx, ty, th = T
    c, s = math.cos(th), math.sin(th)
    return tx + c * x - s * y, ty + s * x + c * y


class Bridge(Node):
    def __init__(self):
        super().__init__("nav_bridge")
        self.lock = threading.Lock()
        self.map_png, self.map_meta, self.map_ver = None, None, 0
        self.cm_png, self.cm_meta, self.cm_ver = None, None, 0
        self.scan = None; self.plan = []; self.footprint = None
        self.T_map_base = None; self.T_map_odom = None; self.tf_wall = 0.0
        self.status = "idle"; self.dist = None; self.goal = None; self.jog = None
        self.route = None; self.route_gen = 0                # waypoint loop: points, idx, lap, loop, state, retry
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.buf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(OccupancyGrid, NS + "/map", self.on_map, latched)
        self.create_subscription(OccupancyGrid, NS + "/local_costmap/costmap", self.on_costmap, latched)
        self.create_subscription(LaserScan, NS + "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, NS + "/plan", self.on_plan, 5)
        self.create_subscription(PolygonStamped, NS + "/local_costmap/published_footprint", self.on_footprint, 5)
        self.create_subscription(GoalStatusArray, NS + "/navigate_to_pose/_action/status", self.on_status, latched)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage, NS + "/navigate_to_pose/_action/feedback", self.on_feedback, 5)
        self.loc_msg, self.loc_wall = None, 0.0
        self.create_subscription(String, NS + "/localization/status", self.on_loc_status, 10)
        self.goal_pub = self.create_publisher(PoseStamped, NS + "/goal_pose", 5)
        self.cancel = [self.create_client(CancelGoal, f"{NS}/{a}/_action/cancel_goal")
                       for a in ("navigate_to_pose", "follow_path", "backup", "drive_on_heading", "spin")]
        self.act = {"fwd": ActionClient(self, DriveOnHeading, NS + "/drive_on_heading"),
                    "back": ActionClient(self, BackUp, NS + "/backup"), "spin": ActionClient(self, Spin, NS + "/spin")}
        self.nav_cli = ActionClient(self, NavigateToPose, NS + "/navigate_to_pose")
        self.fp_cli = ActionClient(self, FollowPath, NS + "/follow_path")
        self.road = None; self.drive_gen = 0; self.mode = None
        self.cur = None                                      # the drive in progress: goal, done, path (map), handle, fallback used
        self.blocked_since = None; self.limit = 0.0
        self.speed_pub = self.create_publisher(SpeedLimit, NS + "/speed_limit", 5)
        self.create_timer(0.2, self.watch)
        tf_ = os.environ.get("ROAD_TRACK", "")
        if tf_ and os.path.exists(tf_):
            import sys; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from road import Road
            self.road = Road(json.load(open(tf_)))
            self.get_logger().info(f"road from {tf_}: {self.road.length:.1f} m, closed={self.road.closed}")
        self.create_timer(0.1, self.on_tf)

    # ── ROS side ───────────────────────────────────────────────────────────
    def on_tf(self):
        try:
            b = se2_of(self.buf.lookup_transform("map", "base_link", Time()).transform)
            o = se2_of(self.buf.lookup_transform("map", "odom", Time()).transform)
        except Exception:                                    # noqa: BLE001  (not localized yet)
            return
        with self.lock:
            self.T_map_base, self.T_map_odom, self.tf_wall = b, o, time.monotonic()

    def on_map(self, m):
        w, h = m.info.width, m.info.height
        g = np.frombuffer(bytes(m.data), np.int8).reshape(h, w)
        a = np.zeros((h, w, 4), np.uint8)
        a[g == 0] = (38, 51, 64, 255)                        # free
        a[g > 50] = (214, 224, 232, 255)                     # occupied
        a[(g > 0) & (g <= 50)] = (60, 72, 84, 255)
        png = png_rgba(a)                                    # unknown stays transparent
        res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
        js, is_ = np.nonzero(g > 50)                         # walls + 1 m, for the viewer's "fit" (most of the grid is empty)
        known = ([ox + is_.min() * res - 1, oy + js.min() * res - 1, ox + (is_.max() + 1) * res + 1, oy + (js.max() + 1) * res + 1]
                 if len(is_) else [ox, oy, ox + w * res, oy + h * res])
        with self.lock:
            self.map_ver += 1
            self.map_png = png
            self.map_meta = {"w": w, "h": h, "res": res, "ox": ox, "oy": oy, "ver": self.map_ver,
                             "known": [round(float(v), 2) for v in known],
                             "road": None if self.road is None else [[round(float(a), 2), round(float(b), 2)] for a, b in self.road.p[::2]]}

    def on_costmap(self, m):
        w, h = m.info.width, m.info.height
        g = np.frombuffer(bytes(m.data), np.int8).reshape(h, w).astype(np.int16)
        a = np.zeros((h, w, 4), np.uint8)
        infl = (g > 0) & (g < 99)
        t = (g[infl] / 98.0)[:, None]
        a[infl] = np.concatenate([(70 + 150 * t), (90 - 40 * t), (220 - 60 * t), (60 + 110 * t)], 1).astype(np.uint8)
        a[g == 99] = (60, 200, 220, 200)                     # inscribed: the robot centre must not be here
        a[g >= 100] = (255, 90, 190, 235)                    # obstacle
        png = png_rgba(a)
        with self.lock:
            self.cm_ver += 1
            self.cm_png = png
            self.cm_meta = {"w": w, "h": h, "res": m.info.resolution, "ox": m.info.origin.position.x,
                            "oy": m.info.origin.position.y, "frame": m.header.frame_id, "ver": self.cm_ver}

    def on_scan(self, m):
        r = np.asarray(m.ranges, np.float32)
        ang = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
        step = max(1, int(ok.sum()) // 500)
        with self.lock:
            self.scan = (m.header.frame_id, (r[ok] * np.cos(ang[ok]))[::step], (r[ok] * np.sin(ang[ok]))[::step])

    def on_plan(self, m):
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        step = max(1, len(pts) // 300)
        with self.lock:
            self.plan = [[round(x, 3), round(y, 3)] for x, y in pts[::step]] + ([[round(pts[-1][0], 3), round(pts[-1][1], 3)]] if pts else [])

    def on_footprint(self, m):
        with self.lock:
            self.footprint = (m.header.frame_id, [(p.x, p.y) for p in m.polygon.points])

    def on_status(self, m):
        if m.status_list:
            with self.lock:
                self.status = STATUS.get(m.status_list[-1].status, "unknown")
                if self.status in ("succeeded", "canceled", "aborted"):
                    self.plan, self.dist = [], None

    def on_loc_status(self, m):
        try:
            msg = json.loads(m.data).get("msg", "")
        except ValueError:
            msg = m.data
        if msg != "tracking":
            with self.lock:
                self.loc_msg, self.loc_wall = msg, time.monotonic()

    def on_feedback(self, m):
        with self.lock:
            self.dist = round(float(m.feedback.distance_remaining), 2)

    # ── HTTP side ──────────────────────────────────────────────────────────
    def state(self):
        with self.lock:
            b, o, age = self.T_map_base, self.T_map_odom, time.monotonic() - self.tf_wall
            out = {"robot": None if b is None else [round(b[0], 3), round(b[1], 3), round(b[2], 4)],
                   "pose_age_s": None if b is None else round(age, 2), "map_ver": self.map_ver,
                   "loc_event": None if self.loc_msg is None else [self.loc_msg, round(time.monotonic() - self.loc_wall, 1)],
                   "nav": {"status": self.status, "distance_remaining": self.dist, "goal": self.goal, "jog": self.jog, "mode": self.mode,
                           "route": None if self.route is None else {k: self.route[k] for k in ("points", "idx", "lap", "loop", "state")}},
                   "plan": self.plan, "scan": [], "footprint": [], "costmap": None}
            if b is None:
                return out
            frame_T = lambda f: b if f == "base_link" else o if f == "odom" else (0.0, 0.0, 0.0)
            if self.scan is not None:
                T = frame_T(self.scan[0]); c, s = math.cos(T[2]), math.sin(T[2])
                x, y = self.scan[1], self.scan[2]
                out["scan"] = np.round(np.column_stack([T[0] + c * x - s * y, T[1] + s * x + c * y]), 2).reshape(-1).tolist()
            if self.footprint is not None:
                T = frame_T(self.footprint[0])
                out["footprint"] = [[round(v, 3) for v in apply(T, x, y)] for x, y in self.footprint[1]]
            if self.cm_meta is not None:
                m = self.cm_meta; T = frame_T(m["frame"]); x0, y0 = apply(T, m["ox"], m["oy"])
                out["costmap"] = {"ver": m["ver"], "w": m["w"], "h": m["h"], "res": m["res"], "x": round(x0, 3), "y": round(y0, 3),
                                  "yaw": round(T[2], 4)}
            return out

    def send_goal(self, d):
        x, y, yaw = float(d["x"]), float(d["y"]), float(d.get("yaw", 0.0))
        with self.lock:
            if self.T_map_base is None or time.monotonic() - self.tf_wall > 2.0:
                return {"ok": False, "error": "로봇 위치를 모릅니다 (위치 추정 대기 중)"}
            self.goal = [round(x, 3), round(y, 3), round(yaw, 4)]
            self.route = None; self.route_gen += 1           # a single goal ends the route
        how = self.drive(x, y, yaw)
        self.get_logger().info(f"goal from the viewer: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg) via {how}")
        return {"ok": True, "goal": self.goal, "via": how}

    # ── one goal: along the road if it is near it, else Nav2's own planner ──
    def pose_stamped(self, x, y, yaw):
        m = PoseStamped(); m.header.frame_id = "map"; m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = x, y
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return m

    def drive(self, x, y, yaw, done=None, use_road=True):
        """send the robot to (x, y, yaw); done(status 4/5/6) when it ends. Returns "road" or "planner"."""
        with self.lock:
            b = self.T_map_base; self.drive_gen += 1; gen = self.drive_gen; self.blocked_since = None
        path = self.road.path_to(b[0], b[1], b[2], x, y, yaw) if (use_road and self.road is not None and b is not None) else None
        with self.lock:
            self.cur = {"gen": gen, "goal": (x, y, yaw), "done": done, "path": path, "handle": None, "how": None}
        if path is not None and self.fp_cli.server_is_ready():
            g = FollowPath.Goal(); g.controller_id = "FollowPath"; g.goal_checker_id = "general_goal_checker"
            g.progress_checker_id = "progress_checker"; g.path.header.frame_id = "map"; g.path.header.stamp = self.get_clock().now().to_msg()
            g.path.poses = [self.pose_stamped(px, py, ph) for px, py, ph in path]
            with self.lock:
                self.mode = "road"; self.status = "executing"; self.dist = None
                self.plan = [[round(px, 3), round(py, 3)] for px, py, _ in path[::2]] + [[round(path[-1][0], 3), round(path[-1][1], 3)]]
            fut = self.fp_cli.send_goal_async(g, feedback_callback=lambda f: self.on_fp_feedback(f, gen))
            how = "road"
        else:
            g = NavigateToPose.Goal(); g.pose = self.pose_stamped(x, y, yaw)
            with self.lock:
                self.mode = "planner"
            fut = self.nav_cli.send_goal_async(g); how = "planner"
        with self.lock:
            self.cur["how"] = how
        fut.add_done_callback(lambda f: self.drive_accepted(f, gen, how, done))
        return how

    def detour(self, why):
        """the road goal in progress -> NavigateToPose for the same goal (the planner goes round live obstacles)"""
        with self.lock:
            c = self.cur
            if c is None or c["how"] != "road":
                return
            h = c["handle"]; x, y, yaw = c["goal"]; done = c["done"]
        self.get_logger().info(f"road blocked ({why}): going round with the planner to ({x:.2f}, {y:.2f})")
        how = self.drive(x, y, yaw, done=done, use_road=False)     # new gen first, so the cancelled result is ignored
        if h is not None:
            h.cancel_goal_async()
        return how

    def watch(self):
        """every 0.2 s: speed limit ahead of sharp bends; blocked road -> detour"""
        with self.lock:
            b, sc, c, st = self.T_map_base, self.scan, self.cur, self.status
            path = (c or {}).get("path") if c and c["how"] == "road" else ([[p[0], p[1], 0.0] for p in self.plan] if self.plan else None)
        lim = 0.0
        if b is not None and path and len(path) > 3 and st in ("executing", "accepted"):
            P = np.array([[p[0], p[1]] for p in path]); i0 = int(np.argmin(np.hypot(P[:, 0] - b[0], P[:, 1] - b[1])))
            L = np.r_[0, np.cumsum(np.hypot(*np.diff(P[i0:], axis=0).T))]
            ahead = P[i0:][L <= 0.8]
            if len(ahead) >= 4:                                   # turn over the next 0.8 m -> radius
                h = np.unwrap(np.arctan2(np.diff(ahead[:, 1]), np.diff(ahead[:, 0])))
                turn = float(np.max(h) - np.min(h)); r = (L[len(ahead) - 1] / turn) if turn > 1e-3 else 1e9
                for rad, v in CURVE_LIMITS:
                    if r < rad:
                        lim = v; break
            if c and c["how"] == "road" and sc is not None:       # anything on the road ahead?
                seg = P[i0:][(L >= 0.4) & (L <= BLOCK_AHEAD_M)]
                T = b if sc[0] == "base_link" else None
                if T is not None and len(seg):
                    cc, ss = math.cos(T[2]), math.sin(T[2])
                    px = T[0] + cc * sc[1] - ss * sc[2]; py = T[1] + ss * sc[1] + cc * sc[2]
                    d = np.min(np.hypot(seg[:, 0][:, None] - px[None], seg[:, 1][:, None] - py[None]), axis=0)
                    hit = int((d < BLOCK_HALF_M).sum()) >= 3
                    now = time.monotonic()
                    with self.lock:
                        if hit and self.blocked_since is None: self.blocked_since = now
                        if not hit: self.blocked_since = None
                        go = hit and now - self.blocked_since >= BLOCK_S
                    if go:
                        self.detour("lidar points on the road ahead")
        if abs(lim - self.limit) > 1e-6:
            m = SpeedLimit(); m.header.stamp = self.get_clock().now().to_msg(); m.percentage = False; m.speed_limit = lim
            self.speed_pub.publish(m); self.limit = lim

    def on_fp_feedback(self, f, gen):
        with self.lock:
            if gen == self.drive_gen:
                self.dist = round(float(f.feedback.distance_to_goal), 2)

    def drive_accepted(self, fut, gen, how, done):
        h = fut.result()
        with self.lock:
            if self.cur and self.cur["gen"] == gen:
                self.cur["handle"] = h
        if not h.accepted:
            with self.lock:
                if gen == self.drive_gen and how == "road": self.status = "aborted"
            if done: done(6)
            return
        h.get_result_async().add_done_callback(lambda f: self.drive_done(f, gen, how, done))

    def drive_done(self, fut, gen, how, done):
        st = fut.result().status
        with self.lock:
            current = gen == self.drive_gen
            if current and how == "road":                     # FollowPath has no bt_navigator status topic: report it here
                self.status = STATUS.get(st, "unknown"); self.plan = []; self.dist = None
            if current:
                self.cur = None if st != 6 or how != "road" else self.cur
        if not current:
            return                                            # replaced (detour / new goal / cancel): its result means nothing
        if how == "road" and st == 6:
            self.get_logger().info(f"road goal ended: {fut.result().result.error_msg or 'aborted'}")
            if self.detour("the controller gave up") is not None:
                return
        if done: done(st)

    # ── waypoint loop ──────────────────────────────────────────────────────
    def start_route(self, d):
        pts = [[round(float(p["x"]), 3), round(float(p["y"]), 3), round(float(p.get("yaw", 0.0)), 4)] for p in d.get("points", [])]
        if len(pts) < 2:
            return {"ok": False, "error": "지점이 2개 이상 필요합니다"}
        if not self.nav_cli.server_is_ready():
            return {"ok": False, "error": "Nav2 주행 서버가 준비되지 않았습니다"}
        with self.lock:
            if self.T_map_base is None or time.monotonic() - self.tf_wall > 2.0:
                return {"ok": False, "error": "로봇 위치를 모릅니다 (위치 추정 대기 중)"}
            self.route_gen += 1
            self.route = {"points": pts, "idx": 0, "lap": 1, "loop": bool(d.get("loop", True)), "state": "running", "retry": 0}
            gen = self.route_gen
        self.get_logger().info(f"route from the viewer: {len(pts)} points, loop={bool(d.get('loop', True))}")
        self.route_send(gen)
        return {"ok": True, "points": len(pts)}

    def route_send(self, gen):
        with self.lock:
            r = self.route
            if r is None or gen != self.route_gen or r["state"] != "running":
                return
            x, y, yaw = r["points"][r["idx"]]; self.goal = [x, y, yaw]; i, lap = r["idx"], r["lap"]
        how = self.drive(x, y, yaw, done=lambda st: self.route_done(st, gen))
        self.get_logger().info(f"route: lap {lap}, point {i + 1} ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg) via {how}")

    def route_stop(self, gen, why):
        with self.lock:
            if self.route is not None and gen == self.route_gen:
                self.route["state"] = why
        self.get_logger().info(f"route ended: {why}")

    def route_done(self, status, gen):                        # 4 succeeded, 5 canceled, 6 aborted
        with self.lock:
            r = self.route
            if r is None or gen != self.route_gen or r["state"] != "running":
                return
            b = self.T_map_base; px, py, _ = r["points"][r["idx"]]
            if status == 6 and b is not None and math.hypot(b[0] - px, b[1] - py) < ROUTE_NEAR_M:
                # Nav2 gave up right next to the point (2026-10-14: "start occupied" 0.25 m from a point by a wall, the
                # route stopped after the first point): count it as reached and go on
                self.get_logger().info(f"route: point {r['idx'] + 1} given up {math.hypot(b[0] - px, b[1] - py):.2f} m away, counted as reached")
                status = 4
            if status == 4:
                r["retry"] = 0; r["idx"] += 1
                if r["idx"] >= len(r["points"]):
                    if not r["loop"]:
                        r["idx"] = len(r["points"]) - 1; r["state"] = "done"; return
                    r["idx"] = 0; r["lap"] += 1
            elif status == 6 and r["retry"] < 1:
                r["retry"] += 1                               # same point once more
            else:
                r["state"] = "failed" if status == 6 else "canceled"
                self.get_logger().info(f"route ended: {r['state']} at point {r['idx'] + 1}")
                return
        t = [None]

        def go():
            t[0].cancel(); self.destroy_timer(t[0]); self.route_send(gen)
        t[0] = self.create_timer(ROUTE_PAUSE_S, go)

    def cancel_all(self):
        with self.lock:
            if self.route is not None and self.route["state"] == "running":
                self.route["state"] = "canceled"
            self.route_gen += 1                              # results of the cancelled goal must not start the next point
            self.drive_gen += 1; self.cur = None
        for c in self.cancel:
            if c.service_is_ready():
                c.call_async(CancelGoal.Request())           # zero id + zero stamp = every goal
        with self.lock:
            self.goal, self.jog = None, None
            if self.mode == "road" and self.status in ("executing", "accepted"):
                self.status, self.plan, self.dist = "canceled", [], None
        self.get_logger().info("cancel from the viewer")
        return {"ok": True}

    def send_jog(self, d):
        kind = d.get("kind")
        with self.lock:
            if self.status in ("executing", "accepted", "canceling") or (self.route is not None and self.route["state"] == "running"):
                return {"ok": False, "error": "목표 주행 중입니다. 먼저 정지하세요"}
        if kind == "fwd":
            cli, g = self.act["fwd"], DriveOnHeading.Goal(); g.target.x, g.speed = JOG_M, JOG_SPEED
        elif kind == "back":
            cli, g = self.act["back"], BackUp.Goal(); g.target.x, g.speed = JOG_M, JOG_SPEED
        elif kind in ("left", "right"):
            cli, g = self.act["spin"], Spin.Goal(); g.target_yaw = JOG_RAD if kind == "left" else -JOG_RAD
        else:
            return {"ok": False, "error": "unknown jog"}
        g.time_allowance = DurationMsg(sec=12)
        if not cli.server_is_ready():
            return {"ok": False, "error": "Nav2 동작 서버가 준비되지 않았습니다"}
        cli.send_goal_async(g)
        with self.lock:
            self.jog = kind
        self.get_logger().info(f"jog from the viewer: {kind}")
        return {"ok": True}


def serve(node: Bridge):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def reply(self, body, ctype="application/json", code=HTTPStatus.OK):
            if not isinstance(body, (bytes, bytearray)):
                body = json.dumps(body, separators=(",", ":")).encode()
            self.send_response(code); self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body))); self.send_header("Cache-Control", "no-store"); self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            p = self.path.split("?")[0]
            if p == "/state":
                self.reply(node.state())
            elif p == "/meta":
                with node.lock:
                    m = node.map_meta
                self.reply(m or {"ver": 0})
            elif p in ("/map.png", "/costmap.png"):
                with node.lock:
                    b = node.map_png if p == "/map.png" else node.cm_png
                self.reply(b, "image/png") if b else self.send_error(HTTPStatus.NOT_FOUND)
            else:
                self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self):
            p = self.path.split("?")[0]
            try:
                d = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0)) or 0) or b"{}")
                r = (node.send_goal(d) if p == "/goal" else node.cancel_all() if p == "/cancel"
                     else node.send_jog(d) if p == "/jog" else node.start_route(d) if p == "/route" else None)
            except Exception as e:                           # noqa: BLE001
                r = {"ok": False, "error": str(e)}
            self.send_error(HTTPStatus.NOT_FOUND) if r is None else self.reply(r)

    srv = ThreadingHTTPServer(("0.0.0.0", PORT), H); srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    node.get_logger().info(f"nav bridge on :{PORT}")


def main():
    rclpy.init(args=["--ros-args", "-r", f"/tf:={NS}/tf", "-r", f"/tf_static:={NS}/tf_static"])
    node = Bridge()
    serve(node)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
