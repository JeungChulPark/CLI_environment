#!/usr/bin/env python3
"""mount_yaw.py — the lidar's mounting yaw from manual driving (laptop, next to real_up.sh or real_loc_up.sh).

The robot can only move along its own x axis. While the WHEELS say "straight" (|v| above V_MIN, |w| below W_MAX), the
course of the lidar-based base_link pose must equal its heading. With a wrong base_link -> velodyne yaw the position is
still right (the lidar sits on the rotation axis, x = y = 0) but the heading is off, so on every straight stretch
    d = course - heading = LIDAR_YAW(assumed) - LIDAR_YAW(true)
(reversing: course + 180 deg). d is also the angle the lidar was turned about its own axis since the calibration in use,
i.e. the start value for the lidar -> camera registration (rt/calib_mount.py).

Pose source: hdl_graph_slam's scan-matching /odom while mapping, <ns>/localization/pose on a saved map; whichever
arrives. It also records the path driven in the MAP frame (map -> base_link TF, a point every 3 cm) to logs/track.json:
the "road" of the saved map (nav/real/road.py; new_place.sh copies it to maps/<name>_track.json).
Result, rewritten after every stretch: logs/mount_yaw.json
    {"n", "d_deg", "mad_deg", "lidar_yaw_assumed", "lidar_yaw_estimated", "ok", "segments": [...]}
env: NS (/a200_0881), LIDAR_YAW (the value the stack runs with; default from mount.env), MOUNT_YAW_OUT
"""
import json, math, os, sys, time
from collections import deque

V_MIN, W_MAX = 0.12, 0.06            # wheel odometry: m/s, rad/s
T_MIN, DIST_MIN = 1.5, 0.40          # a stretch: at least this long [s] and this far [m]
TRIM_HEAD, TRIM_TAIL = 0.4, 0.2      # s cut from both ends (the pose lags the wheels by about a scan)
TURN_MAX = math.radians(4)           # heading change allowed inside a stretch
N_MIN, MAD_MAX = 3, 3.0              # "ok": at least N stretches agreeing within MAD_MAX deg


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def stretch(poses, sign):
    """poses [(t, x, y, yaw)] of one straight stretch, sign +1 forward / -1 reverse -> course - heading [rad] or None"""
    if len(poses) < 5:
        return None
    t0, t1 = poses[0][0] + TRIM_HEAD, poses[-1][0] - TRIM_TAIL
    p = [q for q in poses if t0 <= q[0] <= t1]
    if len(p) < 4 or p[-1][0] - p[0][0] < T_MIN - TRIM_HEAD - TRIM_TAIL:
        return None
    dx, dy = p[-1][1] - p[0][1], p[-1][2] - p[0][2]
    if math.hypot(dx, dy) < DIST_MIN:
        return None
    if max(abs(wrap(q[3] - p[0][3])) for q in p) > TURN_MAX:
        return None
    heading = math.atan2(sum(math.sin(q[3]) for q in p), sum(math.cos(q[3]) for q in p))
    course = math.atan2(dy, dx) + (math.pi if sign < 0 else 0.0)
    return wrap(course - heading), math.hypot(dx, dy)


def summary(segs, yaw_assumed):
    """segs [{"d_deg", ...}] -> the result dict (circular median, so +179 / -179 do not cancel)"""
    out = {"n": len(segs), "d_deg": None, "mad_deg": None, "lidar_yaw_assumed": yaw_assumed, "lidar_yaw_estimated": None,
           "ok": False, "segments": segs[-60:]}
    if not segs:
        return out
    a = [math.radians(s["d_deg"]) for s in segs]
    ref = math.atan2(sum(math.sin(v) for v in a), sum(math.cos(v) for v in a))
    rel = sorted(wrap(v - ref) for v in a); med = rel[len(rel) // 2] if len(rel) % 2 else (rel[len(rel) // 2 - 1] + rel[len(rel) // 2]) / 2
    dev = sorted(abs(v - med) for v in rel); mad = dev[len(dev) // 2]
    d = wrap(ref + med)
    out.update(d_deg=round(math.degrees(d), 2), mad_deg=round(math.degrees(mad), 2),
               lidar_yaw_estimated=round(wrap(yaw_assumed - d), 4), ok=len(segs) >= N_MIN and math.degrees(mad) <= MAD_MAX)
    return out


class Collector:
    """feed wheel(t, v, w) and pose(t, x, y, yaw); .segs grows by one per straight stretch"""
    def __init__(self):
        self.poses = deque(maxlen=4000); self.segs = []; self.t_start = None; self.sign = 0; self.max_len = 4.0

    def pose(self, t, x, y, yaw):
        self.poses.append((t, x, y, yaw))

    def wheel(self, t, v, w):
        straight = abs(v) >= V_MIN and abs(w) <= W_MAX
        sign = 1 if v > 0 else -1
        if straight and self.t_start is None:
            self.t_start, self.sign = t, sign
        ended = self.t_start is not None and (not straight or sign != self.sign or t - self.t_start >= self.max_len)
        if ended:
            r = stretch([q for q in self.poses if self.t_start <= q[0] <= t], self.sign)
            if r is not None:
                self.segs.append({"t": round(self.t_start, 2), "dur_s": round(t - self.t_start, 2), "dist_m": round(r[1], 2),
                                  "dir": "fwd" if self.sign > 0 else "rev", "d_deg": round(math.degrees(r[0]), 2)})
            self.t_start = t if straight else None; self.sign = sign
            return r is not None
        return False


def default_yaw(here):
    v = os.environ.get("LIDAR_YAW")
    if v is None:
        try:
            v = next(l.split("=", 1)[1] for l in open(os.path.join(here, "mount.env")) if l.startswith("LIDAR_YAW="))
        except (OSError, StopIteration):
            v = "0"
    return float(v)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    here = os.path.dirname(os.path.abspath(__file__))
    ns = os.environ.get("NS", "/a200_0881"); yaw0 = default_yaw(here)
    out = os.environ.get("MOUNT_YAW_OUT", os.path.join(here, "logs", "mount_yaw.json"))
    col = Collector(); state = {"wheel": 0, "pose": 0, "src": None}
    import tf2_ros
    from rclpy.time import Time
    rclpy.init(args=["--ros-args", "-r", f"/tf:={ns}/tf", "-r", f"/tf_static:={ns}/tf_static"]); node = Node("mount_yaw")
    buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, node); track = []
    track_out = os.path.join(os.path.dirname(out), "track.json")

    def on_track():
        try:
            tr = buf.lookup_transform("map", "base_link", Time()).transform
        except Exception:                                    # noqa: BLE001  (no map frame yet)
            return
        q = tr.rotation; p = [round(tr.translation.x, 3), round(tr.translation.y, 3), round(yaw_of(q), 4)]
        if not track or math.hypot(p[0] - track[-1][0], p[1] - track[-1][1]) >= 0.03:
            track.append(p)

    def save_track():
        tmp = track_out + ".tmp"; json.dump(track, open(tmp, "w")); os.replace(tmp, track_out)

    def write():
        s = summary(col.segs, yaw0); s.update(wheel_msgs=state["wheel"], pose_msgs=state["pose"], pose_source=state["src"], wall=time.time())
        tmp = out + ".tmp"; json.dump(s, open(tmp, "w")); os.replace(tmp, out)
        return s

    def yaw_of(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_pose(m, src):
        if state["src"] not in (None, src):
            return                                           # one source per run (their frames differ)
        state["src"] = src; state["pose"] += 1; p = m.pose.pose
        col.pose(time.monotonic(), p.position.x, p.position.y, yaw_of(p.orientation))

    def on_wheel(m):
        state["wheel"] += 1
        if col.wheel(time.monotonic(), m.twist.twist.linear.x, m.twist.twist.angular.z):
            s = write(); g = col.segs[-1]
            node.get_logger().info(f"stretch {s['n']}: {g['dir']} {g['dist_m']} m, course - heading {g['d_deg']:+.1f} deg | "
                                   f"median {s['d_deg']:+.1f} deg (spread {s['mad_deg']:.1f}) -> LIDAR_YAW {s['lidar_yaw_estimated']}")

    node.create_subscription(Odometry, ns + "/platform/odom", on_wheel, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/odom", lambda m: on_pose(m, "hdl /odom"), 20)
    node.create_subscription(PoseWithCovarianceStamped, ns + "/localization/pose", lambda m: on_pose(m, "localization/pose"), 20)
    node.create_timer(5.0, lambda: (write(), save_track()))
    node.create_timer(0.1, on_track)
    write(); node.get_logger().info(f"mount_yaw: LIDAR_YAW in use {yaw0} rad, result -> {out}")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
