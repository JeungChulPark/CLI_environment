#!/usr/bin/env python3
"""loc_stream.py — robot pose from the fixed-map localizer (nav/real/map_localizer.py) for the object-pose hub.

Navigation mode (real_loc_up.sh) has no SLAM: map_localizer.py matches every VLP-16 scan against the saved global map
and publishes <ns>/localization/pose (map -> base_link, stamped with the scan). This streamer only forwards those
poses, with the same live protocol as lidar/lidar_stream.py (no keyframes: the map never changes, objects are anchored
to the map), so the hub's world frame is the saved map. Nothing is launched or stopped here.
    -> {"type":"hello","source":"lidar_map_localizer", ...}   <- {"type":"start","t0_ns":..,"rate":1.0}
    -> {"type":"pose","t_ns":..,"state":"OK","T_wc":[16], ...}  ->  {"type":"end", ...}
No pose is sent while the localizer is lost (it publishes only accepted matches). The stream ends when the hub closes
the socket or the localizer node disappears.
"""
import argparse, json, os, signal, socket, sys, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseWithCovarianceStamped


def pose_T(p):
    x, y, z, w = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


class Line:
    def __init__(self, host, port):
        self.s = socket.create_connection((host, port)); self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.buf = b""; self.lock = threading.Lock()

    def send(self, obj):
        data = (json.dumps(obj, separators=(",", ":")) + "\n").encode()
        with self.lock:
            self.s.sendall(data)

    def recv(self):
        while b"\n" not in self.buf:
            chunk = self.s.recv(65536)
            if not chunk:
                raise ConnectionError("hub closed the connection")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line)


def planar(T):
    """keep x, y and yaw only. The robot drives on a flat floor, but the scan matcher's z / roll / pitch wander
    (standing still: z +-3 cm, roll +-0.3 deg; driving: z range 38 cm, 2026-10-07), which shook the object boxes up and
    down in the camera image (the camera looks sideways, so robot roll is camera pitch)."""
    yaw = np.arctan2(T[1, 0], T[0, 0]); c, s = np.cos(yaw), np.sin(yaw)
    P = np.eye(4); P[:2, :2] = [[c, -s], [s, c]]; P[0, 3], P[1, 3] = T[0, 3], T[1, 3]
    return P


class Bridge(Node):
    def __init__(self, topic, line):
        super().__init__("objpose_loc_stream")
        self.line, self.topic = line, topic
        self.muted = True; self.n = 0; self.sent = 0; self.last_wall = None; self.last_t = 0
        self.t0 = None; self.rate = 1.0; self.wall0 = None; self.lags = []
        self.create_subscription(PoseWithCovarianceStamped, topic, self.on_pose, 20)

    def on_pose(self, m):
        t_ns = int(m.header.stamp.sec) * 1_000_000_000 + int(m.header.stamp.nanosec)
        self.n += 1; self.last_wall = time.monotonic(); self.last_t = t_ns
        if self.muted:
            return
        lag = None if self.wall0 is None else round((time.monotonic() - self.wall0) - (t_ns - self.t0) / 1e9 / self.rate, 4)
        if lag is not None:
            self.lags.append(lag)
        self.line.send({"type": "pose", "t_ns": t_ns, "state": "OK",
                        "T_wc": [round(float(v), 6) for v in (planar(pose_T(m.pose.pose)) if PLANAR else pose_T(m.pose.pose)).reshape(-1)],
                        "track_ms": None, "frame_idx": -1, "dropped": 0, "map_id": 0, "send_lag_s": lag,
                        "map_changed": False, "ref_kf": None, "T_w_kf": None, "kf_map_id": None})
        self.sent += 1


PLANAR = True


def main():
    global PLANAR
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", required=True)
    ap.add_argument("--planar", type=int, default=1, help="1 = send x, y, yaw only (z = 0, no roll / pitch); 0 = the raw 6-DoF match")
    ap.add_argument("--topic", default=os.environ.get("NS", "/a200_0881") + "/localization/pose")
    ap.add_argument("--first-pose-wait-s", type=float, default=90.0, help="wait for the first accepted match (global search) before hello")
    ap.add_argument("--status-s", type=float, default=10.0)
    a, _ = ap.parse_known_args()             # --session / --mode / --rate of the hub's common command line are not needed
    PLANAR = bool(a.planar)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    host, port = a.connect.rsplit(":", 1)
    line = Line(host, int(port)); node = Bridge(a.topic, line)
    ex = rclpy.executors.SingleThreadedExecutor(); ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True); spin.start()
    rc = 0
    try:
        print(f"[loc_stream] waiting for {a.topic}", flush=True)
        t_w = time.monotonic()
        while node.n == 0 and time.monotonic() - t_w < a.first_pose_wait_s and not stop.is_set():
            time.sleep(0.2)
        if node.n == 0:
            print("[loc_stream] no localization pose (is real_loc_up.sh running, is the robot localized?)", file=sys.stderr, flush=True)
            line.send({"type": "end", "processed": 0, "dropped": 0, "error": "no localization pose"}); return 1
        print(f"[loc_stream] first pose after {time.monotonic() - t_w:.1f} s", flush=True)
        now = time.time_ns()
        line.send({"type": "hello", "source": "lidar_map_localizer", "features": None, "session": "live:fixed map",
                   "first_ns": now, "last_ns": now + 3_600_000_000_000, "n_frames": 0, "live": True, "time_source": "header",
                   "time_domain": "velodyne scan stamp (live, host wall clock)", "kf_updates": False,
                   "params": {"backend": "map_localizer", "registration": "small_gicp vs saved map", "smooth_scans": 1,
                              "pose_frame": "base_link", "planar": PLANAR}})
        line.s.settimeout(1.0); msg = {}
        while msg.get("type") != "start" and not stop.is_set():
            try:
                msg = line.recv()
            except socket.timeout:
                continue
        if stop.is_set():
            return 1
        node.t0, node.rate, node.wall0 = int(msg["t0_ns"]), float(msg.get("rate", 1.0)), time.monotonic()
        node.muted = False
        print(f"[loc_stream] start t0_ns={node.t0}", flush=True)
        line.s.settimeout(0.5); t_status = time.monotonic(); t_gone = None
        while not stop.is_set():
            try:
                if line.recv().get("type") == "stop":
                    break
            except socket.timeout:
                pass
            except (ConnectionError, OSError):
                print("[loc_stream] hub closed the connection", flush=True); break
            if node.count_publishers(a.topic) == 0:
                t_gone = t_gone or time.monotonic()
                if time.monotonic() - t_gone > 5.0:
                    print("[loc_stream] the localizer is gone", flush=True); break
            else:
                t_gone = None
            if time.monotonic() - t_status >= a.status_s:
                t_status = time.monotonic()
                age = None if node.last_wall is None else round(time.monotonic() - node.last_wall, 1)
                print(f"[loc_stream] live +{time.monotonic() - node.wall0:6.1f}s | poses {node.sent} | last pose {age} s ago | "
                      f"lag {node.lags[-1] if node.lags else None} s", flush=True)
        try:
            line.send({"type": "end", "stopped": stop.is_set(), "processed": node.sent, "dropped": 0, "poses": node.sent})
        except OSError:
            pass
        print(f"[loc_stream] done: poses {node.sent}, send lag median "
              f"{(np.median(node.lags) * 1e3 if node.lags else 0):.0f} ms", flush=True)
        return rc
    finally:
        try:
            ex.shutdown(timeout_sec=2.0); spin.join(timeout=3.0); node.destroy_node(); rclpy.shutdown()
        except Exception as e:                       # noqa: BLE001
            print(f"[loc_stream] shutdown: {e}", file=sys.stderr, flush=True)


if __name__ == "__main__":
    rc = main()
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(int(rc or 0))
