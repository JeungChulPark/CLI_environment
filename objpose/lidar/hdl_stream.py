#!/usr/bin/env python3
"""hdl_stream.py — hdl_graph_slam (VLP-16, ROS 2 Jazzy) streamer for the object-pose hub.

Same live protocol as slam_stream / lidar_stream (newline JSON over TCP, the hub owns the socket):
    -> {"type":"hello","source":"lidar_hdl_graph_slam", first_ns, last_ns, n_frames, kf_updates:true, ...}
    <- {"type":"start","t0_ns":..,"rate":1.0}
    -> {"type":"pose","t_ns":..,"state":"OK","T_wc":[16],"track_ms":..,"frame_idx":..,"dropped":..,
        "map_id":0,"map_changed":false,"ref_kf":k,"T_w_kf":[16],"kf_map_id":0}
    -> {"type":"kf_update","t_ns":..,"reason":"periodic|optimize|loop","map_id":0,"kfs":[[id, T16...], ...]}
    -> {"type":"end","processed":..,"dropped":..}

How it runs (all on the SLAM host, inside the `hdl_graph_slam_jazzy` conda env, see run_hdl_stream.sh):
  * `ros2 launch hdl_graph_slam hdl_graph_slam_501.launch.py` = prefiltering -> scan matching odometry
    (fast_gicp, 10 Hz, frame odom->velodyne) -> graph_slam_node (keyframes every --kf-trans m / --kf-angle rad,
    g2o every 2 s, loop closure, publishes map->odom and, added for this streamer,
    /hdl_graph_slam/keyframes = nav_msgs/Path with every keyframe's optimised pose).
  * this script replays the /velodyne_points scans of the rosbag2 sqlite file at bag rate (serialized
    messages are published as-is) and publishes /clock (use_sim_time), starting at the hub's t0.
  * every /odom message becomes a pose: T_wc = T_w_kf(ref) . T_odom_kf(ref)^-1 . T_odom_c, where ref is the
    latest keyframe before the scan; so an object anchored to ref moves with the graph correction of ref
    (the hub's keyframe anchoring, as with ORB-SLAM3). Before the first keyframe estimate arrives the
    pose is map->odom . T_odom_c with no keyframe (world anchoring).
  * each /hdl_graph_slam/keyframes message becomes a kf_update; reason "loop" when the loop-closure
    count increased, "optimize" when a keyframe moved more than 1 mm, else "periodic".
  * stamps are the velodyne header stamps of the converted bag (shared clock with the camera bags).
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import sqlite3
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path as PathMsg
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32
from hdl_graph_slam.msg import ScanMatchingOdometryDebug
from tf2_msgs.msg import TFMessage

HERE = Path(__file__).resolve().parent


def header_stamp(blob):
    sec, nsec = struct.unpack_from("<iI", blob, 4)
    return sec * 1_000_000_000 + nsec


def stamp_ns(st) -> int:
    return int(st.sec) * 1_000_000_000 + int(st.nanosec)


def pose_to_T(p) -> np.ndarray:
    from scipy.spatial.transform import Rotation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def tf_to_T(tr) -> np.ndarray:
    from scipy.spatial.transform import Rotation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]).as_matrix()
    T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    return T


def inv(T):
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def m16(T):
    return [round(float(v), 6) for v in np.asarray(T).reshape(-1)]


class Line:
    def __init__(self, host, port):
        self.s = socket.create_connection((host, port))
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.buf = b""
        self.lock = threading.Lock()

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


class Bridge(Node):
    """ROS side: scan/clock publishers, odometry + keyframe subscribers -> hub messages"""

    def __init__(self, a, line: Line):
        super().__init__("objpose_hdl_stream")
        self.a, self.line = a, line
        rel = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scan_pub = self.create_publisher(PointCloud2, a.topic, rel)
        self.clock_pub = self.create_publisher(Clock, "/clock", QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Odometry, "/odom", self.on_odom, rel)
        self.create_subscription(TFMessage, "/tf", self.on_tf, QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(PathMsg, "/hdl_graph_slam/keyframes", self.on_keyframes, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Int32, "/hdl_graph_slam/debug/loop_count", self.on_loops, latched)
        self.create_subscription(ScanMatchingOdometryDebug, "/scan_matching_odometry/debug", self.on_smo_debug, rel)
        self.reg_ms = None
        self.reg_ms_hist: list[float] = []
        self.smo_count = 0
        self.muted = True            # nothing goes to the hub before hello (warm-up scans)
        self.lock = threading.Lock()
        self.odom_by_t: dict[int, np.ndarray] = {}
        self.odom_t: list[int] = []
        self.T_map_odom = np.eye(4)
        self.kfs: dict[int, tuple[int, np.ndarray]] = {}       # id -> (stamp_ns, T_map_kf)
        self.kf_stamps: list[int] = []
        self.kf_ids: list[int] = []
        self.loop_count = 0
        self.loops_at_last_update = 0
        self.kf_updates = 0
        self.poses_sent = 0
        self.world_anchored = 0
        self.publish_wall: dict[int, float] = {}                 # scan stamp -> wall time it was published
        self.last_odom_t = 0
        self.frame_idx_of: dict[int, int] = {}
        self.dropped = 0
        self.t0 = None
        self.rate = 1.0
        self.wall0 = None
        self.lags: list[float] = []
        self.proc_ms: list[float] = []

    # ── hub timing helpers ───────────────────────────────────────────────
    def lag_s(self, t_ns):
        if self.wall0 is None:
            return None
        return round((time.monotonic() - self.wall0) - (t_ns - self.t0) / 1e9 / self.rate, 4)

    # ── ROS callbacks (executor thread) ──────────────────────────────────
    def on_tf(self, msg):
        for tr in msg.transforms:
            if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                with self.lock:
                    self.T_map_odom = tf_to_T(tr.transform)

    def on_loops(self, msg):
        self.loop_count = int(msg.data)

    def on_smo_debug(self, msg):
        self.reg_ms = float(msg.registration_time_ms)
        self.reg_ms_hist.append(self.reg_ms)
        self.smo_count = int(msg.odom_count)

    def on_keyframes(self, msg):
        rows, moved = [], 0.0
        with self.lock:
            for i, ps in enumerate(msg.poses):
                t = stamp_ns(ps.header.stamp)
                T = pose_to_T(ps.pose)
                old = self.kfs.get(i)
                if old is not None:
                    moved = max(moved, float(np.linalg.norm(T[:3, 3] - old[1][:3, 3])))
                self.kfs[i] = (t, T)
                rows.append([i] + m16(T))
            self.kf_ids = sorted(self.kfs, key=lambda k: self.kfs[k][0])
            self.kf_stamps = [self.kfs[k][0] for k in self.kf_ids]
        reason = "loop" if self.loop_count > self.loops_at_last_update else ("optimize" if moved > 1e-3 else "periodic")
        self.loops_at_last_update = self.loop_count
        t_ns = stamp_ns(msg.header.stamp)
        if self.muted:
            return
        self.line.send({"type": "kf_update", "t_ns": int(t_ns), "reason": reason, "map_id": 0, "kfs": rows,
                        "n": len(rows), "max_move_m": round(moved, 4), "loops": self.loop_count,
                        "send_lag_s": self.lag_s(t_ns)})
        self.kf_updates += 1
        if reason != "periodic":
            print(f"[hdl_stream] kf_update {reason}: {len(rows)} keyframes, max move {moved * 100:.1f} cm, "
                  f"loops {self.loop_count}", flush=True)

    def ref_keyframe(self, t_ns):
        """latest keyframe stamped <= t (caller holds the lock); (id, stamp, T_map_kf) or None"""
        import bisect
        k = bisect.bisect_right(self.kf_stamps, t_ns) - 1
        if k < 0:
            return None
        kid = self.kf_ids[k]
        return kid, self.kfs[kid][0], self.kfs[kid][1]

    def odom_at(self, t_ns):
        """odometry pose at exactly t (keyframe stamps are scan stamps), else nearest within 5 ms"""
        import bisect
        T = self.odom_by_t.get(t_ns)
        if T is not None:
            return T
        k = bisect.bisect_left(self.odom_t, t_ns)
        best = None
        for j in (k - 1, k):
            if 0 <= j < len(self.odom_t) and abs(self.odom_t[j] - t_ns) <= 5_000_000:
                if best is None or abs(self.odom_t[j] - t_ns) < abs(best - t_ns):
                    best = self.odom_t[j]
        return None if best is None else self.odom_by_t[best]

    def on_odom(self, msg):
        t_ns = stamp_ns(msg.header.stamp)
        T_oc = pose_to_T(msg.pose.pose)
        now = time.monotonic()
        with self.lock:
            import bisect
            if t_ns not in self.odom_by_t:
                bisect.insort(self.odom_t, t_ns)
            self.odom_by_t[t_ns] = T_oc
            self.last_odom_t = max(self.last_odom_t, t_ns)
            ref = self.ref_keyframe(t_ns)
            T_kf_o = None
            if ref is not None:
                T_o_kf = self.odom_at(ref[1])
                if T_o_kf is not None:
                    T_kf_o = ref[2] @ inv(T_o_kf)          # T_map_kf . T_odom_kf^-1 : odom -> map through ref
            if T_kf_o is None:
                T_wc = self.T_map_odom @ T_oc
                ref = None
                self.world_anchored += 1
            else:
                T_wc = T_kf_o @ T_oc
            key = t_ns // 1000
            pub_wall = self.publish_wall.pop(key, None)
            idx = self.frame_idx_of.pop(key, -1)
        if self.muted:
            self.poses_sent += 1          # warm-up: counted, not sent
            return
        proc = None if pub_wall is None else (now - pub_wall) * 1e3
        if proc is not None:
            self.proc_ms.append(proc)
        reg = self.reg_ms
        lag = self.lag_s(t_ns)
        if lag is not None:
            self.lags.append(lag)
        self.line.send({"type": "pose", "t_ns": int(t_ns), "state": "OK", "T_wc": m16(T_wc),
                        "track_ms": None if reg is None else round(reg, 2), "latency_ms": None if proc is None else round(proc, 1), "frame_idx": idx, "dropped": self.dropped,
                        "map_id": 0, "send_lag_s": lag, "map_changed": False,
                        "ref_kf": None if ref is None else int(ref[0]),
                        "T_w_kf": None if ref is None else m16(ref[2]), "kf_map_id": None if ref is None else 0})
        self.poses_sent += 1

    # ── clock ────────────────────────────────────────────────────────────
    def publish_clock(self, t_ns):
        c = Clock()
        c.clock.sec = int(t_ns // 1_000_000_000)
        c.clock.nanosec = int(t_ns % 1_000_000_000)
        self.clock_pub.publish(c)


def wait_for_nodes(node: Node, topic: str, timeout_s: float) -> bool:
    need = {"graph_slam_node", "scan_matching_odometry_node", "prefiltering_node"}
    t_end = time.monotonic() + timeout_s
    while time.monotonic() < t_end:
        names = {n for n, _ in node.get_node_names_and_namespaces()}
        subs = node.count_subscribers(topic)
        if need <= names and subs >= 1:
            return True
        time.sleep(0.25)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True, help="dataset lidar/ folder with the velodyne *.db3")
    ap.add_argument("--mode", default="live", choices=["live"])
    ap.add_argument("--connect", required=True)
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--offset-ns", type=int, default=0)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--topic", default="/velodyne_points")
    ap.add_argument("--domain", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "77")))
    ap.add_argument("--kf-trans", type=float, default=0.5, help="graph keyframe gate [m] (0807 runs: 0.5)")
    ap.add_argument("--kf-angle", type=float, default=0.25, help="graph keyframe gate [rad]")
    ap.add_argument("--far-thresh", type=float, default=30.0)
    ap.add_argument("--reg-threads", type=int, default=8)
    ap.add_argument("--floor", default="true", choices=["true", "false"], help="floor detection + constraints")
    ap.add_argument("--solver", default=os.environ.get("HDL_G2O_SOLVER", "lm_var"),
                    help="g2o solver (lm_var = csparse; lm_var_eigen if g2o has no csparse, e.g. a source build on the Mac)")
    ap.add_argument("--launch-wait-s", type=float, default=60.0)
    ap.add_argument("--warmup-s", type=float, default=20.0, help="before hello: publish the first scan until odometry answers (0 = off)")
    ap.add_argument("--tail-s", type=float, default=3.0, help="after the last scan: wait for odometry + a final optimisation")
    ap.add_argument("--dump-dir", default="", help="call /hdl_graph_slam/dump_graph here at the end (optimised keyframes)")
    ap.add_argument("--launch-log", default="", help="ros2 launch stdout/stderr (default: stderr of this process)")
    a, _unknown = ap.parse_known_args()       # accept and ignore slam_stream-only flags (--features ...)

    db = next(Path(a.session).expanduser().glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    tid = con.execute("select id from topics where name=?", (a.topic,)).fetchone()[0]
    rows = con.execute("select id, substr(data, 1, 16) from messages where topic_id=?", (tid,)).fetchall()
    frames = sorted((header_stamp(h) + a.offset_ns, mid) for mid, h in rows)
    print(f"[hdl_stream] {db.name} | {len(frames)} scans | topic {a.topic} | domain {a.domain}", flush=True)

    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(a.domain)
    env.setdefault("ROS_LOCALHOST_ONLY", "1")
    os.environ.update({k: env[k] for k in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY")})
    launch_cmd = ["ros2", "launch", "hdl_graph_slam", "hdl_graph_slam_501.launch.py", "use_sim_time:=true",
                  f"raw_points_topic:={a.topic}", "points_topic:=/filtered_points",
                  "raw_points_qos:=reliable", "filtered_points_qos:=reliable",
                  f"enable_floor_detection:={a.floor}", f"distance_far_thresh:={a.far_thresh}",
                  f"graph_keyframe_delta_trans:={a.kf_trans}", f"graph_keyframe_delta_angle:={a.kf_angle}",
                  f"scan_reg_num_threads:={a.reg_threads}", f"g2o_solver_type:={a.solver}"]
    lf = open(a.launch_log, "w") if a.launch_log else None
    launch = subprocess.Popen(launch_cmd, env=env, stdout=lf or sys.stderr, stderr=subprocess.STDOUT, start_new_session=True)
    print(f"[hdl_stream] launched pid={launch.pid}: {' '.join(launch_cmd)}", flush=True)

    # own SIGINT/SIGTERM handling (the hub stops a lane with SIGINT to the process group): finish the
    # protocol (end, graph dump) and stop the launch instead of letting rclpy tear the context down
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    host, port = a.connect.rsplit(":", 1)
    line = Line(host, int(port))
    node = Bridge(a, line)
    executor = (rclpy.executors.SingleThreadedExecutor() if os.environ.get("HDL_EXEC", "single") == "single"
                else rclpy.executors.MultiThreadedExecutor(num_threads=2))
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    def stop_launch():
        if launch.poll() is None:
            try:
                os.killpg(launch.pid, signal.SIGINT)
                launch.wait(timeout=15)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(launch.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    try:
        # sim time must exist before the nodes' timers start; hold it at the first scan until start
        clock_t = {"ns": frames[0][0] - 1_000_000_000}
        clock_stop = threading.Event()

        def clock_loop():
            while not clock_stop.is_set():
                if node.wall0 is not None:
                    clock_t["ns"] = int(node.t0 + (time.monotonic() - node.wall0) * node.rate * 1e9)
                node.publish_clock(clock_t["ns"])
                time.sleep(0.02)
        threading.Thread(target=clock_loop, daemon=True).start()
        t_launch_end = time.monotonic() + a.launch_wait_s

        while not stop.is_set() and not wait_for_nodes(node, a.topic, 1.0) and time.monotonic() < t_launch_end:
            pass
        if stop.is_set():
            return 1
        if not wait_for_nodes(node, a.topic, 0.5):
            print("[hdl_stream] hdl_graph_slam nodes did not come up", file=sys.stderr, flush=True)
            line.send({"type": "end", "processed": 0, "dropped": 0, "error": "hdl_graph_slam nodes did not start"})
            return 1
        print("[hdl_stream] hdl_graph_slam nodes up", flush=True)
        # warm-up BEFORE hello: the first scan through DDS + prefiltering + fast_gicp initialisation took ~10 s on the
        # Mac (91 scans dropped after start). Publish the first scan until an odometry message comes back, so the
        # pipeline is primed when the hub starts the clock. The map origin becomes this (stationary) first scan.
        if a.warmup_s > 0:
            blob0 = bytes(con.execute("select data from messages where id=?", (frames[0][1],)).fetchone()[0])
            t_w = time.monotonic()
            n_w = 0
            while node.poses_sent == 0 and time.monotonic() - t_w < a.warmup_s and not stop.is_set():
                node.scan_pub.publish(blob0)
                n_w += 1
                time.sleep(1.0)
            print(f"[hdl_stream] warm-up: {n_w} scans in {time.monotonic() - t_w:.1f} s, odometry {'ok' if node.poses_sent else 'NOT seen'}",
                  flush=True)
            node.poses_sent = 0
            node.world_anchored = 0
        node.muted = False

        line.send({"type": "hello", "source": "lidar_hdl_graph_slam", "features": None, "session": str(db),
                   "first_ns": frames[0][0], "last_ns": frames[-1][0], "n_frames": len(frames),
                   "time_source": "header", "time_domain": "velodyne header stamp", "kf_updates": True,
                   "params": {"backend": "hdl_graph_slam", "registration": "FAST_GICP", "kf_trans_m": a.kf_trans,
                              "kf_angle_rad": a.kf_angle, "far_thresh_m": a.far_thresh, "floor": a.floor == "true",
                              "optimize_interval_s": 2.0, "reg_threads": a.reg_threads, "solver": a.solver, "smooth_scans": 1}})
        line.s.settimeout(1.0)
        msg = {}
        while msg.get("type") != "start" and not stop.is_set():
            try:
                msg = line.recv()
            except socket.timeout:
                continue
        line.s.settimeout(None)
        if stop.is_set():
            return 1
        node.t0, node.rate = int(msg["t0_ns"]), float(msg.get("rate", 1.0))
        node.wall0 = time.monotonic()
        t0, rate, wall0 = node.t0, node.rate, node.wall0
        print(f"[hdl_stream] start t0_ns={t0} rate={rate}", flush=True)

        idx = next((i for i, f in enumerate(frames) if f[0] >= t0), len(frames))
        processed = 0
        timing = {"read": 0.0, "pub": 0.0, "n": 0}
        while idx < len(frames) and not stop.is_set():
            if a.max_frames and processed >= a.max_frames:
                break
            due = wall0 + (frames[idx][0] - t0) / 1e9 / rate
            now = time.monotonic()
            if due > now:
                time.sleep(min(due - now, 0.02))
                continue
            j = idx
            while j + 1 < len(frames) and wall0 + (frames[j + 1][0] - t0) / 1e9 / rate <= now:
                j += 1
            node.dropped += j - idx
            idx = j
            t_ns, mid = frames[idx]
            t_a = time.perf_counter()
            blob = con.execute("select data from messages where id=?", (mid,)).fetchone()[0]
            t_b = time.perf_counter()
            with node.lock:
                node.publish_wall[t_ns // 1000] = time.monotonic()
                node.frame_idx_of[t_ns // 1000] = idx
            node.scan_pub.publish(bytes(blob))
            t_c = time.perf_counter()
            timing["read"] += t_b - t_a; timing["pub"] += t_c - t_b; timing["n"] += 1
            processed += 1
            idx += 1
            if processed % int(os.environ.get("HDL_LOG_EVERY", "100")) == 0:
                print(f"[hdl_stream] timing per scan: read {timing['read'] / timing['n'] * 1e3:.1f} ms publish "
                      f"{timing['pub'] / timing['n'] * 1e3:.1f} ms", flush=True)
                print(f"[hdl_stream] +{(t_ns - t0) / 1e9:6.1f}s published {processed} dropped {node.dropped} | odom {node.poses_sent} "
                      f"(smo {node.smo_count}) reg {node.reg_ms} ms | keyframes {len(node.kfs)} loops {node.loop_count} "
                      f"| lag {node.lag_s(t_ns)} s", flush=True)
        last_t = frames[idx - 1][0] if idx > 0 else t0
        # let odometry catch up with the last scans and let the graph run one more optimisation tick
        t_end = time.monotonic() + a.tail_s
        while time.monotonic() < t_end and node.last_odom_t < last_t and not stop.is_set():
            time.sleep(0.05)
        if not stop.is_set():
            time.sleep(max(0.0, min(a.tail_s, t_end - time.monotonic())))
        if stop.is_set():
            print(f"[hdl_stream] stopped by signal after {processed} scans", flush=True)
        line.send({"type": "end", "stopped": stop.is_set(), "processed": processed, "dropped": node.dropped, "poses": node.poses_sent,
                   "kf_updates": node.kf_updates, "keyframes": len(node.kfs), "loops": node.loop_count,
                   "world_anchored_poses": node.world_anchored})
        lags = np.array(node.lags) if node.lags else np.zeros(1)
        pm = np.array(node.proc_ms) if node.proc_ms else np.zeros(1)
        rm = np.array(node.reg_ms_hist) if node.reg_ms_hist else np.zeros(1)
        print(f"[hdl_stream] live done: published {processed} dropped {node.dropped} | poses {node.poses_sent} "
              f"(world-anchored {node.world_anchored}) | keyframes {len(node.kfs)} loops {node.loop_count} "
              f"kf_updates {node.kf_updates} | registration mean {rm.mean():.1f} p99 {np.percentile(rm, 99):.1f} ms "
              f"| scan->odom latency mean {pm.mean():.1f} p99 {np.percentile(pm, 99):.1f} ms "
              f"| send lag median {np.median(lags) * 1e3:.0f} ms", flush=True)
        if a.dump_dir:
            from hdl_graph_slam.srv import DumpGraph
            cli = node.create_client(DumpGraph, "/hdl_graph_slam/dump_graph")
            if cli.wait_for_service(timeout_sec=5.0):
                req = DumpGraph.Request()
                req.destination = str(Path(a.dump_dir).resolve())
                fut = cli.call_async(req)
                t_wait = time.monotonic() + 60
                while not fut.done() and time.monotonic() < t_wait:
                    time.sleep(0.1)
                ok = fut.done() and fut.result() is not None and fut.result().success
                print(f"[hdl_stream] dump_graph -> {req.destination}: {'ok' if ok else 'FAILED'}", flush=True)
                with node.lock:
                    tum = [(t / 1e9, T) for _, (t, T) in sorted(node.kfs.items())]
                if tum:
                    from scipy.spatial.transform import Rotation
                    p = Path(a.dump_dir) / "keyframes_tum.txt"
                    p.parent.mkdir(parents=True, exist_ok=True)
                    with open(p, "w") as f:
                        for t, T in tum:
                            q = Rotation.from_matrix(T[:3, :3]).as_quat()
                            f.write(f"{t:.9f} {T[0, 3]:.6f} {T[1, 3]:.6f} {T[2, 3]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")
        return 0
    finally:
        if "clock_stop" in locals():
            clock_stop.set()
        stop_launch()
        # orderly ROS teardown: stop spinning, join, destroy the node, then shut the context down —
        # tearing the context down under a live spin thread aborts the interpreter at exit
        try:
            executor.shutdown(timeout_sec=2.0)
            spin.join(timeout=3.0)
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:                       # noqa: BLE001
            print(f"[hdl_stream] shutdown: {e}", file=sys.stderr, flush=True)
        if lf:
            lf.close()


if __name__ == "__main__":
    rc = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(int(rc or 0))      # skip interpreter teardown of DDS threads
