#!/usr/bin/env python3
"""rtab_live_stream.py — LIVE RTAB-Map (RGB-D) streamer for the object-pose hub, run on the SLAM host (laptop etri-rog).

    D455f (this host) -> realsense2_camera -> rtabmap_launch (rgbd_odometry + rtabmap) -> this bridge -> hub TCP

Same newline-JSON protocol and hub lane as orb_live_stream.py (hello / start / pose / stop / end).

Pose convention (identical to ORB-SLAM3 and to mac_slam/rtab_stream.cc):
  RTAB-Map runs with frame_id = camera_color_optical_frame, so every pose sent is
      T_w_cam = T_map_odom . T_odom_cam
  i.e. camera optical axes and world = the first camera's optical frame. (camera_link + the driver's static TF was the
  first design; this Humble bridge cannot deserialize the Jazzy driver's /tf_static — "sequence size exceeds remaining
  buffer" — so the optical frame is used directly and no TF from the driver is needed.) T_odom_cam is /rtabmap/odom;
  T_map_odom is RTAB-Map's loop-closure correction, taken from /rtabmap/mapGraph (map_to_odom). Frames where visual
  odometry is lost (covariance >= 9999) are not sent.

Keyframe anchoring — OFF by default, enable with --kf-anchoring (2026-09-17). With it off (the default, reverted on
request) every pose has ref_kf = null and no kf_update is sent: the hub stores objects in world coordinates, exactly as
before. Measured with it on (one lap, objpose/output/one_live_rtab_260917_155717): 61 loop closures and corrections up
to 25 cm were applied, but objects barely moved (90% <= 0.1 cm) and, with 538 updates in 10 min (459 node culls), the
displayed object position jittered ~2.5x more while the camera stood still (median 0.8 -> 2.3 mm).
When enabled (same protocol as lidar/hdl_stream.py), RTAB-Map graph nodes are the keyframes.
  * every /rtabmap/mapGraph (optimized node poses in the map frame + the map_to_odom used for them) becomes
      {"type":"kf_update","reason":"loop|optimize|periodic","kfs":[[node_id, T_w_node(16)], ...]}
    reason "loop" when the graph gained a loop-closure link, "optimize" when a known node moved > 1 mm, "culled" when
    nodes left the working memory; an unchanged graph (RTAB-Map republishes it every second) is not sent;
  * every pose carries ref_kf = the newest node of the latest graph and T_w_kf = its pose in that graph, with T_wc
    computed from the SAME graph's map_to_odom, so T_kf_c = T_w_kf^-1 . T_wc is consistent;
  * the hub stores objects relative to ref_kf, so when a later loop closure moves the nodes, objects registered
    earlier move with them and the object memory is rebuilt (hub.py / fusion.py / memory.py, unchanged).

Runtime (laptop): this bridge + RTAB-Map use system ROS 2 Humble (/opt/ros/humble + ~/CLI_environment/rtabmap_ws/install);
the RealSense driver too (ros-humble-realsense2-camera). All on ROS_DOMAIN_ID --domain (78) with default discovery, and
Fast DDS shared memory with 64 MB segments (fastdds_shm_big.xml, exported by run_rtab_live.sh). Odometry rate at
640x480x30 with exact sync, measured on the laptop 2026-09-17: default SHM (512 kB segments) 1-3 Hz, UDP-only 6-9 Hz,
SHM 64 MB 27.8 Hz. The odometry itself takes 14-20 ms per frame; the limit was image transport, not computation.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

HOME = Path.home()
CONDA_SH = next((p for p in (HOME / "miniconda3/etc/profile.d/conda.sh", HOME / "anaconda3/etc/profile.d/conda.sh")
                 if p.exists()), HOME / "miniconda3/etc/profile.d/conda.sh")


# ── 4x4 helpers (no numpy dependency) ─────────────────────────────────────────
def T_from(q, t):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), t[0]],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), t[1]],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), t[2]],
            [0.0, 0.0, 0.0, 1.0]]


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def inv(T):
    R = [[T[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(R[i][k] * T[k][3] for k in range(3)) for i in range(3)]
    return [R[0] + [t[0]], R[1] + [t[1]], R[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def flat(T):
    return [round(float(v), 6) for row in T for v in row]


I4 = [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]


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


def say(msg, err=False):
    try:
        print(msg, file=sys.stderr if err else sys.stdout, flush=True)
    except (BrokenPipeError, OSError):
        pass


def stop_group(proc, name, wait_s=20.0):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=wait_s)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    say(f"[rtab_live] stopped {name}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True, help="only `live` (this host's D455f)")
    ap.add_argument("--connect", required=True, help="hub host:port")
    ap.add_argument("--domain", type=int, default=int(os.environ.get("ORB_LIVE_DOMAIN", "78")))
    ap.add_argument("--ws", default=str(HOME / "CLI_environment/rtabmap_ws"))
    ap.add_argument("--camera-serial", default="")
    ap.add_argument("--camera-wait-s", type=float, default=60.0)
    ap.add_argument("--rtab-wait-s", type=float, default=120.0, help="RTAB-Map up + first tracked odometry")
    ap.add_argument("--status-s", type=float, default=10.0)
    ap.add_argument("--kf-anchoring", action="store_true",
                    help="send RTAB-Map graph nodes as keyframes (ref_kf + kf_update); default off = world anchoring")
    a, _unknown = ap.parse_known_args()      # --slam/--features/--time-source/--mode/--rate from the hub are ignored
    if a.session.strip().lower() != "live":
        say("[rtab_live] only --session live is supported", err=True)
        return 2
    ws = Path(a.ws).expanduser()
    if not (ws / "install/setup.bash").exists():
        say(f"[rtab_live] missing: {ws / 'install/setup.bash'}", err=True)
        return 2
    run_dir = HOME / "objpose/live_orb/runs" / ("rtab_" + time.strftime("%y%m%d_%H%M%S"))
    run_dir.mkdir(parents=True, exist_ok=True)

    os.environ["ROS_DOMAIN_ID"] = str(a.domain)
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    os.environ.pop("ROS_AUTOMATIC_DISCOVERY_RANGE", None)
    env = dict(os.environ)

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())

    def start_camera(reset: bool):
        cam_args = (f"align_depth.enable:=true enable_sync:=true initial_reset:={'true' if reset else 'false'} "
                    "rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30"
                    + (f" serial_no:={a.camera_serial}" if a.camera_serial else ""))
        log = run_dir / ("realsense_reset.log" if reset else "realsense.log")
        # the driver runs on the SAME ROS 2 distro as RTAB-Map (system Humble, ros-humble-realsense2-camera 4.57.7):
        # with the Jazzy `realsense` conda driver, Humble RTAB-Map dropped most images ("sequence size exceeds remaining
        # buffer") and odometry ran at ~1.4 Hz (2026-09-17)
        cmd = ["bash", "-c", f"source /opt/ros/humble/setup.bash && exec ros2 launch realsense2_camera rs_launch.py {cam_args}"]
        p = subprocess.Popen(cmd, env=env, stdout=open(log, "w"), stderr=subprocess.STDOUT, start_new_session=True)
        say(f"[rtab_live] realsense driver pid={p.pid} reset={reset} (domain {a.domain}, log {log})")
        return p

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo
    from rtabmap_msgs.msg import MapGraph

    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = Node("objpose_rtab_live")
    lock = threading.Lock()
    st = {"info": None, "O": I4, "T_map_odom": I4, "odom": 0, "lost": 0, "sent": 0, "t0": None, "wall0": None,
          "lags": [], "loop_updates": 0,
          "nodes": {}, "ref_kf": None, "graphs": 0, "kf_updates": 0, "loops": 0, "max_move_m": 0.0}
    line_box = {"line": None}

    def on_info(m):
        if st["info"] is None:
            st["info"] = (list(m.k), m.width, m.height)

    def on_graph(m):
        """/rtabmap/mapGraph -> node poses (keyframes) + the map_to_odom they belong to -> kf_update to the hub"""
        tr, rr = m.map_to_odom.translation, m.map_to_odom.rotation
        T_mo = T_from((rr.x, rr.y, rr.z, rr.w), (tr.x, tr.y, tr.z))
        nodes = {}
        for nid, ps in zip(m.poses_id, m.poses):
            p, r = ps.position, ps.orientation
            nodes[int(nid)] = T_from((r.x, r.y, r.z, r.w), (p.x, p.y, p.z))
        loops = sum(1 for ln in m.links if int(ln.type) == 1)          # Link::kGlobalClosure
        with lock:
            prev = st["nodes"]
            moved = max((max(abs(nodes[k][i][3] - prev[k][i][3]) for i in range(3)) for k in nodes if k in prev), default=0.0)
            removed = any(k not in nodes for k in prev if k < (max(nodes) if nodes else -1))
            reason = "loop" if loops > st["loops"] else ("optimize" if moved > 1e-3 else ("culled" if removed else "periodic"))
            st["loops"] = max(st["loops"], loops)
            if any(abs(T_mo[i][j] - st["T_map_odom"][i][j]) > 1e-4 for i in range(3) for j in range(4)):
                st["loop_updates"] += 1
            st["T_map_odom"], st["nodes"] = T_mo, nodes
            st["ref_kf"] = max(nodes) if nodes else None
            st["graphs"] += 1
            st["max_move_m"] = max(st["max_move_m"], moved)
            line, started = line_box["line"], st["t0"] is not None
        # only a graph that actually changed is sent: every kf_update makes the hub rebuild the object memory from all
        # kept frames, and RTAB-Map republishes an unchanged graph every second while the rig stands still (the first run
        # rebuilt 52 times in 50 s with nothing moving). New nodes need no update: each pose carries its node's T_w_kf.
        if not a.kf_anchoring or line is None or not started or not nodes or reason == "periodic":
            return
        t_ns = int(m.header.stamp.sec) * 1_000_000_000 + int(m.header.stamp.nanosec)
        try:
            line.send({"type": "kf_update", "t_ns": t_ns, "reason": reason, "map_id": 0, "n": len(nodes),
                       "kfs": [[k] + flat(T) for k, T in sorted(nodes.items())], "max_move_m": round(moved, 4),
                       "loops": loops})
        except OSError:
            stop.set()
            return
        with lock:
            st["kf_updates"] += 1
        if reason != "periodic":
            say(f"[rtab_live] kf_update {reason}: {len(nodes)} nodes, max move {moved * 100:.1f} cm, loop links {loops}")

    def on_odom(m):
        cov0 = m.pose.covariance[0] if len(m.pose.covariance) else 0.0
        with lock:
            st["odom"] += 1
            if cov0 >= 9999.0:
                st["lost"] += 1
                return
            O, T_mo = st["O"], st["T_map_odom"]
            line, t0, wall0 = line_box["line"], st["t0"], st["wall0"]
            ref = st["ref_kf"] if a.kf_anchoring else None
            T_w_kf = st["nodes"].get(ref) if ref is not None else None
        if O is None or line is None or t0 is None:
            return
        p, r = m.pose.pose.position, m.pose.pose.orientation
        T_ol = T_from((r.x, r.y, r.z, r.w), (p.x, p.y, p.z))
        T_w_cam = mul(inv(O), mul(T_mo, mul(T_ol, O)))
        t_ns = int(m.header.stamp.sec) * 1_000_000_000 + int(m.header.stamp.nanosec)
        lag = round((time.monotonic() - wall0) - (t_ns - t0) / 1e9, 4)
        try:
            line.send({"type": "pose", "t_ns": t_ns, "state": "OK", "T_wc": flat(T_w_cam), "track_ms": None,
                       "frame_idx": -1, "dropped": 0, "map_id": 0, "send_lag_s": lag, "map_changed": False,
                       "ref_kf": None if T_w_kf is None else int(ref),
                       "T_w_kf": None if T_w_kf is None else flat(T_w_kf), "kf_map_id": None if T_w_kf is None else 0})
        except OSError:
            stop.set()
            return
        with lock:
            st["sent"] += 1
            st["lags"].append(lag)

    rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(CameraInfo, "/camera/camera/color/camera_info", on_info, rel)
    node.create_subscription(MapGraph, "/rtabmap/mapGraph", on_graph, QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
    node.create_subscription(Odometry, "/rtabmap/odom", on_odom, 30)

    def spin():
        try:
            while not stop.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            pass
    threading.Thread(target=spin, daemon=True).start()

    cam = start_camera(reset=False)
    rtab = None
    line = None
    try:
        t_w = time.monotonic()
        while st["info"] is None and time.monotonic() - t_w < min(30.0, a.camera_wait_s) and not stop.is_set():
            time.sleep(0.2)
        if st["info"] is None and not stop.is_set():
            say("[rtab_live] no camera_info in 30 s: restarting the driver once with a hardware reset")
            stop_group(cam, "realsense driver", wait_s=10.0)
            time.sleep(2.0)
            cam = start_camera(reset=True)
            t_w = time.monotonic()
            while st["info"] is None and time.monotonic() - t_w < a.camera_wait_s and not stop.is_set():
                time.sleep(0.2)
        if st["info"] is None:
            say(f"[rtab_live] no camera_info from the D455f (see {run_dir})", err=True)
            return 1
        K, W, H = st["info"]
        say(f"[rtab_live] camera up: {W}x{H} fx={K[0]:.1f} fy={K[4]:.1f}")

        rtab_args = ("rgb_topic:=/camera/camera/color/image_raw "
                     "depth_topic:=/camera/camera/aligned_depth_to_color/image_raw "
                     "camera_info_topic:=/camera/camera/color/camera_info "
                     "frame_id:=camera_color_optical_frame approx_sync:=false "
                     "rtabmap_viz:=false rviz:=false "
                     f"database_path:={run_dir / 'rtabmap.db'} rtabmap_args:=--delete_db_on_start")
        rtab = subprocess.Popen(["bash", "-c", f"source /opt/ros/humble/setup.bash && source {ws / 'install/setup.bash'} && "
                                               f"exec ros2 launch rtabmap_launch rtabmap.launch.py {rtab_args}"],
                                env=env, stdout=open(run_dir / "rtabmap.log", "w"), stderr=subprocess.STDOUT,
                                start_new_session=True)
        say(f"[rtab_live] rtabmap_launch pid={rtab.pid} (log {run_dir / 'rtabmap.log'})")

        t_w = time.monotonic()
        while (st["odom"] - st["lost"] == 0 or st["O"] is None) and time.monotonic() - t_w < a.rtab_wait_s and not stop.is_set():
            if rtab.poll() is not None:
                say(f"[rtab_live] rtabmap exited rc={rtab.returncode} (see {run_dir / 'rtabmap.log'})", err=True)
                return 1
            time.sleep(0.2)
        if st["odom"] - st["lost"] == 0 or st["O"] is None:
            say(f"[rtab_live] no tracked odometry (odom {st['odom']}, lost {st['lost']}) or no camera TF "
                f"within {a.rtab_wait_s:.0f} s", err=True)
            return 1
        say(f"[rtab_live] RTAB-Map tracking after {time.monotonic() - t_w:.1f} s (odom {st['odom']}, lost {st['lost']})")

        host, port = a.connect.rsplit(":", 1)
        line = Line(host, int(port))
        now_ns = time.time_ns()
        line.send({"type": "hello", "source": "rtabmap_live", "features": None, "session": "live:D455f",
                   "first_ns": now_ns, "last_ns": now_ns + 3_600_000_000_000, "n_frames": 0, "live": True,
                   "time_source": "header", "time_domain": "colour header stamp (live, host clock)", "kf_updates": bool(a.kf_anchoring),
                   "params": {"backend": "rtabmap",
                              "keyframes": "RTAB-Map graph nodes (/rtabmap/mapGraph)" if a.kf_anchoring else "off (world anchoring)", "sensor": "RGB-D D455f 640x480x30", "odometry": "rgbd_odometry (F2M)",
                              "frame_id": "camera_color_optical_frame", "fx": K[0], "fy": K[4], "cx": K[2], "cy": K[5]}})
        line.s.settimeout(0.5)
        msg = {}
        while msg.get("type") != "start" and not stop.is_set():
            try:
                msg = line.recv()
            except socket.timeout:
                continue
        if stop.is_set():
            return 1
        with lock:
            st["t0"], st["wall0"] = int(msg["t0_ns"]), time.monotonic()
            line_box["line"] = line
        say(f"[rtab_live] start (live) t0_ns={st['t0']}")

        t_status = time.monotonic()
        while not stop.is_set():
            try:
                m = line.recv()
                if m.get("type") == "stop":
                    break
            except socket.timeout:
                pass
            except (ConnectionError, OSError):
                say("[rtab_live] hub closed the connection")
                break
            if rtab.poll() is not None:
                say(f"[rtab_live] rtabmap exited rc={rtab.returncode}", err=True)
                break
            if time.monotonic() - t_status >= a.status_s:
                t_status = time.monotonic()
                with lock:
                    lags = sorted(st["lags"][-300:])
                    med = lags[len(lags) // 2] if lags else None
                    say(f"[rtab_live] live +{time.monotonic() - st['wall0']:6.1f}s sent {st['sent']} poses | odom {st['odom']} "
                        f"(lost {st['lost']}) | nodes {len(st['nodes'])} graphs {st['graphs']} kf_updates {st['kf_updates']} "
                        f"loop links {st['loops']} max node move {st['max_move_m'] * 100:.1f} cm | map->odom changes "
                        f"{st['loop_updates']} | lag median {med} s")
        with lock:
            line_box["line"] = None
        try:
            line.s.settimeout(None)
            line.send({"type": "end", "stopped": stop.is_set(), "processed": st["odom"], "dropped": st["lost"],
                       "poses": st["sent"], "kf_updates": st["kf_updates"], "keyframes": len(st["nodes"]), "loops": st["loops"]})
        except OSError:
            pass
        say(f"[rtab_live] live done: poses sent {st['sent']} | odom {st['odom']} (lost {st['lost']})")
        return 0
    finally:
        stop.set()
        stop_group(rtab, "rtabmap")
        stop_group(cam, "realsense driver", wait_s=10.0)
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
