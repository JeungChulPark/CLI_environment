#!/usr/bin/env python3
"""orb_live_stream.py — LIVE ORB-SLAM3 (RGB-D) streamer for the object-pose hub, run on the SLAM host (laptop etri-rog).

    D455f (this host) -> realsense2_camera -> orbslam3_ros2 rgbd_node -> /orbslam3/pose -> this bridge -> hub TCP

Same newline-JSON protocol as mac_slam/slam_stream and lidar/hdl_stream:
    -> {"type":"hello","source":"orbslam3_live", first_ns, last_ns, n_frames:0, live:true, kf_updates:false, ...}
    <- {"type":"start","t0_ns":..,"rate":1.0}
    -> {"type":"pose","t_ns":..,"state":"OK","T_wc":[16],"map_id":0,"ref_kf":null, ...}   (every tracked frame)
    <- {"type":"stop"}   or the hub closes the socket
    -> {"type":"end", ...}

T_wc = Tcw.inverse() as published by rgbd_node: camera (colour optical frame) -> ORB-SLAM3 world. Stamps are the
colour header stamps of this host's RealSense driver (host clock); the hub adds the host clock offset (--sam-tau-s).
The rgbd_node publishes no keyframes, so the hub anchors objects in world coordinates for this lane.

Everything runs in ROS_DOMAIN_ID --domain (78), so this host's /camera/camera/* never meets the hub PC's own D455f topics
(domain 72) over the cable. Discovery is NOT limited to localhost (see the comment in main()).

Envs (laptop): the bridge runs in `orbslam3` (rclpy, Jazzy); the driver in `realsense`; the node from
~/CLI_environment/orbslam_ws/install (built in `orbslam3`).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
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


def quat_pose_to_T16(p):
    x, y, z, w = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    R = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    t = [p.position.x, p.position.y, p.position.z]
    return [round(float(v), 6) for v in (R[0] + [t[0]] + R[1] + [t[1]] + R[2] + [t[2]] + [0.0, 0.0, 0.0, 1.0])]


def conda_cmd(env: str, command: str, extra_source: str = "") -> list[str]:
    src = f"source {extra_source} && " if extra_source else ""
    return ["bash", "-c", f"set +u; source {CONDA_SH} && conda activate {env} && {src}exec {command}"]


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
    try:
        print(f"[orb_live] stopped {name}", flush=True)
    except (BrokenPipeError, OSError):
        pass


def write_settings(template: Path, out: Path, K, D, width, height, features: int):
    """ORB-SLAM3 settings = the D455f template with this camera's own intrinsics and the requested feature count"""
    text = template.read_text()
    vals = {"Camera1.fx": K[0], "Camera1.fy": K[4], "Camera1.cx": K[2], "Camera1.cy": K[5],
            "Camera.width": int(width), "Camera.height": int(height), "ORBextractor.nFeatures": int(features)}
    d = list(D) + [0.0] * 5
    vals.update({"Camera1.k1": d[0], "Camera1.k2": d[1], "Camera1.p1": d[2], "Camera1.p2": d[3], "Camera1.k3": d[4]})
    for key, v in vals.items():
        fmt = f"{v}" if isinstance(v, int) else f"{float(v):.8f}"
        pat = re.compile(rf"^{re.escape(key)}:.*$", re.M)
        text = pat.sub(f"{key}: {fmt}", text) if pat.search(text) else text + f"\n{key}: {fmt}\n"
    out.write_text(text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True, help="only `live` (this host's D455f)")
    ap.add_argument("--connect", required=True, help="hub host:port")
    ap.add_argument("--features", type=int, default=2000)
    ap.add_argument("--domain", type=int, default=int(os.environ.get("ORB_LIVE_DOMAIN", "78")))
    ap.add_argument("--ws", default=str(HOME / "CLI_environment/orbslam_ws"))
    ap.add_argument("--settings-template", default="")
    ap.add_argument("--camera-serial", default="")
    ap.add_argument("--camera-wait-s", type=float, default=60.0)
    ap.add_argument("--orb-wait-s", type=float, default=180.0, help="vocabulary load + first tracked frame")
    ap.add_argument("--status-s", type=float, default=10.0)
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--bag-env", default="orbslam3",
                    help="conda env that plays the bag in bag mode. orbslam3 (default) = same Fast DDS 2.6 as rgbd_node; "
                         "realsense (Jazzy, Fast DDS 2.14) intermittently corrupted image sizes across versions "
                         "(Bad alloc deserializing, rgbd_node OOM-killed at 31 GB, 2026-09-17)")
    ap.add_argument("--bag-preroll-s", type=float, default=0.5,
                    help="bag mode: the paused player is positioned this far into the bag; hub.py starts at first + --start-s")
    a, _unknown = ap.parse_known_args()      # --slam/--time-source/--mode/--param from the hub are ignored
    live = a.session.strip().lower() == "live"
    bag = None if live else Path(a.session).expanduser()
    if bag is not None and not (bag / "metadata.yaml").exists():
        print(f"[orb_live] --session must be `live` or a converted RGB-D bag folder (metadata.yaml): {bag}", file=sys.stderr)
        return 2

    ws = Path(a.ws).expanduser()
    template = Path(a.settings_template).expanduser() if a.settings_template else ws / "data/orbslam3_d455f_640x480_rgbd.yaml"
    vocab = ws / "src/ORB_SLAM3/Vocabulary/ORBvoc.txt"
    for p in (ws / "install/setup.bash", template, vocab):
        if not p.exists():
            print(f"[orb_live] missing: {p}", file=sys.stderr)
            return 2
    run_dir = HOME / "objpose/live_orb/runs" / time.strftime("%y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)

    # isolate this host's ROS graph by DOMAIN only (the hub PC's camera is on domain 72). Do NOT restrict discovery to
    # localhost: the driver (`realsense` env, Fast DDS 2.14) and ORB-SLAM3/this bridge (`orbslam3` env, Fast DDS 2.6)
    # then never exchange data (shared-memory/loopback transports differ); with default discovery they talk at ~28 Hz
    # (measured 2026-09-17).
    os.environ["ROS_DOMAIN_ID"] = str(a.domain)
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    os.environ.pop("ROS_AUTOMATIC_DISCOVERY_RANGE", None)
    env = dict(os.environ)

    stop = threading.Event()
    # SIGHUP too: the hub runs this over ssh, and closing that session must still reach the `finally` below
    # (ORB-SLAM3 saves its trajectory, then the camera driver is stopped) instead of killing the bridge mid-way
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())

    def start_camera(reset: bool):
        # initial_reset is NOT the default: on this D455f (and the hub PC's) the stream opened right after the hardware
        # reset fails with "uvc streamer watchdog triggered / Depth stream start failure" (2026-09-17). Without the reset
        # the driver streams at ~28 Hz. The reset is only a fallback for a camera left wedged by a killed driver.
        cam_args = (f"align_depth.enable:=true enable_sync:=true initial_reset:={'true' if reset else 'false'} "
                    "rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30"
                    + (f" serial_no:={a.camera_serial}" if a.camera_serial else ""))
        log = run_dir / ("realsense_reset.log" if reset else "realsense.log")
        p = subprocess.Popen(conda_cmd("realsense", f"ros2 launch realsense2_camera rs_launch.py {cam_args}"), env=env,
                             stdout=open(log, "w"), stderr=subprocess.STDOUT, start_new_session=True)
        print(f"[orb_live] realsense driver pid={p.pid} reset={reset} (domain {a.domain}, log {log})", flush=True)
        return p

    cam = start_camera(reset=False) if live else None

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import CameraInfo
    from std_msgs.msg import String

    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = Node("objpose_orb_live")
    st = {"info": None, "states": 0, "not_tracking": 0, "poses": 0, "sent": 0, "t0": None, "wall0": None,
          "last_t": None, "lags": []}
    line_box = {"line": None}
    lock = threading.Lock()

    def on_info(m):
        if st["info"] is None:
            st["info"] = (list(m.k), list(m.d), m.width, m.height)

    def on_state(m):
        with lock:
            st["states"] += 1
            if m.data != "tracking" and m.data.lower() != "ok":
                st["not_tracking"] += 1 if m.data == "not_tracking" else 0

    def on_pose(m):
        t_ns = int(m.header.stamp.sec) * 1_000_000_000 + int(m.header.stamp.nanosec)
        with lock:
            st["poses"] += 1
            st["last_t"] = t_ns
            line, t0, wall0 = line_box["line"], st["t0"], st["wall0"]
        if line is None or t0 is None:
            return
        lag = round((time.monotonic() - wall0) - (t_ns - t0) / 1e9, 4)
        try:
            line.send({"type": "pose", "t_ns": t_ns, "state": "OK", "T_wc": quat_pose_to_T16(m.pose),
                       "track_ms": None, "frame_idx": -1, "dropped": 0, "map_id": 0, "send_lag_s": lag,
                       "map_changed": False, "ref_kf": None, "T_w_kf": None, "kf_map_id": None})
        except OSError:
            stop.set()
            return
        with lock:
            st["sent"] += 1
            st["lags"].append(lag)

    rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(CameraInfo, "/camera/camera/color/camera_info", on_info, rel)
    node.create_subscription(String, "/orbslam3/tracking_state", on_state, 30)
    node.create_subscription(PoseStamped, "/orbslam3/pose", on_pose, 30)

    def spin():
        try:
            while not stop.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            pass
    threading.Thread(target=spin, daemon=True).start()

    orb = None
    line = None
    try:
        # 1. camera intrinsics -> ORB-SLAM3 settings (bag mode: the converter's info.json next to the bag folder)
        if bag is not None:
            info = json.loads((bag.parent / "info.json").read_text())[bag.name]
            st["info"] = (list(info["K"]), list(info["D"]), int(info["color_size"][0]), int(info["color_size"][1]))
        t_w = time.monotonic()
        while st["info"] is None and time.monotonic() - t_w < min(30.0, a.camera_wait_s) and not stop.is_set():
            time.sleep(0.2)
        if st["info"] is None and not stop.is_set():
            print("[orb_live] no camera_info in 30 s: restarting the driver once with a hardware reset", flush=True)
            stop_group(cam, "realsense driver", wait_s=10.0)
            time.sleep(2.0)
            cam = start_camera(reset=True)
            t_w = time.monotonic()
            while st["info"] is None and time.monotonic() - t_w < a.camera_wait_s and not stop.is_set():
                time.sleep(0.2)
        if st["info"] is None:
            print(f"[orb_live] no camera_info from the D455f within {a.camera_wait_s:.0f} s (see {run_dir / 'realsense.log'})",
                  file=sys.stderr, flush=True)
            return 1
        K, D, W, H = st["info"]
        settings = run_dir / "orbslam3_live_settings.yaml"
        write_settings(template, settings, K, D, W, H, a.features)
        print(f"[orb_live] camera up after {time.monotonic() - t_w:.1f} s: {W}x{H} fx={K[0]:.1f} fy={K[4]:.1f} "
              f"cx={K[2]:.1f} cy={K[5]:.1f} -> {settings.name} (features {a.features})", flush=True)

        # 2. ORB-SLAM3 rgbd_node (no bag, no dense map, no map-point dumps)
        cfg = run_dir / "orbslam3_live_config.yaml"
        cfg.write_text(json.dumps({
            "dataset": "live_d455f",
            "bag": {"path": "", "play": False, "clock": False, "rate": 1.0, "start_delay": 0.0},
            "orbslam3": {"vocabulary_path": str(vocab), "settings_path": str(settings)},
            "topics": {"rgb": "/camera/camera/color/image_raw", "depth": "/camera/camera/aligned_depth_to_color/image_raw"},
            "frames": {"world": "map", "camera": "camera_color_optical_frame"},
            "runtime": {"sync_queue_size": 10, "publish_map_points": False, "save_map_points_on_shutdown": False},
            "dense_map": {"enabled": False, "save_pcd": False, "save_ply": False},
            "output": {"dir": str(run_dir / "orbslam3")},
            "visualization": {"enabled": False},
        }, indent=1))            # JSON is valid YAML
        orb = subprocess.Popen(conda_cmd("orbslam3", f"ros2 launch orbslam3_ros2 orb_slam.launch.py config:={cfg}",
                                         extra_source=str(ws / "install/setup.bash")),
                               env=env, stdout=open(run_dir / "orbslam3.log", "w"), stderr=subprocess.STDOUT,
                               start_new_session=True)
        print(f"[orb_live] orbslam3_ros2 rgbd_node pid={orb.pid} (log {run_dir / 'orbslam3.log'}), loading vocabulary ...",
              flush=True)

        # 3. warm-up: vocabulary loaded and frames being tracked (first tracking_state message)
        #    bag mode: no frames yet -> wait until rgbd_node subscribes to the colour topic (vocabulary loaded), then
        #    open the bag paused at first + preroll so it can start within milliseconds of the hub's start
        gate = None
        if bag is not None:
            t_w = time.monotonic()
            while node.count_subscribers("/camera/camera/color/image_raw") == 0 and time.monotonic() - t_w < a.orb_wait_s \
                    and not stop.is_set():
                if orb.poll() is not None:
                    print(f"[orb_live] rgbd_node exited rc={orb.returncode} (see {run_dir / 'orbslam3.log'})", file=sys.stderr, flush=True)
                    return 1
                time.sleep(0.2)
            time.sleep(2.0)       # the subscription appears slightly before the SLAM system finishes initialising
            print(f"[orb_live] ORB-SLAM3 subscribed after {time.monotonic() - t_w:.1f} s; opening {bag.name} paused", flush=True)
            here = Path(__file__).resolve().parent
            gate = subprocess.Popen(conda_cmd(a.bag_env, f"python -u {here / 'bag_player_gate.py'} --bag {bag} "
                                                            f"--start-offset {a.bag_preroll_s} --rate {a.rate} "
                                                            f"--log {run_dir / 'bag_play.log'}"),
                                    env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, start_new_session=True)
            first_line = gate.stdout.readline().strip()
            if first_line != "READY":
                print(f"[orb_live] bag player not ready: {first_line} (see {run_dir / 'bag_play.log'})", file=sys.stderr, flush=True)
                return 1
            md = (bag / "metadata.yaml").read_text()
            import re as _re
            start_ns = int(_re.search(r"starting_time:\s*\n\s*nanoseconds_since_epoch:\s*(\d+)", md).group(1))
            dur_ns = int(_re.search(r"duration:\s*\n\s*nanoseconds:\s*(\d+)", md).group(1))
            st["bag_first_ns"], st["bag_last_ns"] = start_ns, start_ns + dur_ns
        t_w = time.monotonic()
        while bag is None and st["states"] == 0 and time.monotonic() - t_w < a.orb_wait_s and not stop.is_set():
            if orb.poll() is not None:
                print(f"[orb_live] rgbd_node exited rc={orb.returncode} (see {run_dir / 'orbslam3.log'})", file=sys.stderr,
                      flush=True)
                return 1
            time.sleep(0.2)
        if bag is None and st["states"] == 0:
            print(f"[orb_live] ORB-SLAM3 produced no tracking state within {a.orb_wait_s:.0f} s", file=sys.stderr, flush=True)
            return 1
        print(f"[orb_live] ORB-SLAM3 running after {time.monotonic() - t_w:.1f} s (poses so far {st['poses']})", flush=True)

        # 4. hub handshake
        host, port = a.connect.rsplit(":", 1)
        line = Line(host, int(port))
        now_ns = time.time_ns()
        line.send({"type": "hello", "source": "orbslam3_live" if live else "orbslam3_bag", "features": a.features,
                   "session": "live:D455f" if live else str(bag),
                   "first_ns": now_ns if live else st["bag_first_ns"],
                   "last_ns": now_ns + 3_600_000_000_000 if live else st["bag_last_ns"], "n_frames": 0, "live": live,
                   "time_source": "header", "time_domain": "colour header stamp (live, host clock)" if live
                   else "colour header stamp (converted bag, shared clock)", "kf_updates": False,
                   "params": {"backend": "orbslam3", "sensor": "RGB-D D455f 640x480x30", "features": a.features,
                              "fx": K[0], "fy": K[4], "cx": K[2], "cy": K[5]}})
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
        if gate is not None:
            gate.stdin.write("go\n"); gate.stdin.flush()
            print(f"[orb_live] bag {gate.stdout.readline().strip()} | hub t0 = bag first + {(st['t0'] - st['bag_first_ns']) / 1e9:.3f} s "
                  f"(player positioned at +{a.bag_preroll_s:.3f} s)", flush=True)
            # end of bag: a flag, not `stop` — `stop` also ends the ROS spin thread, so the last poses would be lost
            threading.Thread(target=lambda: (gate.stdout.readline(), print("[orb_live] bag finished", flush=True),
                                             st.__setitem__("bag_done", time.monotonic())), daemon=True).start()
        print(f"[orb_live] start ({'live' if live else 'bag'}) t0_ns={st['t0']}", flush=True)

        # 5. forward poses (callback thread) until stop / EOF / signal
        t_status = time.monotonic()
        while not stop.is_set():
            try:
                m = line.recv()
                if m.get("type") == "stop":
                    break
            except socket.timeout:
                pass
            except (ConnectionError, OSError):
                print("[orb_live] hub closed the connection", flush=True)
                break
            if orb.poll() is not None:
                print(f"[orb_live] rgbd_node exited rc={orb.returncode}", file=sys.stderr, flush=True)
                break
            if st.get("bag_done") and time.monotonic() - st["bag_done"] > 3.0:     # let ORB-SLAM3 finish the last frames
                break
            if time.monotonic() - t_status >= a.status_s:
                t_status = time.monotonic()
                with lock:
                    lags = sorted(st["lags"][-300:])
                    med = lags[len(lags) // 2] if lags else None
                    print(f"[orb_live] live +{time.monotonic() - st['wall0']:6.1f}s sent {st['sent']} poses | frames "
                          f"{st['states']} (not tracking {st['not_tracking']}) | lag median {med} s", flush=True)
        with lock:
            line_box["line"] = None
        try:
            line.s.settimeout(None)
            line.send({"type": "end", "stopped": stop.is_set(), "processed": st["states"], "dropped": 0,
                       "poses": st["sent"], "not_tracking": st["not_tracking"]})
        except OSError:
            pass
        print(f"[orb_live] live done: poses sent {st['sent']} | frames {st['states']} (not tracking {st['not_tracking']})",
              flush=True)
        return 0
    finally:
        stop.set()
        if 'gate' in locals() and gate is not None:
            stop_group(gate, "bag player", wait_s=10.0)
        stop_group(orb, "orbslam3_ros2")
        stop_group(cam, "realsense driver", wait_s=10.0)
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
