#!/usr/bin/env python3
"""The 127.0.0.1:8080 entry point of the live SLAM viewer.

Serves web/index.html and lets the page start and stop the backends: live_orbslam (port 8081),
live_rtabmap (port 8082), and the LiDAR SLAMs through live_lidar.py: KISS-ICP (8083) and hdl_graph_slam (8084),
which need the session's lidar/ bag. The page then talks to the chosen backend directly (WebSocket + replay).

Only one backend tracks at a time, so the two never share the CPU during a real-time measurement.
Starting one stops whichever is still tracking; a finished backend stays up, idle, for replay.

    orbslam_ws/mac_harness/live_hub.py [--port 8080] [--bag BAG.db3] [--dataset DIR] [--build DIR] [--out-root DIR]

The page can also pick any session under --dataset that has a SLAM/*.db3 (the converted ROS 2 recordings).
Those come from three different D455F units, so for ORB-SLAM3 the hub writes a per-run settings yaml: the
--orb-settings file with the intrinsics and distortion replaced by the bag's own colour camera_info.
"""
import argparse
import json
import os
import re
import signal
import sqlite3
import struct
import subprocess
import sys
import threading
import time
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
WS = HERE.parent
DEFAULT_BAG = "/Users/user/Documents/DefenseMeta/Dataset/260826_etri_eightcircle_dark/SLAM/recording_20260826_164423.db3"
LIDAR_PY = str(Path.home() / "objpose" / "lidar" / "venv" / "bin" / "python")   # numpy, scipy, kiss_icp, cv2


def bag_label(bag):
    """<session>/SLAM/SLAM_0.db3 -> <session>; the SDK recording keeps its session name too"""
    p = Path(bag)
    return p.parent.parent.name if p.parent.name == "SLAM" else p.name


def find_bags(default_bag, dataset):
    bags = {bag_label(default_bag): str(default_bag)}
    root = Path(dataset).expanduser()
    if root.is_dir():
        for db in sorted(root.glob("*/SLAM/SLAM_*.db3")):
            bags.setdefault(bag_label(db), str(db))
    return bags


def session_dir(bag):
    return Path(bag).parent.parent


def has_lidar(bag):
    return any((session_dir(bag) / "lidar").glob("*.db3"))


def ros2_camera_info(bag):
    """colour camera_info of a converted ROS 2 bag -> (K[9], D[]) or None for the SDK layout"""
    con = sqlite3.connect(f"file:{bag}?immutable=1", uri=True)
    row = con.execute("select id from topics where name='/camera/camera/color/camera_info'").fetchone()
    if not row:
        return None
    b = bytes(con.execute("select data from messages where topic_id=? limit 1", row).fetchone()[0])
    o = 4 + 8
    al = lambda o, n: 4 + (o - 4 + n - 1) // n * n   # CDR alignment counts from after the encapsulation header
    o = al(o, 4); n = struct.unpack_from("<I", b, o)[0]; o += 4 + n
    o = al(o, 4) + 8
    n = struct.unpack_from("<I", b, o)[0]; o += 4 + n
    o = al(o, 4); n = struct.unpack_from("<I", b, o)[0]; o += 4
    o = al(o, 8); d = struct.unpack_from(f"<{n}d", b, o); o += 8 * n
    return struct.unpack_from("<9d", b, o), list(d) + [0.0] * (5 - len(d))


def settings_for(bag, base, out):
    """base settings with the bag's own intrinsics (converted bags); the SDK recording uses base as is"""
    ci = ros2_camera_info(bag)
    if ci is None:
        return base
    k, d = ci
    vals = {"Camera1.fx": k[0], "Camera1.fy": k[4], "Camera1.cx": k[2], "Camera1.cy": k[5],
            "Camera1.k1": d[0], "Camera1.k2": d[1], "Camera1.p1": d[2], "Camera1.p2": d[3], "Camera1.k3": d[4]}
    text = Path(base).read_text()
    for key, v in vals.items():
        text, n = re.subn(rf"^{re.escape(key)}:.*$", f"{key}: {v:.6f}", text, flags=re.M)
        assert n == 1, key
    dst = out / f"{Path(base).stem}_{bag_label(bag)}.yaml"
    head, _, rest = text.partition("\n")             # OpenCV needs %YAML:1.0 as the very first line
    dst.write_text(f"{head}\n# {Path(base).name} with the intrinsics of {bag}\n{rest}")
    return str(dst)


class Backend:
    # status: idle -> loading (server up, SLAM initialising) -> running (camera started) -> finished
    #         (bag done, replay available); stopped = process gone
    def __init__(self, key, label, port, cmd, needs=None):
        self.key, self.label, self.port, self.cmd, self.needs = key, label, port, cmd, needs
        self.proc = None
        self.status = "idle"
        self.out = None

    def info(self):
        return {"id": self.key, "label": self.label, "port": self.port, "status": self.status,
                "out": str(self.out) if self.out else None, "needs": self.needs}

    def start(self, bag, out_root):
        self.kill()
        self.out = out_root / f"mac_live_{self.key}_{time.strftime('%y%m%d_%H%M%S')}"
        self.out.mkdir(parents=True, exist_ok=True)
        env = dict(os.environ, OMP_NUM_THREADS=os.environ.get("OMP_NUM_THREADS", "6"))
        self.proc = subprocess.Popen(self.cmd(bag, self.out) + ["--port", str(self.port)], env=env,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        self.status = "loading"
        threading.Thread(target=self._pump, args=(self.proc, self.out / "run.log"), daemon=True).start()

    def _pump(self, proc, log_path):
        with open(log_path, "w") as log:
            for line in proc.stdout:
                log.write(line)
                log.flush()
                if proc is not self.proc:
                    continue
                if line.startswith("camera started"):
                    self.status = "running"
                elif line.startswith("bag finished") or line.startswith("tracking stopped early"):
                    self.status = "finished"
                    print(f"[hub] {self.label} finished: {self.out}", flush=True)
        proc.wait()
        if proc is self.proc:
            self.status = "stopped"

    def tracking(self):
        return self.proc is not None and self.proc.poll() is None and self.status in ("loading", "running")

    def halt(self, timeout=30):
        """Stop tracking early; outputs are written and the process stays up for replay."""
        p = self.proc
        if not self.tracking():
            return
        p.send_signal(signal.SIGINT)
        end = time.time() + timeout
        while self.tracking() and time.time() < end:
            time.sleep(0.1)

    def kill(self):
        """End the process (after it writes its outputs), e.g. before a new run replaces it."""
        p = self.proc
        if p is None or p.poll() is not None:
            return
        p.send_signal(signal.SIGTERM)
        try:
            p.wait(timeout=30)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
        self.status = "stopped"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--bag", default=DEFAULT_BAG, help="bag selected when the page opens")
    ap.add_argument("--dataset", default=str(Path.home() / "Documents/DefenseMeta/Dataset"),
                    help="sessions here with a SLAM/SLAM_*.db3 are offered on the page as well")
    ap.add_argument("--build", default=str(WS / "output" / "mac_build"))
    ap.add_argument("--out-root", default=str(WS / "output"))
    ap.add_argument("--orb-settings", default=str(WS / "data" / "nf2000_C_early.yaml"))
    ap.add_argument("--backend-args", default="", help="extra flags for both backends, e.g. \"--limit 300\"")
    args = ap.parse_args()
    build, out_root = Path(args.build), Path(args.out_root)
    extra = args.backend_args.split()
    bags = find_bags(args.bag, args.dataset)
    current = {"bag": bag_label(args.bag)}

    backends = {
        "orb": Backend("orb", "ORB-SLAM3", 8081, lambda bag, out: [
            str(build / "orb3" / "build" / "live_orbslam"), str(build / "ORBvoc.txt"),
            settings_for(bag, args.orb_settings, out), bag, str(out)] + extra),
        "rtab": Backend("rtab", "RTAB-Map", 8082, lambda bag, out: [
            str(build / "rtab" / "live_rtabmap"), bag, str(out)] + extra),
        "kiss": Backend("kiss", "KISS-ICP", 8083, lambda bag, out: [
            LIDAR_PY, str(HERE / "live_lidar.py"), "--slam", "kiss", "--session", str(session_dir(bag)), "--out", str(out)],
            needs="lidar"),
        "hdl": Backend("hdl", "hdl_graph_slam", 8084, lambda bag, out: [
            LIDAR_PY, str(HERE / "live_lidar.py"), "--slam", "hdl", "--session", str(session_dir(bag)), "--out", str(out)],
            needs="lidar"),
    }
    gate = threading.Lock()  # one start/stop at a time

    class Handler(SimpleHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _status(self):
            return {"bag": current["bag"], "bags": list(bags), "lidar": [k for k, v in bags.items() if has_lidar(v)],
                    "slams": [b.info() for b in backends.values()]}

        def do_GET(self):
            if urlparse(self.path).path == "/api/status":
                return self._json(self._status())
            return super().do_GET()

        def end_headers(self):
            self.send_header("Cache-Control", "no-store")
            super().end_headers()

        def do_POST(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            key = q.get("slam", [""])[0]
            if key not in backends:
                return self._json({"error": f"unknown slam '{key}'"}, 400)
            want = q.get("bag", [current["bag"]])[0]
            if want not in bags:
                return self._json({"error": f"unknown bag '{want}'"}, 400)
            if u.path == "/api/start" and backends[key].needs == "lidar" and not has_lidar(bags[want]):
                return self._json({"error": f"{want} 세션에는 LiDAR bag이 없어 {backends[key].label}을(를) 돌릴 수 없습니다"}, 400)
            with gate:
                if u.path == "/api/start":
                    for b in backends.values():  # never two trackers on the CPU at once
                        if b.key != key and b.tracking():
                            print(f"[hub] stopping {b.label} to start {backends[key].label}", flush=True)
                            b.halt()
                    current["bag"] = want
                    backends[key].start(bags[want], out_root)
                    print(f"[hub] started {backends[key].label} on {want} -> {backends[key].out}", flush=True)
                elif u.path == "/api/stop":
                    backends[key].halt()
                else:
                    return self._json({"error": "not found"}, 404)
            return self._json(self._status())

    server = ThreadingHTTPServer(("127.0.0.1", args.port), partial(Handler, directory=str(HERE / "web")))

    def shutdown(*_):
        for b in backends.values():
            b.kill()
        os._exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    print(f"[hub] viewer: http://127.0.0.1:{args.port}/  ({len(bags)} sessions, default {current['bag']})", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    sys.exit(main())
