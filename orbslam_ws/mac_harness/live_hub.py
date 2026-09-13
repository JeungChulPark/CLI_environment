#!/usr/bin/env python3
"""The 127.0.0.1:8080 entry point of the live SLAM viewer.

Serves web/index.html and lets the page start and stop the backends: live_orbslam (port 8081) and
live_rtabmap (port 8082). The page then talks to the chosen backend directly (WebSocket + replay).

Only one backend tracks at a time, so the two never share the CPU during a real-time measurement.
Starting one stops whichever is still tracking; a finished backend stays up, idle, for replay.

    orbslam_ws/mac_harness/live_hub.py [--port 8080] [--bag BAG.db3] [--build DIR] [--out-root DIR]
"""
import argparse
import json
import os
import signal
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


class Backend:
    # status: idle -> loading (server up, SLAM initialising) -> running (camera started) -> finished
    #         (bag done, replay available); stopped = process gone
    def __init__(self, key, label, port, cmd):
        self.key, self.label, self.port, self.cmd = key, label, port, cmd
        self.proc = None
        self.status = "idle"
        self.out = None

    def info(self):
        return {"id": self.key, "label": self.label, "port": self.port, "status": self.status,
                "out": str(self.out) if self.out else None}

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
    ap.add_argument("--bag", default=DEFAULT_BAG)
    ap.add_argument("--build", default=str(WS / "output" / "mac_build"))
    ap.add_argument("--out-root", default=str(WS / "output"))
    ap.add_argument("--orb-settings", default=str(WS / "data" / "nf2000_C_early.yaml"))
    ap.add_argument("--backend-args", default="", help="extra flags for both backends, e.g. \"--limit 300\"")
    args = ap.parse_args()
    build, out_root = Path(args.build), Path(args.out_root)
    extra = args.backend_args.split()

    backends = {
        "orb": Backend("orb", "ORB-SLAM3", 8081, lambda bag, out: [
            str(build / "orb3" / "build" / "live_orbslam"), str(build / "ORBvoc.txt"), args.orb_settings, bag, str(out)] + extra),
        "rtab": Backend("rtab", "RTAB-Map", 8082, lambda bag, out: [
            str(build / "rtab" / "live_rtabmap"), bag, str(out)] + extra),
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
            return {"bag": Path(args.bag).name, "slams": [b.info() for b in backends.values()]}

        def do_GET(self):
            if urlparse(self.path).path == "/api/status":
                return self._json(self._status())
            return super().do_GET()

        def end_headers(self):
            self.send_header("Cache-Control", "no-store")
            super().end_headers()

        def do_POST(self):
            u = urlparse(self.path)
            key = parse_qs(u.query).get("slam", [""])[0]
            if key not in backends:
                return self._json({"error": f"unknown slam '{key}'"}, 400)
            with gate:
                if u.path == "/api/start":
                    for b in backends.values():  # never two trackers on the CPU at once
                        if b.key != key and b.tracking():
                            print(f"[hub] stopping {b.label} to start {backends[key].label}", flush=True)
                            b.halt()
                    backends[key].start(args.bag, out_root)
                    print(f"[hub] started {backends[key].label} -> {backends[key].out}", flush=True)
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
    print(f"[hub] viewer: http://127.0.0.1:{args.port}/  (bag {Path(args.bag).name})", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    sys.exit(main())
