#!/usr/bin/env python3
"""launcher.py — one-at-a-time SLAM runner + viewer front door on http://192.168.20.5:8765 (2026-09-14).

    browser ── 8765 ── launcher.py ──proxy──► hub.py on 127.0.0.1:8766  (viewer page, /events SSE, /stats)
                          │  /api/state  /api/run?dataset=D&backend=B  /api/stop
                          └── runs `objpose/run_one.sh <dataset> <backend>` (ORB-SLAM3 on 192.168.20.3, or hdl_graph_slam on this PC);
                              starting a backend first stops the running one, so exactly one SLAM runs at a time.

The served page is the hub's own index.html (unchanged on disk) with a mode bar injected at the top
(web/launcher.js): a tab per INPUT DATASET (DATASETS below), the backend buttons of the active tab, and
which dataset/backend is running, in its colour. The page reloads itself when the run changes, so lanes of
a previous run never linger.
    python objpose/pc/launcher.py [--dataset 260826_etri_eightcircle_dark --backend orb] [--port 8765] [--hub-port 8766]
"""
from __future__ import annotations

import argparse
import http.client
import json
import os
import signal
import subprocess
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
WEB = HERE / "web"
RUN_ONE = REPO / "objpose" / "run_one.sh"

# One entry per INPUT DATASET; the launcher page shows a tab per dataset and, under it, that
# dataset's backend buttons. Backend keys only have to be unique inside a dataset.
ORB_FEATURES = [2000, 3000, 4000]     # feature counts selectable on the page for the ORB-SLAM3 backends
DATASETS = {
    "260910_object": {
        "label": "260910_object",
        "detail": "NAS data_slam_converted/260910_object · 카메라 + VLP-16",
        "backends": {
            # ORB backends carry "features": the page shows one f2000/f3000/f4000 button each; the chosen
            # value goes to run_one.sh as FEATURES (sweep 2026-09-14: .3 is at the 33 ms budget at f4000)
            "orb": {"label": "ORB-SLAM3 (192.168.20.3)", "host": "192.168.20.3", "color": "#56c2e0",
                    "features": ORB_FEATURES, "default_features": 3000,
                    "detail": "카메라 · Ubuntu 192.168.20.3 에서 실행, 포즈 17001 (f4000 은 추적 p99 32 ms 로 예산 한계)"},
            # "hdl" (hdl_graph_slam on this PC, pose 17002) removed from the page on the user's request (2026-09-14);
            # run_one.sh still accepts it from the command line.
            "orb-mac": {"label": "ORB-SLAM3 (Mac)", "host": "192.168.20.55", "color": "#56c2e0",
                        "features": ORB_FEATURES, "default_features": 4000,
                        "detail": "카메라 · Mac 에서 실행 (ssh mac), 포즈 17004"},
            # LiDAR: one card per host, one button per algorithm ("group" + "algo" -> the page renders a card
            # "LiDAR (host)" with KISS-ICP / hdl_graph_slam buttons, like the ORB feature-count buttons)
            "kiss-3": {"label": "LiDAR KISS-ICP (192.168.20.3)", "group": "LiDAR (192.168.20.3)", "algo": "KISS-ICP",
                       "host": "192.168.20.3", "color": "#f2a03d",
                       "detail": "VLP-16 · KISS-ICP 오도메트리 (루프 클로저 없음) · Ubuntu 192.168.20.3 (venv), 포즈 17007"},
            "hdl-3": {"label": "LiDAR hdl_graph_slam (192.168.20.3)", "group": "LiDAR (192.168.20.3)", "algo": "hdl_graph_slam",
                      "host": "192.168.20.3", "color": "#f2a03d",
                      "detail": "VLP-16 · hdl_graph_slam (그래프 + 루프 클로저) · Ubuntu 192.168.20.3 (ROS 2 conda), 포즈 17005"},
            "kiss-mac": {"label": "LiDAR KISS-ICP (Mac)", "group": "LiDAR (Mac)", "algo": "KISS-ICP",
                         "host": "192.168.20.55", "color": "#f2a03d",
                         "detail": "VLP-16 · KISS-ICP 오도메트리 (루프 클로저 없음) · Mac (venv), 포즈 17006"},
            "hdl-mac": {"label": "LiDAR hdl_graph_slam (Mac)", "group": "LiDAR (Mac)", "algo": "hdl_graph_slam",
                        "host": "192.168.20.55", "color": "#f2a03d",
                        "detail": "VLP-16 · hdl_graph_slam (그래프 + 루프 클로저) · Mac (ROS 2 micromamba), 포즈 17003"},
        },
    },
    "260826_etri_eightcircle_dark": {
        "label": "260826_etri_eightcircle_dark",
        "detail": "NAS data_slam_converted/260826_etri_eightcircle_dark · 카메라만 (lidar 폴더 없음), SAM τ −47.4 ms",
        "backends": {
            "orb": {"label": "ORB-SLAM3 (192.168.20.3)", "host": "192.168.20.3", "color": "#56c2e0",
                    "features": ORB_FEATURES, "default_features": 2000,
                    "detail": "카메라 · Ubuntu 192.168.20.3 에서 실행 (세션 ~/Dataset/260826_etri_eightcircle_dark/SLAM), 포즈 17001"},
            "orb-mac": {"label": "ORB-SLAM3 (Mac)", "host": "192.168.20.55", "color": "#56c2e0",
                        "features": ORB_FEATURES, "default_features": 2000,
                        "detail": "카메라 · Mac 에서 실행 (ssh mac, run_lan.sh 기본 세션), 포즈 17004"},
        },
    },
}
DEFAULT_DATASET = "260910_object"
BACKENDS = DATASETS[DEFAULT_DATASET]["backends"]   # kept for the old backend-only API / CLI


def resolve(dataset: str | None, backend: str, features: str | int | None = None):
    """(dataset, backend[, features]) -> (dataset key, feature count or None); empty dataset = the default (old API)."""
    ds = dataset or DEFAULT_DATASET
    if ds not in DATASETS or backend not in DATASETS[ds]["backends"]:
        raise ValueError(f"{ds}/{backend}")
    b = DATASETS[ds]["backends"][backend]
    if "features" not in b:
        return ds, None
    f = int(features) if features not in (None, "") else int(b["default_features"])
    if f not in b["features"]:
        raise ValueError(f"{ds}/{backend}/f{f}")
    return ds, f


def log(msg):
    print(time.strftime("%H:%M:%S"), "[launcher]", msg, flush=True)


class Runner:
    """owns at most one hub subprocess"""

    def __init__(self, hub_port: int, out_root: Path):
        self.hub_port = hub_port
        self.out_root = out_root
        self.lock = threading.Lock()
        self.proc: subprocess.Popen | None = None
        self.backend = None
        self.features = None                # ORB feature count of the running (or last) run
        self.dataset = DEFAULT_DATASET      # dataset of the running (or last) run
        self.run_id = 0
        self.started = None
        self.out = None
        self.last_exit = None
        self.busy = ""            # "stopping" / "starting"

    def state(self):
        alive = self.proc is not None and self.proc.poll() is None
        return {"run_id": self.run_id,
                "dataset": self.dataset if alive else None, "backend": self.backend if alive else None,
                "features": self.features if alive else None,
                "hub_alive": alive, "busy": self.busy, "started": self.started, "out": self.out,
                "last_dataset": self.dataset, "last_backend": self.backend, "last_features": self.features,
                "last_exit": self.last_exit,
                "datasets": DATASETS, "default_dataset": DEFAULT_DATASET,
                # "backends" = the current dataset's backends, so an older cached launcher.js still works
                "backends": DATASETS[self.dataset]["backends"], "hub_port": self.hub_port}

    def stop(self, wait_s=25.0):
        p = self.proc
        if p is None or p.poll() is not None:
            return
        self.busy = "stopping"
        log(f"stopping {self.backend} pid={p.pid}")
        try:
            os.killpg(p.pid, signal.SIGINT)          # hub writes summary.json on SIGINT
        except ProcessLookupError:
            pass
        t0 = time.time()
        while p.poll() is None and time.time() - t0 < wait_s:
            time.sleep(0.2)
        if p.poll() is None:
            log("hub did not exit on SIGINT, killing")
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            p.wait(5)
        self.last_exit = p.returncode
        # the hub's children (SAM-6D, local streamer) share its process group and got the same signal;
        # never pkill by name here — that would hit unrelated shells/tests whose command line mentions hub.py
        for f in Path("/dev/shm").glob("sam6d_*"):
            try:
                f.unlink()
            except OSError:
                pass
        self.busy = ""

    def start(self, dataset: str, backend: str, extra: list[str], features=None):
        dataset, features = resolve(dataset, backend, features)   # raises ValueError on an unknown combination
        with self.lock:
            self.stop()
            self.busy = "starting"
            self.run_id += 1
            self.dataset = dataset
            self.backend = backend
            self.features = features
            self.started = time.time()
            tag = f"{backend}_f{features}" if features else backend
            self.out = str(self.out_root / f"one_{dataset}_{tag}_{time.strftime('%y%m%d_%H%M%S')}")
            env = dict(os.environ, HTTP_PORT=str(self.hub_port), OUT=self.out)
            if features:
                env["FEATURES"] = str(features)          # run_one.sh: --features "${FEATURES:-<backend default>}"
            Path(self.out).mkdir(parents=True, exist_ok=True)
            logf = open(Path(self.out) / "hub_stdout.log", "w")
            self.proc = subprocess.Popen(["bash", str(RUN_ONE), dataset, backend, *extra], env=env, stdout=logf,
                                         stderr=subprocess.STDOUT, start_new_session=True, cwd=str(REPO))
            log(f"started {dataset}/{backend}{' f' + str(features) if features else ''} pid={self.proc.pid} out={self.out} (hub http {self.hub_port})")
            self.busy = ""
            return self.state()


def make_handler(runner: Runner, hub_port: int, extra: list[str]):
    launcher_js = (WEB / "launcher.js").read_bytes()

    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _json(self, obj, code=HTTPStatus.OK):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            if u.path == "/api/run":
                d = (q.get("dataset") or [""])[0]          # omitted = the default dataset (old API)
                b = (q.get("backend") or [""])[0]
                f = (q.get("features") or [""])[0]         # ORB backends only; omitted = the backend's default
                try:
                    d, f = resolve(d, b, f)
                except ValueError:
                    return self._json({"error": "dataset/backend/features?",
                                       "datasets": {k: {bk: bv.get("features") for bk, bv in v["backends"].items()}
                                                    for k, v in DATASETS.items()}},
                                      HTTPStatus.BAD_REQUEST)
                threading.Thread(target=runner.start, args=(d, b, extra, f), daemon=True).start()
                time.sleep(0.3)
                return self._json(runner.state())
            if u.path == "/api/stop":
                threading.Thread(target=runner.stop, daemon=True).start()
                time.sleep(0.3)
                return self._json(runner.state())
            self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/api/state":
                return self._json(runner.state())
            if u.path == "/launcher.js":
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/javascript")
                self.send_header("Content-Length", str(len(launcher_js)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(launcher_js)
                return
            if u.path in ("/", "/index.html"):
                return self._page()
            return self._proxy()

        def _page(self):
            """the hub's index.html (proxied when the hub is up, the file otherwise) + the launcher bar"""
            html = None
            if runner.state()["hub_alive"]:
                try:
                    c = http.client.HTTPConnection("127.0.0.1", hub_port, timeout=3)
                    c.request("GET", "/")
                    r = c.getresponse()
                    if r.status == 200:
                        html = r.read().decode("utf-8", "replace")
                    c.close()
                except OSError:
                    html = None
            if html is None:
                html = (WEB / "index.html").read_text(encoding="utf-8")
            inject = '<div id="launcher-bar"></div>\n<script src="/launcher.js"></script>\n'
            i = html.find("<header>")
            html = html[:i] + inject + html[i:] if i >= 0 else inject + html
            body = html.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _proxy(self):
            """stream any other GET (/events SSE, /stats) from the hub"""
            if not runner.state()["hub_alive"]:
                return self._json({"error": "no hub running", "state": runner.state()}, HTTPStatus.SERVICE_UNAVAILABLE)
            try:
                c = http.client.HTTPConnection("127.0.0.1", hub_port, timeout=10)
                c.request("GET", self.path, headers={"Accept": self.headers.get("Accept", "*/*")})
                r = c.getresponse()
            except OSError as e:
                return self._json({"error": f"hub unreachable: {e}"}, HTTPStatus.BAD_GATEWAY)
            self.send_response(r.status)
            streaming = "text/event-stream" in (r.getheader("Content-Type") or "")
            for k, v in r.getheaders():
                if k.lower() in ("content-type", "cache-control"):
                    self.send_header(k, v)
            if streaming:
                self.send_header("Connection", "close")
                self.end_headers()
                try:
                    while True:
                        line = r.readline()
                        if not line:
                            break
                        self.wfile.write(line)
                        if line == b"\n":
                            self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    c.close()
                    self.close_connection = True
            else:
                data = r.read()
                c.close()
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--hub-port", type=int, default=8766)
    ap.add_argument("--dataset", default=DEFAULT_DATASET, help=f"dataset for --backend ({'|'.join(DATASETS)})")
    ap.add_argument("--backend", default="", help="start this backend right away (orb|hdl-3|...)")
    ap.add_argument("--out-root", default=str(REPO / "objpose" / "output"))
    ap.add_argument("hub_args", nargs="*", help="extra hub.py options passed to every run (after --)")
    a = ap.parse_args()
    runner = Runner(a.hub_port, Path(a.out_root))
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), make_handler(runner, a.hub_port, a.hub_args))
    srv.daemon_threads = True

    def bye(*_):
        log("shutting down")
        runner.stop()
        os._exit(0)
    signal.signal(signal.SIGINT, bye)
    signal.signal(signal.SIGTERM, bye)
    log(f"viewer http://192.168.20.5:{a.port}  (hub on 127.0.0.1:{a.hub_port}, one backend at a time)")
    if a.backend:
        threading.Thread(target=runner.start, args=(a.dataset, a.backend, a.hub_args), daemon=True).start()
    srv.serve_forever()


if __name__ == "__main__":
    main()
