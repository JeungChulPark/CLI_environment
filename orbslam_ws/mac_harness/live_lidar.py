#!/usr/bin/env python3
"""live_lidar.py — LiDAR SLAM backend of the live viewer: KISS-ICP or hdl_graph_slam on a session's VLP-16 bag.

The trackers themselves are the object-pose streamers in ~/objpose/lidar (lidar_stream.py = KISS-ICP,
run_hdl_stream.sh / hdl_stream.py = hdl_graph_slam in its ROS 2 Jazzy env). They replay the scans at bag
rate and send poses as newline JSON over TCP; this process plays the hub's end of that line and serves the
viewer exactly like live_orbslam / live_rtabmap do (WebSocket /ws, replay /rec/frame/<i> and /rec/map/<k>),
so web/index.html shows a LiDAR run with no special casing beyond labels:

  * frame = one scan: the session's colour image nearest in time (camera and LiDAR share the header clock),
    a bird's-eye view of the scan as the small picture, the scan itself in red in the 3D view, the pose
  * map   = the scans accumulated at their poses, voxel-thinned, coloured by height. For hdl_graph_slam each
    scan is held relative to its reference keyframe and re-placed on every keyframe update, so loop
    closures and graph optimisation visibly correct the map
  * timing = the streamer's per-scan registration time against the scan period (10 Hz -> 100 ms deadline)

The viewer draws worlds in the camera convention (x right, y down, z forward), so LiDAR coordinates
(x forward, y left, z up) are re-expressed with M: x_c = -y_l, y_c = -z_l, z_c = x_l.

    live_lidar.py --slam kiss|hdl --session <dataset session dir> --out DIR [--port 8083] [--rate 1.0] [--no-wait]
"""
from __future__ import annotations

import argparse
import base64
import collections
import hashlib
import json
import signal
import socket
import sqlite3
import struct
import subprocess
import sys
import threading
import time
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
from scipy.spatial.transform import Rotation
import warnings

# macOS Accelerate's matmul raises spurious divide/overflow/invalid flags on finite inputs
warnings.filterwarnings("ignore", message=".*encountered in matmul", category=RuntimeWarning)
import numpy as np

OBJPOSE_LIDAR = Path.home() / "objpose" / "lidar"
sys.path.insert(0, str(OBJPOSE_LIDAR))
from lidar_stream import header_stamp, parse_pc2  # noqa: E402

M = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]], float)   # lidar axes -> viewer axes
MAP_VOXEL = 0.20
MAP_RANGE = 25.0


def disp(T):
    """lidar-world pose -> viewer-world pose"""
    return M @ T @ M.T


def voxel(P, size):
    if not len(P):
        return P
    _, i = np.unique(np.floor(P / size).astype(np.int64), axis=0, return_index=True)
    return P[i]


# ---------------------------------------------------------------- minimal WebSocket server (RFC 6455, send-only)
def ws_frame(payload: bytes, binary: bool) -> bytes:
    n = len(payload)
    head = bytes([0x82 if binary else 0x81])
    if n < 126:
        head += bytes([n])
    elif n < 65536:
        head += bytes([126]) + struct.pack(">H", n)
    else:
        head += bytes([127]) + struct.pack(">Q", n)
    return head + payload


class Client:
    """one browser: its own sender thread; per message kind only the newest `keep` wait, as in ws_server.h"""

    def __init__(self, sock):
        self.sock, self.cv, self.q, self.alive = sock, threading.Condition(), collections.deque(), True
        threading.Thread(target=self._run, daemon=True).start()

    def send(self, data, kind=0, keep=0):
        with self.cv:
            if kind and keep:
                same = [m for m in self.q if m[0] == kind]
                while len(same) >= keep:
                    self.q.remove(same.pop(0))
            self.q.append((kind, data))
            self.cv.notify()

    def _run(self):
        while self.alive:
            with self.cv:
                while not self.q and self.alive:
                    self.cv.wait(0.5)
                if not self.alive:
                    return
                _, data = self.q.popleft()
            try:
                self.sock.sendall(data)
            except OSError:
                self.alive = False


class Viewer:
    def __init__(self, on_connect, route):
        self.clients, self.lock, self.on_connect, self.route = [], threading.Lock(), on_connect, route

    def broadcast(self, data, kind=0, keep=0):
        with self.lock:
            self.clients = [c for c in self.clients if c.alive]
            for c in self.clients:
                c.send(data, kind, keep)

    def n(self):
        with self.lock:
            return sum(c.alive for c in self.clients)

    def handler(self):
        viewer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                if self.path == "/ws":
                    key = self.headers.get("Sec-WebSocket-Key", "")
                    acc = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
                    self.send_response(101)
                    self.send_header("Upgrade", "websocket"); self.send_header("Connection", "Upgrade")
                    self.send_header("Sec-WebSocket-Accept", acc)
                    self.end_headers()
                    c = Client(self.connection)
                    for m in viewer.on_connect():
                        c.send(m)
                    with viewer.lock:
                        viewer.clients.append(c)
                    try:                                  # read until the browser closes (frames are ignored)
                        while c.alive:
                            b = self.connection.recv(4096)
                            if not b or (b[0] & 0x0F) == 0x8:
                                break
                    except OSError:
                        pass
                    c.alive = False
                    with c.cv:
                        c.cv.notify()
                    return
                body = viewer.route(self.path)
                if body is None:
                    self.send_response(404); self.send_header("Content-Length", "0"); self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(body)

        return H


# ---------------------------------------------------------------- recorder (same role as Recorder in live_common.h)
class Recorder:
    def __init__(self, out: Path):
        self.ff, self.mf = open(out / "stream_frames.bin", "w+b"), open(out / "stream_maps.bin", "w+b")
        self.frames, self.maps, self.lock = [], [], threading.Lock()

    def _app(self, f, b):
        f.seek(0, 2); off = f.tell(); f.write(b); f.flush()
        return off, len(b)

    def add_frame(self, i, b):
        with self.lock:
            while len(self.frames) <= i:
                self.frames.append(None)
            self.frames[i] = self._app(self.ff, b)

    def add_map(self, frame_idx, b):
        with self.lock:
            self.maps.append((frame_idx, self._app(self.mf, b)))

    def frame(self, i):
        with self.lock:
            for j in range(min(i, len(self.frames) - 1), -1, -1):
                if self.frames[j]:
                    off, n = self.frames[j]
                    return os_pread(self.ff, off, n)
        return None

    def map(self, k):
        with self.lock:
            if 0 <= k < len(self.maps):
                off, n = self.maps[k][1]
                return os_pread(self.mf, off, n)
        return None

    def map_frames(self):
        with self.lock:
            return [m[0] for m in self.maps]


def os_pread(f, off, n):
    import os
    return os.pread(f.fileno(), n, off)


# ---------------------------------------------------------------- session data: scans and camera frames by stamp
class Session:
    def __init__(self, root: Path):
        self.root = root
        ldb = next((root / "lidar").glob("*.db3"))
        self.lcon = sqlite3.connect(f"file:{ldb}?immutable=1", uri=True, check_same_thread=False)
        tid = self.lcon.execute("select id from topics where name='/velodyne_points'").fetchone()[0]
        self.scan_id = {}
        for mid, h in self.lcon.execute("select id, substr(data,1,16) from messages where topic_id=?", (tid,)):
            self.scan_id.setdefault(header_stamp(h), mid)
        self.scan_t = np.array(sorted(self.scan_id), np.int64)
        self.cam = None
        sdb = sorted((root / "SLAM").glob("SLAM_*.db3"))
        if sdb:
            self.ccon = sqlite3.connect(f"file:{sdb[0]}?immutable=1", uri=True, check_same_thread=False)
            row = self.ccon.execute("select id from topics where name='/camera/camera/color/image_raw'").fetchone()
            if row:
                # the bag's timestamp column equals the image header stamp in the converted sessions; reading it
                # avoids substr() on the image blobs, which loads every image (15 GB for 260901_cbnu_bigeightcircle)
                st = list(self.ccon.execute("select timestamp, id from messages where topic_id=? order by timestamp", row))
                self.cam = (np.array([s for s, _ in st], np.int64), [m for _, m in st])
        self.lock = threading.Lock()

    def scan(self, t_ns):
        # nearest stamp: hdl_graph_slam's poses come back up to ~1 us off the header stamp (ROS time conversion)
        k = int(np.clip(np.searchsorted(self.scan_t, t_ns), 1, len(self.scan_t) - 1))
        k = k - 1 if abs(self.scan_t[k - 1] - t_ns) < abs(self.scan_t[k] - t_ns) else k
        if abs(int(self.scan_t[k]) - t_ns) > 2_000_000:
            return None
        mid = self.scan_id[int(self.scan_t[k])]
        with self.lock:
            blob = self.lcon.execute("select data from messages where id=?", (mid,)).fetchone()[0]
        p = parse_pc2(blob)
        xyz = np.column_stack([p["x"], p["y"], p["z"]]).astype(np.float64)
        r = np.linalg.norm(xyz, axis=1)
        return xyz[np.isfinite(r) & (r > 0.5) & (r < MAP_RANGE)]

    def image(self, t_ns):
        """bgr colour frame nearest to t_ns (None if the session has no camera bag)"""
        if self.cam is None:
            return None
        ts, ids = self.cam
        k = int(np.clip(np.searchsorted(ts, t_ns), 1, len(ts) - 1))
        k = k - 1 if abs(ts[k - 1] - t_ns) < abs(ts[k] - t_ns) else k
        with self.lock:
            b = bytes(self.ccon.execute("select data from messages where id=?", (ids[k],)).fetchone()[0])
        o = 12; n = struct.unpack_from("<I", b, o)[0]; o += 4 + n
        o = 4 + (o - 4 + 3) // 4 * 4
        h, w = struct.unpack_from("<II", b, o); o += 8
        n = struct.unpack_from("<I", b, o)[0]; enc = b[o + 4:o + 3 + n].decode(); o += 4 + n
        o += 1; o = 4 + (o - 4 + 3) // 4 * 4; o += 8
        img = np.frombuffer(b, np.uint8, h * w * 3, o).reshape(h, w, 3)
        return img[:, :, ::-1] if enc == "rgb8" else img


def bev(scan_l, size=(320, 240), span=10.0):
    """bird's-eye view of one scan in the sensor frame (forward = up), coloured by height"""
    w, h = size
    img = np.zeros((h, w, 3), np.uint8)
    if scan_l is None or not len(scan_l):
        return img
    s = h / (2 * span)
    u = (w / 2 - scan_l[:, 1] * s).astype(int); v = (h / 2 - scan_l[:, 0] * s).astype(int)
    ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    z = np.clip((scan_l[ok, 2] + 1.5) / 3.0, 0, 1)
    col = cv2.applyColorMap((z * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)
    for du, dv in ((0, 0), (1, 0), (0, 1), (1, 1)):     # 2x2 dots
        img[np.minimum(v[ok] + dv, h - 1), np.minimum(u[ok] + du, w - 1)] = col
    cv2.drawMarker(img, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_TRIANGLE_UP, 10, 1)
    return img


# ---------------------------------------------------------------- the streamer line (we are the hub end)
class Line:
    def __init__(self, sock):
        self.s, self.buf = sock, b""

    def send(self, obj):
        self.s.sendall((json.dumps(obj) + "\n").encode())

    def recv(self):
        while b"\n" not in self.buf:
            d = self.s.recv(1 << 20)
            if not d:
                return None
            self.buf += d
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--slam", required=True, choices=["kiss", "hdl"])
    ap.add_argument("--session", required=True, help="dataset session dir (with lidar/ and SLAM/)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--port", type=int, default=8083)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--rate", type=float, default=1.0, help="bag replay speed")
    ap.add_argument("--no-wait", action="store_true", help="start without waiting for a browser")
    ap.add_argument("--jpeg-quality", type=int, default=80)
    a = ap.parse_args()
    proc = None
    signal.signal(signal.SIGINT, lambda *_: sys.exit(130))     # until the streamer runs: a stop just ends us
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    root, out = Path(a.session).expanduser(), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    label = {"kiss": "KISS-ICP", "hdl": "hdl_graph_slam"}[a.slam]
    config = {"kiss": "KISS-ICP 1.3 · voxel 0.10 m · deskew x2", "hdl": "hdl_graph_slam · FAST_GICP · g2o + loop closure"}[a.slam]
    sess = Session(root)
    scan_hz = 10.0
    deadline = 1000.0 / scan_hz

    # ---- shared state for (re)connecting browsers
    rec = Recorder(out)
    st = {"status": "loading", "summary": "", "last_map": None, "last_frame": None}
    traj = []                       # [idx, state, ms, e2e, x, y, z] in viewer coordinates
    hl = threading.Lock()
    size = sess.image(next(iter(sess.scan_id))).shape[1::-1] if sess.cam else (640, 480)

    def hello():
        return ws_frame(json.dumps({"type": "hello", "slam": label, "bag": root.name, "settings": config,
                                    "rate": scan_hz * a.rate, "queue": 0, "width": size[0], "height": size[1],
                                    "deadline_ms": deadline, "status": st["status"], "sensor": "lidar"}).encode(), False)

    def history():
        with hl:
            t = [r[:4] + ([round(v, 4) for v in r[4:]] if len(r) > 4 else []) for r in traj]
        return ws_frame(json.dumps({"type": "history", "traj": t, "maps": rec.map_frames()}).encode(), False)

    def end_msg():
        return ws_frame(json.dumps({"type": "end", "summary": st["summary"]}).encode(), False)

    def on_connect():
        v = [hello(), history()]
        if st["last_map"]:
            v.append(st["last_map"])
        if st["last_frame"]:
            v.append(st["last_frame"])
        if st["status"] == "finished":
            v.append(end_msg())
        return v

    def route(path):
        for pre, fn in (("/rec/frame/", rec.frame), ("/rec/map/", rec.map)):
            if path.startswith(pre):
                try:
                    return fn(int(path[len(pre):]))
                except ValueError:
                    return None
        return None

    viewer = Viewer(on_connect, route)
    srv = ThreadingHTTPServer((a.host, a.port), viewer.handler())
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"viewer: http://{a.host}:{a.port}/", flush=True)

    # ---- streamer
    lsock = socket.socket(); lsock.bind(("127.0.0.1", 0)); lsock.listen(1)
    port = lsock.getsockname()[1]
    lidar_dir = root / "lidar"
    if a.slam == "kiss":
        cmd = [str(OBJPOSE_LIDAR / "venv/bin/python"), "-u", str(OBJPOSE_LIDAR / "lidar_stream.py"),
               "--session", str(lidar_dir), "--connect", f"127.0.0.1:{port}", "--smooth-scans", "1"]
    else:
        cmd = ["bash", str(OBJPOSE_LIDAR / "run_hdl_stream.sh"), "--session", str(lidar_dir),
               "--connect", f"127.0.0.1:{port}", "--launch-log", str(out / "hdl_launch.log"), "--dump-dir", str(out / "hdl_graph")]
    slog = open(out / "streamer.log", "w")
    proc = subprocess.Popen(cmd, stdout=slog, stderr=subprocess.STDOUT, start_new_session=True)
    print(f"[live_lidar] {label}: {' '.join(cmd)}", flush=True)

    stop = threading.Event()

    def on_int(*_):
        if stop.is_set():
            sys.exit(130)
        stop.set()
        try:
            proc.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass

    def on_term(*_):
        try:
            import os
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        sys.exit(143)

    signal.signal(signal.SIGINT, on_int)
    signal.signal(signal.SIGTERM, on_term)

    lsock.settimeout(1.0)
    conn = None
    while conn is None:
        try:
            conn, _ = lsock.accept()
        except socket.timeout:
            if proc.poll() is not None or stop.is_set():
                print("tracking stopped early: the streamer exited before connecting (see streamer.log)", flush=True)
                st["status"] = "finished"; st["summary"] = f"slam {label}\nerror streamer exited ({proc.returncode})\n"
                viewer.broadcast(end_msg())
                return idle(srv)
    line = Line(conn)
    h = line.recv()
    while h is not None and h.get("type") != "hello":
        h = line.recv()
    if h is None:
        print("tracking stopped early: no hello from the streamer", flush=True)
        st["status"] = "finished"
        return idle(srv)
    first_ns = int(h["first_ns"])
    st["status"] = "running"
    if not a.no_wait:
        print("waiting for a browser to connect before starting the camera ...", flush=True)
        while not stop.is_set() and viewer.n() == 0:
            time.sleep(0.05)
        time.sleep(0.5)
    viewer.broadcast(hello())
    line.send({"type": "start", "t0_ns": first_ns, "rate": a.rate})
    wall0 = time.monotonic()
    print("camera started", flush=True)

    # ---- per pose
    kfs = {}                          # hdl: keyframe id -> T_w_kf (lidar world)
    # map storage, voxel-merged as it grows so a rebuild stays cheap (it must not starve the reader thread):
    # held[kf id] = scans in that keyframe's frame (hdl, re-placed when the graph moves the keyframe),
    # held[None]  = scans in the world frame (KISS-ICP, or hdl poses before the first keyframe)
    held = {}
    ms_hist, e2e_hist, recent = [], [], collections.deque()
    idx = drops = loops = kf_updates = 0
    last_map_t = 0.0
    tum = open(out / "LidarTrajectory_tum.txt", "w")
    params = [cv2.IMWRITE_JPEG_QUALITY, a.jpeg_quality]

    def hold(ref, P):
        held[ref] = voxel(np.concatenate([held[ref], P]), MAP_VOXEL) if ref in held else P

    def build_map():
        pts = []
        for ref, P in held.items():
            if ref is None:
                pts.append(P)
            else:
                Tw = kfs[ref]
                pts.append(P @ Tw[:3, :3].T + Tw[:3, 3])
        if not pts:
            return np.zeros((0, 3), np.float32), None
        W = voxel(np.concatenate(pts), MAP_VOXEL)
        z = np.clip((W[:, 2] - np.percentile(W[:, 2], 2)) / max(1e-6, np.ptp(np.percentile(W[:, 2], [2, 98]))), 0, 1)
        rgb = cv2.applyColorMap((z * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_VIRIDIS).reshape(-1, 3)[:, ::-1]
        V = (W @ M[:3, :3].T).astype(np.float32)
        return V, np.ascontiguousarray(rgb, np.uint8)

    def send_map(frame_idx):
        V, rgb = build_map()
        n = len(V)
        b = bytes([2, 1 if rgb is not None and n else 0, 0, 0]) + struct.pack("<I", n) + V.tobytes()
        if rgb is not None and n:
            b += rgb.tobytes()
        rec.add_map(frame_idx, b)
        m = ws_frame(b, True)
        st["last_map"] = m
        viewer.broadcast(m, 2, 1)
        return V

    # a reader thread stamps each message on arrival, so drawing and map building here never inflate the latency
    inbox = collections.deque()
    inbox_cv = threading.Condition()

    def reader():
        while True:
            try:
                m = line.recv()
            except (OSError, ValueError):       # streamer killed (stop button / another backend started)
                m = None
            if m is not None:
                m["_rx"] = time.monotonic()
            with inbox_cv:
                inbox.append(m)
                inbox_cv.notify()
            if m is None or m.get("type") == "end":
                return

    threading.Thread(target=reader, daemon=True).start()
    while True:
        with inbox_cv:
            while not inbox:
                inbox_cv.wait()
            msg = inbox.popleft()
        if msg is None or msg.get("type") == "end":
            break
        typ = msg.get("type")
        if typ == "kf_update":
            for row in msg["kfs"]:
                kfs[int(row[0])] = np.array(row[1:], float).reshape(4, 4)
            kf_updates += 1
            loops = max(loops, int(msg.get("loops") or 0))
            continue
        if typ != "pose":
            continue
        t_ns = int(msg["t_ns"])
        T = np.array(msg["T_wc"], float).reshape(4, 4)
        ms = msg.get("track_ms")
        ms = float(ms if ms is not None else (msg.get("latency_ms") or 0.0))
        now = msg["_rx"]
        e2e = (now - (wall0 + (t_ns - first_ns) / 1e9 / a.rate)) * 1e3
        drops = int(msg.get("dropped") or 0)
        ms_hist.append(ms); e2e_hist.append(e2e)
        recent.append(now)
        while now - recent[0] > 1.0:
            recent.popleft()
        q = T[:3, :3]
        qq = Rotation.from_matrix(q).as_quat()
        tum.write(f"{t_ns / 1e9:.9f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} {qq[0]:.9f} {qq[1]:.9f} {qq[2]:.9f} {qq[3]:.9f}\n")

        scan = sess.scan(t_ns)
        if scan is not None and idx % 2 == 0:
            P = voxel(scan, MAP_VOXEL)
            ref = msg.get("ref_kf")
            if ref is not None and msg.get("T_w_kf"):
                Tkf = np.array(msg["T_w_kf"], float).reshape(4, 4)
                kfs.setdefault(int(ref), Tkf)
                Tk = np.linalg.inv(Tkf) @ T                 # keyframe -> scan
                hold(int(ref), P @ Tk[:3, :3].T + Tk[:3, 3])
            else:
                hold(None, P @ T[:3, :3].T + T[:3, 3])
        Td = disp(T)
        with hl:
            traj.append([idx, 2, round(ms, 3), round(e2e, 3), float(Td[0, 3]), float(Td[1, 3]), float(Td[2, 3])])

        # frame message, same layout as live_common.h
        arr = np.array(ms_hist)
        cur = (scan[::4] @ T[:3, :3].T + T[:3, 3]) if scan is not None else np.zeros((0, 3))
        cur = (cur @ M[:3, :3].T).astype(np.float32)
        img = sess.image(t_ns)
        if img is None:
            img = np.zeros((size[1], size[0], 3), np.uint8)
        ok1, jc = cv2.imencode(".jpg", img, params)
        ok2, jd = cv2.imencode(".jpg", bev(scan), params)
        js = json.dumps({"i": idx, "stamp": (t_ns - first_ns) / 1e9, "state": 2, "track_ms": round(ms, 3), "wait_ms": 0,
                         "e2e_ms": round(e2e, 3), "hz": float(len(recent)) if now - wall0 >= 1 else 0, "n": len(arr),
                         "mean": round(float(arr.mean()), 3), "p99": round(float(np.percentile(arr, 99)), 3),
                         "max": round(float(arr.max()), 3), "over": int((arr > deadline).sum()), "drops": drops,
                         "viz_ms": 0, "jpeg_ms": 0, "n_kp": int(len(cur)), "rel_wall_ms": int(time.time() * 1000 - e2e),
                         "T": [round(float(v), 5) for v in Td[:3].reshape(-1)]}).encode()
        b = (bytes([1, 0, 0, 0]) + struct.pack("<IIII", len(js), len(cur), len(jc), len(jd)) + js + cur.tobytes()
             + jc.tobytes() + jd.tobytes())
        rec.add_frame(idx, b)
        m = ws_frame(b, True)
        st["last_frame"] = m
        viewer.broadcast(m, 1, 2)
        if now - last_map_t >= 1.0:
            last_map_t = now
            send_map(idx)
        idx += 1
        if idx % 300 == 0:
            print(f"[live] {idx} scans, last {ms:.2f} ms, p99 {np.percentile(arr, 99):.2f} ms, browsers {viewer.n()}", flush=True)

    tum.close()
    wall_s = time.monotonic() - wall0
    V = send_map(max(0, idx - 1))
    with open(out / "map_points.pcd", "w") as f:
        f.write("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                f"WIDTH {len(V)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(V)}\nDATA ascii\n")
        W = V @ M[:3, :3]                       # back to lidar axes
        np.savetxt(f, W, fmt="%.4f")
    arr, e = np.array(ms_hist or [0.0]), np.sort(np.array(e2e_hist or [0.0]))
    s = (f"slam {label}\nsession {root.name}\nscans {idx}\nqueue_dropped {drops}\nwall_s {wall_s:.2f}\n"
         f"effective_hz {idx / max(wall_s, 1e-9):.2f}\n"
         f"track_ms mean {arr.mean():.2f} p50 {np.percentile(arr, 50):.2f} p95 {np.percentile(arr, 95):.2f} "
         f"p99 {np.percentile(arr, 99):.2f} max {arr.max():.2f}\nover_{deadline:.0f}ms {(arr > deadline).sum()}\n"
         f"release_to_done_ms p99 {e[int(0.99 * (len(e) - 1))]:.2f} max {e[-1]:.2f}\n"
         f"map_points {len(V)} (voxel {MAP_VOXEL} m)\n")
    if a.slam == "hdl":
        s += f"keyframes {len(kfs)}\nkf_updates {kf_updates}\nloop_closures {loops}\n"
    if stop.is_set():
        s += "stopped_early 1\n"
    (out / "summary.txt").write_text(s)
    print("==== SUMMARY ====\n" + s, flush=True)
    st["summary"] = s
    st["status"] = "finished"
    viewer.broadcast(end_msg())
    print("tracking stopped early" if stop.is_set() else
          f"bag finished; the viewer (with replay) stays up at http://{a.host}:{a.port}/", flush=True)
    try:
        proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        import os
        os.killpg(proc.pid, signal.SIGTERM)
    return idle(srv)


def idle(srv):
    """stay up for replay until SIGTERM"""
    signal.signal(signal.SIGINT, lambda *_: None)
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    sys.exit(main())
