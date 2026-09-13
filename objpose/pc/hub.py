#!/usr/bin/env python3
"""hub.py — PC (.100) side of the object-pose system.

    Mac (.113) ORB-SLAM3 ──poses over TCP (ssh -R tunnel)──┐
                                                          ▼
    SAM session (raw) ──1x replay──► shared memory ──► sam6d_infer.py (unchanged)
          │                                                │ results (shared memory)
          └──────── display frames ──► Fusion ◄────────────┘
                                          │
                                  http://localhost:8765  (video + 3D view)

The hub commands the Mac: it opens the SSH reverse tunnel and launches the SLAM
streamer there with --features 2000, waits until both ORB-SLAM3 (hello) and SAM-6D
(READY) have loaded, then starts both replays at the same dataset time t0.
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import queue
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SAM6D = REPO / "sam6d_realtime"
sys.path.insert(0, str(SAM6D / "realtime"))
sys.path.insert(0, str(HERE))

from clock import frame_clock                           # noqa: E402
from conv_session import ConvSession                    # noqa: E402
from fusion import Fusion, PoseBuffer, inv_se3          # noqa: E402
from memory import ObjectMemory                         # noqa: E402
from rs_session import RsSession                        # noqa: E402
from shm_channel import FrameWriter, JsonReader         # noqa: E402

SAM_MINUS_SLAM_CLOCK_NS = 18225662057   # SLAM/peer_timestamp_comparison.json
PY_SAM6D = "/home/jucpark/anaconda3/envs/sam6d/bin/python"


def log(*a):
    print(time.strftime("%H:%M:%S"), "[hub]", *a, flush=True)


# ── extrinsic ────────────────────────────────────────────────────────────────
def load_extrinsic(path: Path) -> tuple[np.ndarray, str]:
    if path.suffix == ".json":
        d = json.loads(path.read_text())
        T = np.asarray(d["T_slam_sam"], np.float64).reshape(4, 4)
        return T, d.get("source", str(path))
    import xml.etree.ElementTree as ET
    from scipy.spatial.transform import Rotation
    root = ET.parse(path).getroot()
    for j in root.iter("joint"):
        if j.find("child").get("link") == "sam_camera_color_optical_frame":
            o = j.find("origin")
            T = np.eye(4)
            T[:3, :3] = Rotation.from_euler("xyz", [float(v) for v in o.get("rpy").split()]).as_matrix()
            T[:3, 3] = [float(v) for v in o.get("xyz").split()]
            return T, f"urdf:{path}"
    raise ValueError(f"no slam->sam joint in {path}")


# ── drawing ──────────────────────────────────────────────────────────────────
PALETTE = {"milk": (240, 240, 240), "choco_hazelnut_high": (60, 60, 200), "Febreze_high": (230, 170, 60),
           "Mugcup_high": (180, 110, 240), "saffron": (80, 200, 240), "Sauce_high": (40, 140, 255),
           "Sikhye_high": (60, 220, 230), "Bear": (60, 120, 190), "Dinosaur": (90, 210, 110)}
BOX_EDGES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def box_corners(ext):
    mn, mx = np.asarray(ext["min_mm"]) / 1000.0, np.asarray(ext["max_mm"]) / 1000.0
    return np.array([[x, y, z] for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])])


DISTORT_R_MAX = 1.3   # the colour model's radial polynomial folds back beyond r = 1.63; image corners are r = 1.06


def project(cam, K, dist):
    """Pinhole + Brown-Conrady projection that never folds.

    Points far outside the field of view (up to 65 deg off-axis here) pushed through the
    distortion polynomial land back INSIDE the image, ~1000 px from where they belong, and
    a box edge to them crosses the whole screen. Beyond DISTORT_R_MAX, where nothing is
    visible anyway, points are projected without distortion."""
    x = cam[:, 0] / cam[:, 2]
    y = cam[:, 1] / cam[:, 2]
    k1, k2, p1, p2, k3 = (list(dist) + [0.0] * 5)[:5]
    r2 = x * x + y * y
    radial = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    far = r2 > DISTORT_R_MAX ** 2
    xd[far], yd[far] = x[far], y[far]
    return np.column_stack([K[0, 0] * xd + K[0, 2], K[1, 1] * yd + K[1, 2]])


def draw_objects(img, K, dist, rows, extents, max_age_s=float("inf")):
    for r in rows:
        if r["age_s"] > max_age_s:
            continue
        T = r.get("T_cam_obj")
        ext = extents.get(r["name"])
        if T is None or ext is None:
            continue
        corners = box_corners(ext)
        axis_len = 0.6 * float(max(ext["size_mm"])) / 1000.0
        pts = np.vstack([corners, [[0, 0, 0], [axis_len, 0, 0], [0, axis_len, 0], [0, 0, axis_len]]])
        cam = (T[:3, :3] @ pts.T).T + T[:3, 3]
        # objects beside or behind the camera project to huge, meaningless lines: require the
        # whole box in front, its centre inside the field of view, and every point near the image
        if np.any(cam[:, 2] < 0.2):
            continue
        uv = project(cam, K, dist)
        h_img, w_img = img.shape[:2]
        c = uv[:8].mean(0)
        if not np.all(np.isfinite(uv)) or not (0 <= c[0] < w_img and 0 <= c[1] < h_img):
            continue
        interp = r["source"] != "sam6d"
        col = PALETTE.get(r["name"], (255, 255, 255))
        thick = 1 if interp else 2
        uv = np.clip(uv, -1e5, 1e5)
        p = [tuple(int(round(v)) for v in q) for q in uv]
        rect = (0, 0, w_img, h_img)

        def seg(a_, b_, colour, width):
            ok, q1, q2 = cv2.clipLine(rect, p[a_], p[b_])
            if ok:
                cv2.line(img, q1, q2, colour, width, cv2.LINE_AA)
        for a, b in BOX_EDGES:
            seg(a, b, col, thick)
        for k, c in ((9, (0, 0, 255)), (10, (0, 200, 0)), (11, (255, 80, 0))):
            seg(8, k, c, 2)
        top = min(p[:8], key=lambda q: q[1])
        top = (int(np.clip(top[0], 0, w_img - 1)), int(np.clip(top[1], 0, h_img - 1)))
        name = f"{r['name']}#{r['instance']}" if r.get("multi") else r["name"]
        label = f"{name}  {'SAM-6D' if not interp else 'interp +%.1fs' % r['age_s']}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        x0, y0 = max(0, top[0] - tw // 2), max(th + 4, top[1] - 6)
        cv2.rectangle(img, (x0 - 3, y0 - th - 4), (x0 + tw + 3, y0 + 4), (20, 20, 20), -1)
        cv2.putText(img, label, (x0, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
    return img


def mat16(T):
    return None if T is None else [round(float(v), 6) for v in np.asarray(T).reshape(-1)]


# ── hub ──────────────────────────────────────────────────────────────────────
class Hub:
    def __init__(self, a):
        self.a = a
        self.out = Path(a.out).resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        self.stop = threading.Event()
        clock_dir = REPO / "objpose" / "rt" / "clock"
        xj = json.loads(Path(a.extrinsic).read_text()) if a.extrinsic.endswith(".json") else {}
        self.raw_sam = (Path(a.sam_session) / "rgbd_timestamp_associations.json").exists()
        if self.raw_sam:
            # raw SDK sessions: association stamps of the SLAM session drift by seconds, so both
            # sides use the colour-metadata frame clock; SAM "System Time" is arrival time and the
            # rig calibration measured its lag (tau_s, SLAM time = SAM time + tau)
            self.sam_clock = frame_clock(a.sam_session, clock_dir)
            self.sam_tau_ns = int(round(float(xj.get("tau_s", 0.0)) * 1e9)) if "ftime" in str(xj.get("inputs", "")) or "frame clock" in str(xj.get("clock", "")) else 0
            self.sam = RsSession(a.sam_session, offset_ns=-SAM_MINUS_SLAM_CLOCK_NS,
                                 times_ns=self.sam_clock["frame_ns"] + self.sam_tau_ns)
        else:
            # converted bags (260910): header stamps are already on the shared clock
            self.sam_tau_ns = int(round(float(xj.get("sam_tau_s", 0.0)) * 1e9))
            self.sam = ConvSession(a.sam_session, offset_ns=self.sam_tau_ns)
        # frame-index remapping of SLAM poses only exists for raw camera SLAM sessions
        self.slam_clock = (frame_clock(a.slam_session, clock_dir)
                           if a.slam != "lidar" and (Path(a.slam_session) / "rgbd_timestamp_associations.json").exists()
                           else None)
        self.K = self.sam.K
        self.dist = np.asarray(self.sam.kc, np.float64)
        X, xsrc = load_extrinsic(Path(a.extrinsic))
        self.extrinsic_source = xsrc
        self.poses = PoseBuffer(max_gap_s=a.max_gap)
        self.fusion = Fusion(X, self.poses)
        # position-only association: box-like objects flip 180 deg between SAM-6D estimates,
        # and a rotation gate split one object into two instances on this dataset
        self.memory = None if a.no_memory else ObjectMemory(
            self.fusion, self.sam.K, (self.sam.W, self.sam.H),
            assoc_trans_gate_m=a.assoc_gate_m, assoc_rot_gate_deg=None)
        self.extents = json.loads((REPO / "integration" / "cad_extents.json").read_text())
        self.clients: list[queue.Queue] = []
        self.clients_lock = threading.Lock()
        self.frames = deque(maxlen=120)            # (t_ns, index, bgr)
        self.hello = None
        self.slam_sock = None
        self.slam_end = None
        self.unrefined = {}
        self.t0_ns = None
        self.wall0 = None
        self.replay_t_ns = 0
        self.replay_done = False
        self.fed = 0
        self.dropped_display = 0
        self.sam6d_runs = []                       # latency records
        self.sam6d_frames = 0
        self.last_result = None
        self.procs: list[subprocess.Popen] = []
        self.logs = {k: open(self.out / f"{k}.jsonl", "w", encoding="utf-8")
                     for k in ("slam_poses", "sam6d_estimates", "display_objects", "kf_updates", "sam6d_frames")}
        self.log_lock = threading.Lock()
        self.status = "starting"
        log(f"SAM session {self.sam.dir} ({'raw' if self.raw_sam else 'converted'}) frames={len(self.sam)} "
            f"extrinsic={xsrc} SAM tau {self.sam_tau_ns / 1e6:+.1f} ms")

    def write_log(self, key, obj):
        with self.log_lock:
            self.logs[key].write(json.dumps(obj, ensure_ascii=False) + "\n")

    # ── subprocesses ────────────────────────────────────────────────────────
    def start_sam6d(self):
        cfg = yaml.safe_load((SAM6D / "realtime" / "run_split_example.yaml").read_text(encoding="utf-8"))
        cfg["output"] = {"dir": str(self.out / "sam6d"), "diagnostics": False}
        cfg["runtime"]["idle_exit_s"] = 0
        cfg.pop("bag", None)
        cfg["anchor"]["enabled"] = False
        cfg_path = self.out / "sam6d_config.yaml"
        cfg_path.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")
        (self.out / "sam6d").mkdir(exist_ok=True)
        try:
            (self.out / "sam6d" / "READY").unlink()
        except FileNotFoundError:
            pass
        env = dict(os.environ)
        env.pop("ROS_DOMAIN_ID", None)
        f = open(self.out / "sam6d_infer.log", "w")
        p = subprocess.Popen([PY_SAM6D, "-u", str(SAM6D / "realtime" / "sam6d_infer.py"), "--config", str(cfg_path)],
                             cwd=str(self.out / "sam6d"), stdout=f, stderr=subprocess.STDOUT, env=env,
                             start_new_session=True)
        self.procs.append(p)
        log(f"SAM-6D started pid={p.pid} (log {self.out / 'sam6d_infer.log'})")

    def start_mac_slam(self):
        if self.a.slam == "lidar":
            remote = (f"~/objpose/lidar/venv/bin/python -u ~/objpose/lidar/lidar_stream.py --session {self.a.mac_slam_session} "
                      f"--mode live --connect 127.0.0.1:{self.a.slam_port} --rate {self.a.rate} "
                      # --slam-param deskew_passes=1 -> --deskew-passes 1 (lidar_stream.py options)
                      + " ".join(f"--{k.replace('_', '-')} {v}" for k, v in (x.split("=", 1) for x in self.a.slam_param)))
        else:
            remote = (f"{self.a.mac_runner} --slam {self.a.slam} --session {self.a.mac_slam_session} --features {self.a.features} "
                      f"--time-source frame {' '.join('--param ' + x for x in self.a.slam_param)} "
                      f"--mode live --connect 127.0.0.1:{self.a.slam_port} --rate {self.a.rate}")
        cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ExitOnForwardFailure=yes", "-o", "ServerAliveInterval=10",
               "-R", f"{self.a.slam_port}:127.0.0.1:{self.a.slam_port}", self.a.mac_host, remote]
        f = open(self.out / "mac_slam.log", "w")
        p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        self.procs.append(p)
        (self.out / "mac_command.txt").write_text(" ".join(cmd) + "\n")
        log(f"Mac {self.a.slam} launched over ssh (features={self.a.features}): {remote}")

    # ── SLAM pose server ────────────────────────────────────────────────────
    def slam_server(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", self.a.slam_port))
        srv.listen(1)
        srv.settimeout(1.0)
        log(f"waiting for SLAM stream on 127.0.0.1:{self.a.slam_port}")
        while not self.stop.is_set():
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.slam_sock = conn
            buf = b""
            conn.settimeout(1.0)
            while not self.stop.is_set():
                try:
                    chunk = conn.recv(65536)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if line.strip():
                        self.on_slam_msg(json.loads(line))
            log("SLAM stream closed")
            self.slam_sock = None
        srv.close()

    def on_slam_msg(self, m):
        kind = m.get("type")
        if kind == "hello":
            self.hello = m
            # the streamer may still stamp with association times: t0 is always on the frame clock
            self.hello["first_ns_frame"] = (int(self.slam_clock["frame_ns"][0]) if self.slam_clock is not None
                                            else int(m["first_ns"]))
            log(f"SLAM hello: {m}")
        elif kind == "pose":
            recv = time.monotonic()
            if "frame_idx" in m and self.slam_clock is not None:
                m["t_ns_streamer"] = m["t_ns"]
                m["t_ns"] = int(self.slam_clock["frame_ns"][int(m["frame_idx"])])
            self.poses.add(int(m["t_ns"]), m.get("state", "?"), m.get("T_wc"), m.get("map_id"),
                           kf=m.get("ref_kf"), T_w_kf=m.get("T_w_kf"), kf_map_id=m.get("kf_map_id"),
                           map_changed=bool(m.get("map_changed")))
            if m.get("ref_kf") is not None:
                self.kf_anchoring = True
            if m.get("map_changed"):
                self.slam_loop_events = getattr(self, "slam_loop_events", 0) + 1
            lag = None
            if self.wall0 is not None:
                lag = (recv - self.wall0) - (int(m["t_ns"]) - self.t0_ns) / 1e9 / self.a.rate
            m["recv_lag_s"] = None if lag is None else round(lag, 4)
            if self.smoothing_streamer():
                self.unrefined[int(m["t_ns"])] = m      # logged once its smoothed pose arrives
            else:
                self.write_log("slam_poses", m)
            self.slam_lag = lag
            self.slam_track_ms = m.get("track_ms")
            self.slam_dropped = m.get("dropped", 0)
        elif kind == "pose_refine":
            # delayed smoothed pose (lidar_stream --smooth-scans): replaces the raw one in the buffer; the log keeps
            # the final pose in T_wc and the streamed one in T_wc_raw
            recv = time.monotonic()
            ok = self.poses.refine(int(m["t_ns"]), m["T_wc"])
            p = self.unrefined.pop(int(m["t_ns"]), None)
            if p is not None:
                p["T_wc_raw"], p["T_wc"], p["smooth_n"] = p["T_wc"], m["T_wc"], m.get("smooth_n")
                if self.wall0 is not None:
                    p["refine_lag_s"] = round((recv - self.wall0) - (int(m["t_ns"]) - self.t0_ns) / 1e9 / self.a.rate, 4)
                self.write_log("slam_poses", p)
            self.refined = getattr(self, "refined", 0) + int(ok)
        elif kind == "kf_update":
            self.poses.apply_kf_update(m.get("map_id"), m.get("kfs", []))
            self.write_log("kf_updates", m)
            self.last_kf_update = {"t_ns": m.get("t_ns"), "reason": m.get("reason"), "n": len(m.get("kfs", []))}
            if m.get("reason") != "periodic":
                log(f"kf_update ({m.get('reason')}): {len(m.get('kfs', []))} keyframes, "
                    f"max correction so far {self.poses.kfs.max_correction_m * 100:.1f} cm")
        elif kind == "end":
            for t in sorted(self.unrefined):
                self.write_log("slam_poses", self.unrefined.pop(t))
            self.slam_end = m
            log(f"SLAM end: {m}")

    def smoothing_streamer(self) -> bool:
        return bool(self.hello) and int((self.hello.get("params") or {}).get("smooth_scans", 1) or 1) > 1

    def send_slam(self, obj):
        self.slam_sock.sendall((json.dumps(obj) + "\n").encode())

    # ── coordinator + SAM replay ────────────────────────────────────────────
    def run_replay(self):
        ready = self.out / "sam6d" / "READY"
        self.status = "loading SAM-6D and ORB-SLAM3"
        while not self.stop.is_set() and not (ready.exists() and self.hello and self.slam_sock):
            time.sleep(0.2)
        if self.stop.is_set():
            return
        start = self.a.start_s
        t0 = max(int(self.sam.t_ns[0]), int(self.hello["first_ns_frame"])) + int(max(start, 0.3) * 1e9)
        self.t0_ns = t0
        self.send_slam({"type": "start", "t0_ns": t0, "rate": self.a.rate})
        self.wall0 = time.monotonic()
        self.status = "running"
        log(f"START t0_ns={t0} (dataset +{(t0 - int(self.hello['first_ns_frame'])) / 1e9:.2f}s) rate={self.a.rate}")
        end_ns = int(self.sam.t_ns[-1]) if self.a.duration_s <= 0 else t0 + int(self.a.duration_s * 1e9)
        i = int(np.searchsorted(self.sam.t_ns, t0))
        feed_period = 1.0 / self.a.sam_feed_hz
        next_feed = 0.0
        while not self.stop.is_set() and i < len(self.sam) and self.sam.t_ns[i] <= end_ns:
            due = self.wall0 + (int(self.sam.t_ns[i]) - t0) / 1e9 / self.a.rate
            now = time.monotonic()
            if due > now:
                time.sleep(min(due - now, 0.05))
                continue
            # behind schedule: jump to the newest frame that is already due
            j = i
            while j + 1 < len(self.sam) and self.wall0 + (int(self.sam.t_ns[j + 1]) - t0) / 1e9 / self.a.rate <= now:
                j += 1
            self.dropped_display += j - i
            i = j
            feed = now >= next_feed
            fr = self.sam.read(i, with_depth=feed)
            self.frames.append((fr.t_ns, i, fr.color_bgr))
            self.replay_t_ns = fr.t_ns
            if feed:
                depth = self.sam.align(fr.depth_raw)
                self.fw.write(np.ascontiguousarray(fr.color_bgr[:, :, ::-1]), depth, self.K, fr.t_ns, time.time(),
                              {"tracking_state": "TRACKING_LOST", "map_id": "primary_sam_camera"},
                              depth_stamp_ns=fr.t_ns)
                self.fed += 1
                next_feed = now + feed_period
            i += 1
        self.replay_done = True
        self.status = "replay finished"
        log(f"SAM replay finished: fed SAM-6D {self.fed} frames, display dropped {self.dropped_display}")

    # ── SAM-6D results ──────────────────────────────────────────────────────
    def run_results(self):
        jr = None
        ready = self.out / "sam6d" / "READY"
        while not self.stop.is_set():
            if jr is None:
                # sam6d_infer writes READY and then (re)creates the result channel; attaching
                # earlier could bind to a stale segment left by a previous run
                if not ready.exists() or time.time() - ready.stat().st_mtime < 1.0:
                    time.sleep(0.2)
                    continue
                try:
                    jr = JsonReader()
                except FileNotFoundError:
                    time.sleep(0.2)
                    continue
            r = jr.read_new()
            if r is None:
                time.sleep(0.01)
                continue
            self.sam6d_frames += 1
            stamp = int(r["stamp_ns"])
            done_wall = time.monotonic()
            frame_age = ((done_wall - self.wall0) - (stamp - self.t0_ns) / 1e9 / self.a.rate) if self.wall0 else None
            placed = []
            for d in r.get("dets", []):
                res = self.fusion.add_estimate(stamp, d["object"], d["R"], d["t_mm"], d.get("score", 0.0))
                placed.append(res)
                self.write_log("sam6d_estimates", {
                    "t_ns": stamp, "object": d["object"], "score": d.get("score"), "R": d["R"], "t_mm": d["t_mm"],
                    "placed": res["placed"], "slam_interp": res.get("slam"),
                    "anchor_kf": res.get("kf"), "near_loop_closure": res.get("near_loop"),
                    "T_w_obj": mat16(res.get("T_w_obj")), "result_age_s": frame_age, "ms": r.get("ms")})
            self.write_log("sam6d_frames", {"t_ns": stamp, "dets": [{k: d.get(k) for k in ("object", "score", "R", "t_mm")}
                                                                   for d in r.get("dets", [])]})
            if self.memory is not None:
                self.memory.add_frame(stamp, [(d["object"], d.get("score", 0.0), d["R"], d["t_mm"])
                                              for d in r.get("dets", [])])
            self.last_result = {"t_ns": stamp, "n": len(r.get("dets", [])), "ms": r.get("ms"),
                                "age_s": frame_age, "objects": [p["name"] for p in placed if p["placed"]],
                                "unplaced": [p["name"] for p in placed if not p["placed"]]}
            self.sam6d_runs.append(frame_age)

    # ── render + broadcast ──────────────────────────────────────────────────
    def run_render(self):
        period = 1.0 / self.a.view_hz
        last_traj = 0.0
        while not self.stop.is_set():
            t_loop = time.monotonic()
            if self.frames:
                target = self.replay_t_ns - int(self.a.display_delay * 1e9)
                pick = None
                for t_ns, idx, img in reversed(self.frames):
                    if t_ns <= target:
                        pick = (t_ns, idx, img)
                        break
                if pick is None:
                    pick = self.frames[0]
                t_ns, idx, img = pick
                T_ws, slam_status, rows = self.fusion.view(t_ns)
                if self.memory is not None:
                    rows = self.memory_rows(t_ns, T_ws, rows)
                canvas = draw_objects(img.copy(), self.K, self.dist, rows, self.extents, self.a.overlay_max_age)
                ok, jpg = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 78])
                T_wslam, _ = self.poses.at(t_ns)
                payload = {
                    "type": "frame", "t_ns": t_ns, "t_rel_s": round((t_ns - self.t0_ns) / 1e9, 3) if self.t0_ns else 0,
                    "frame_idx": idx, "jpeg": base64.b64encode(jpg.tobytes()).decode(),
                    "slam_status": slam_status, "T_w_sam": mat16(T_ws), "T_w_slam": mat16(T_wslam),
                    "objects": [{"name": r["name"], "source": r["source"], "age_s": round(r["age_s"], 2),
                                 "score": round(r["score"], 3), "est_count": r["est_count"],
                                 "anchor_kf": r.get("anchor_kf"), "near_loop": r.get("near_loop"),
                                 "status": r.get("status"), "instance": r.get("instance"), "multi": r.get("multi", False),
                                 "confidence": r.get("confidence"),
                                 "T_w_obj": mat16(r["T_w_obj"]), "T_cam_obj": mat16(r["T_cam_obj"]),
                                 "size_mm": self.extents.get(r["name"], {}).get("size_mm"),
                                 "center_mm": (np.add(self.extents[r["name"]]["min_mm"], self.extents[r["name"]]["max_mm"]) / 2).tolist()
                                 if r["name"] in self.extents else None,
                                 "history": r["history"][-20:]} for r in rows],
                    "stats": self.stats(),
                }
                self.write_log("display_objects", {"t_ns": t_ns, "frame_idx": idx, "slam": slam_status,
                                                   "objects": [{"name": o["name"], "source": o["source"],
                                                                "age_s": o["age_s"], "T_cam_obj": o["T_cam_obj"]}
                                                               for o in payload["objects"]]})
                if time.monotonic() - last_traj > 1.0:
                    payload["trajectory"] = self.poses.positions(step=3)
                    last_traj = time.monotonic()
                self.broadcast(payload)
            time.sleep(max(0.0, period - (time.monotonic() - t_loop)))

    def run_memory(self):
        while not self.stop.is_set():
            if self.memory is not None:
                self.memory.update()
            time.sleep(0.2)

    def memory_rows(self, t_ns, T_ws, fusion_rows, exact_window_ns=20_000_000):
        """display rows from the object memory: one per kept instance, in the corrected map."""
        hist = {r["name"]: r["history"] for r in fusion_rows}
        lms = [l for l in self.memory.landmarks() if l["status"] in ("active", "lost", "remembered")]
        per_class = {}
        for l in lms:
            per_class[l["name"]] = per_class.get(l["name"], 0) + 1
        rows = []
        for l in lms:
            T_w_obj = l["T_w_obj"]
            rows.append({
                "name": l["name"], "instance": l["object_id"], "multi": per_class[l["name"]] > 1,
                "status": l["status"], "score": float(l["score"] or 0.0), "confidence": l["confidence"],
                "T_w_obj": T_w_obj, "age_s": (t_ns - l["last_seen_ns"]) / 1e9, "est_count": l["n_obs"],
                "anchor_kf": None, "near_loop": False, "history": hist.get(l["name"], []),
                "T_cam_obj": None if T_ws is None else inv_se3(T_ws) @ T_w_obj,
                "source": ("no_slam" if T_ws is None else
                           "sam6d" if abs(t_ns - l["last_seen_ns"]) <= exact_window_ns else "slam_interp"),
            })
        return rows

    def stats(self):
        lags = [v for v in self.sam6d_runs if v is not None]
        return {
            "status": self.status,
            "extrinsic": self.extrinsic_source,
            "clock": (f"frame clock, SAM tau {self.sam_tau_ns / 1e6:+.1f} ms" if self.raw_sam
                      else f"header stamps, SAM tau {self.sam_tau_ns / 1e6:+.1f} ms"),
            "features": self.a.features,
            "slam_source": (self.hello or {}).get("source", self.a.slam),
            "slam_params": self.a.slam_param,
            "slam_state": self.poses.last_state,
            "slam_poses": self.poses.count_all, "slam_ok": self.poses.count_ok,
            "slam_track_ms": getattr(self, "slam_track_ms", None),
            "slam_lag_s": None if getattr(self, "slam_lag", None) is None else round(self.slam_lag, 3),
            "slam_dropped": getattr(self, "slam_dropped", 0),
            "slam_map_changes": self.poses.map_changes,
            "slam_loop_events": getattr(self, "slam_loop_events", 0),
            "sam6d_frames": self.sam6d_frames, "sam6d_fed": self.fed,
            "sam6d_last": self.last_result,
            "sam6d_age_median_s": round(float(np.median(lags)), 2) if lags else None,
            "objects_unplaced": self.fusion.unplaced,
            "kf_anchoring": getattr(self, "kf_anchoring", False),
            "kf_count": len(self.poses.kfs.T), "kf_updates": self.poses.kfs.updates,
            "kf_culled": self.poses.kfs.culled,
            "kf_max_correction_cm": round(self.poses.kfs.max_correction_m * 100, 2),
            "near_loop_estimates": self.fusion.near_loop_estimates,
            "memory": None if self.memory is None else self.memory.stats(),
            "display_delay_s": self.a.display_delay,
            "slam_refined_poses": getattr(self, "refined", 0),
            "overlay_max_age_s": self.a.overlay_max_age,
            "replay_t_rel_s": round((self.replay_t_ns - self.t0_ns) / 1e9, 2) if self.t0_ns else 0,
            "duration_s": round((int(self.sam.t_ns[-1]) - self.t0_ns) / 1e9, 1) if self.t0_ns else None,
        }

    def broadcast(self, payload):
        data = ("data: " + json.dumps(payload, separators=(",", ":")) + "\n\n").encode()
        with self.clients_lock:
            for q in list(self.clients):
                if q.qsize() > 4:
                    try:
                        q.get_nowait()
                    except queue.Empty:
                        pass
                q.put(data)

    # ── http ────────────────────────────────────────────────────────────────
    def http(self):
        hub = self
        page = (HERE / "web" / "index.html")

        class H(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    body = page.read_bytes()
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == "/events":
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "text/event-stream")
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    q = queue.Queue()
                    with hub.clients_lock:
                        hub.clients.append(q)
                    try:
                        init = {"type": "init", "K": hub.K.tolist(), "width": hub.sam.W, "height": hub.sam.H,
                                "X_slam_sam": mat16(hub.fusion.X), "trajectory": hub.poses.positions(step=3)}
                        self.wfile.write(("data: " + json.dumps(init) + "\n\n").encode())
                        while not hub.stop.is_set():
                            try:
                                self.wfile.write(q.get(timeout=5))
                            except queue.Empty:
                                self.wfile.write(b": keepalive\n\n")
                            self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                    finally:
                        with hub.clients_lock:
                            hub.clients.remove(q)
                elif self.path == "/stats":
                    body = json.dumps(hub.stats(), default=str).encode()
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(body)
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)

        srv = ThreadingHTTPServer((self.a.http_host, self.a.http_port), H)
        srv.daemon_threads = True
        log(f"viewer: http://localhost:{self.a.http_port}")
        srv.serve_forever()

    # ── lifecycle ───────────────────────────────────────────────────────────
    def run(self):
        self.fw = FrameWriter()
        threads = [threading.Thread(target=self.http, daemon=True),
                   threading.Thread(target=self.slam_server, daemon=True),
                   threading.Thread(target=self.run_results, daemon=True),
                   threading.Thread(target=self.run_render, daemon=True),
                   threading.Thread(target=self.run_memory, daemon=True)]
        for t in threads:
            t.start()
        if not self.a.no_sam6d:
            self.start_sam6d()
        if not self.a.no_mac:
            self.start_mac_slam()
        replay = threading.Thread(target=self.run_replay, daemon=True)
        replay.start()
        try:
            while True:
                time.sleep(1.0)
                for p in self.procs:
                    if p.poll() is not None and not getattr(p, "_reported", False):
                        p._reported = True
                        log(f"subprocess pid={p.pid} exited rc={p.returncode}")
                if self.replay_done and not getattr(self, "_summary_written", False):
                    time.sleep(self.a.tail_s)
                    self.write_summary()
                    self._summary_written = True
                    self.status = "finished — viewer stays up (Ctrl+C to exit)"
                    if self.a.exit_when_done:
                        break
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def write_summary(self):
        s = self.stats()
        s["objects"] = {k: {"est_count": v.est_count, "T_w_obj": mat16(self.fusion.world(v.current)),
                            "t_est_ns": v.current.t_ns, "anchor_kf": v.current.kf}
                        for k, v in self.fusion.objects.items()}
        s["t0_ns"] = self.t0_ns
        s["observations_final"] = self.fusion.dump_observations()
        if self.memory is not None:
            self.memory.update()
            s["memory_landmarks"] = [{**{k: v for k, v in l.items() if k != "T_w_obj"}, "T_w_obj": mat16(l["T_w_obj"])}
                                     for l in self.memory.landmarks()]
        s["slam_end"] = self.slam_end
        (self.out / "summary.json").write_text(json.dumps(s, indent=1, default=str))
        for f in self.logs.values():
            f.flush()
        log(f"summary written: {self.out / 'summary.json'}")

    def shutdown(self):
        self.stop.set()
        for p in self.procs:
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.time() + 20
        for p in self.procs:
            try:
                p.wait(timeout=max(0.1, deadline - time.time()))
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
        for f in self.logs.values():
            f.close()
        self.fw.close()
        log("stopped")


def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ds = "/home/jucpark/DeepLearning/Dataset/260826_etri_eightcircle_dark"
    ap.add_argument("--sam-session", default=f"{ds}/SAM")
    ap.add_argument("--slam-session", default=f"{ds}/SLAM", help="PC copy of the SLAM session (for its frame clock)")
    ap.add_argument("--extrinsic", default=str(REPO / "objpose" / "rt" / "X_slam_sam.json"))
    ap.add_argument("--out", default=str(REPO / "objpose" / "output" / time.strftime("run_%y%m%d_%H%M%S")))
    ap.add_argument("--mac-host", default="mac")
    ap.add_argument("--mac-runner", default="~/objpose/run_slam.sh")
    ap.add_argument("--mac-slam-session", default="~/Documents/DefenseMeta/Dataset/260826_etri_eightcircle_dark/SLAM")
    ap.add_argument("--features", type=int, default=2000)
    ap.add_argument("--slam", choices=["orbslam3", "rtabmap", "lidar"], default="orbslam3", help="SLAM backend on the Mac")
    ap.add_argument("--slam-param", action="append", default=[], metavar="Key=Value",
                    help="backend parameter override passed to the Mac streamer (repeatable)")
    ap.add_argument("--slam-port", type=int, default=17001)
    ap.add_argument("--http-host", default="0.0.0.0")
    ap.add_argument("--http-port", type=int, default=8765)
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--start-s", type=float, default=0.5, help="start this far into the overlap")
    ap.add_argument("--duration-s", type=float, default=0.0, help="0 = to the end")
    ap.add_argument("--sam-feed-hz", type=float, default=10.0)
    ap.add_argument("--view-hz", type=float, default=15.0)
    ap.add_argument("--display-delay", type=float, default=0.3)
    ap.add_argument("--overlay-max-age", type=float, default=60.0,
                    help="draw an object on the video only if SAM-6D saw it within this many seconds")
    ap.add_argument("--max-gap", type=float, default=0.25, help="max SLAM pose gap to interpolate across [s]")
    ap.add_argument("--tail-s", type=float, default=8.0)
    ap.add_argument("--no-mac", action="store_true", help="don't launch the Mac; wait for an external SLAM client")
    ap.add_argument("--no-sam6d", action="store_true")
    ap.add_argument("--no-memory", action="store_true", help="show raw latest estimates instead of the object memory")
    ap.add_argument("--assoc-gate-m", type=float, default=0.15, help="object memory association distance gate")
    ap.add_argument("--exit-when-done", action="store_true")
    Hub(ap.parse_args()).run()


if __name__ == "__main__":
    main()
