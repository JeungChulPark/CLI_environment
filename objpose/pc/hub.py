#!/usr/bin/env python3
"""hub.py — PC side of the object-pose system.

    SLAM host(s) ──poses over TCP──┐   one "lane" per SLAM stream: ORB-SLAM3 / RTAB-Map on the Mac,
                                   ▼   KISS-ICP or hdl_graph_slam (LiDAR) on the Mac or on this PC
    SAM session ──1x replay──► shared memory ──► sam6d_infer.py (unchanged)
          │                                        │ results (shared memory)
          └──────── display frames ──► Fusion per lane ◄─┘
                                          │
                                  http://<pc>:8765  (one video per lane + one 3D view, lanes colour-coded)

The hub commands each SLAM host (ssh tunnel / direct / manual / local), waits until every lane has said
hello and SAM-6D is READY, then starts every replay at the same dataset time t0.

Link modes (--link):
    tunnel  ssh -R: the Mac connects to its own 127.0.0.1:PORT
    direct  ssh runs the streamer on the SLAM host with --connect <pc-host>:PORT, no tunnel
    manual  nothing is launched; the exact command is printed and written to <out>/mac_command*.txt
    local   the streamer runs on this PC (bash), e.g. the hdl_graph_slam LiDAR lane

Lanes (2026-09-14): the legacy arguments (--slam, --slam-port, --extrinsic, --mac-slam-session, ...) define
lane 0; each --lane "key=value,..." adds another SLAM stream that gets the same SAM-6D detections:
    --lane name=hdl,slam=hdl,port=17002,ssh=local,session=/mnt/.../260910_object/lidar,extrinsic=objpose/rt/260910/X_lidar_sam.json
keys: name label slam port ssh(alias|local) link host session extrinsic tau features params(a=b;c=d) runner color
Every lane has its own pose buffer, fusion, object memory and logs (<key>_<name>.jsonl). For display the
lanes are drawn in ONE map: the reference lane is the first LiDAR lane (z-up, metric), and every other lane
is rigidly aligned to it with its SAM-camera trajectory (first pose, then a Kabsch fit over the first 15 s of
overlap, then frozen), so a divergence between the lanes shows as such.
"""
from __future__ import annotations

import argparse
import base64
import copy
import json
import os
import queue
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
import zlib
from collections import OrderedDict, deque
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
from bop_session import BopSession                      # noqa: E402
from fusion import Fusion, PoseBuffer, inv_se3          # noqa: E402
from memory import ObjectMemory                         # noqa: E402
from rs_session import RsSession                        # noqa: E402
from live_session import LiveSamSession                 # noqa: E402
from object_extents import load_object_extents           # noqa: E402
from memory import DepthCheckConfig                      # noqa: E402
from shm_channel import FrameWriter, JsonReader         # noqa: E402

SAM_MINUS_SLAM_CLOCK_NS = 18225662057   # SLAM/peer_timestamp_comparison.json
PY_SAM6D = "/home/jucpark/anaconda3/envs/sam6d/bin/python"

# SAM-6D opt-in 속도 옵션 프리셋 (--sam6d-speed).
# 후보값 출처: sam6d_realtime/_speed_opt_2026_09_13/REPORT.md §9 "권장 설정"
# 채택 여부는 이 배치(9객체 + verify 스택, 260910_object)에서 직접 재본 결과로 정했다
# — objpose/docs/sam6d_inference_speed_260914.md "실측 결과" 참고.
#   fine_npoint 1024 : total -9%, 검출 수/포즈 유지(1.2 mm / 1.3°)  → 채택
#   mask_cache       : 카메라가 움직이면 재사용률 2%(이득 ~0)지만 해도 무해   → 채택
#   precision bf16   : 검출 채택률이 fp32 대비 17~22% 로 붕괴(verify 임계값이
#                      fp32 로 캘리브레이션돼 있다)                          → 제외
#   pem_schedule     : carried 포즈가 카메라 이동만큼 낡아 77~131 mm 오차     → 제외
# 두 제외 항목은 "report" 프리셋으로 그대로 재현할 수 있게 남겨 둔다.
_SAM6D_MASK_CACHE = {"enabled": True, "iou_thresh": 0.90, "max_age": 5, "interval": 30}
SAM6D_SPEED_PRESETS = {
    "off": {},                                   # 2026-09-14 이전 동작 (fp32 / 캐시 off / 예산 off)
    "fast": {"fine_npoint": 1024, "mask_cache": dict(_SAM6D_MASK_CACHE)},
    "report": {"precision": "bf16", "fine_npoint": 1024, "coarse_npoint": 196,
               "mask_cache": dict(_SAM6D_MASK_CACHE),
               "pem_schedule": {"enabled": True, "budget": 1, "max_age": 4}},
}

BACKENDS = ("orbslam3", "rtabmap", "lidar", "hdl")
DEPTH_DS = 8          # depth kept for the object memory's depth check: every 8th pixel
BACKEND_NAME = {"orbslam3": "orb", "rtabmap": "rtab", "lidar": "kiss", "hdl": "hdl"}
BACKEND_LABEL = {"orbslam3": "ORB-SLAM3", "rtabmap": "RTAB-Map", "lidar": "LiDAR KISS-ICP", "hdl": "LiDAR hdl_graph_slam"}
LANE_COLORS = ["#56c2e0", "#f2a03d", "#c792ea", "#3ddc97"]
# default lane colour by backend (ORB blue, LiDAR orange, RTAB purple) so a single-backend run keeps the same
# colour everywhere (lane card, 3D boxes, launcher mode bar); a second lane of the same backend gets the next free one
BACKEND_COLOR = {"orbslam3": "#56c2e0", "rtabmap": "#a78bfa", "lidar": "#f2a03d", "hdl": "#f2a03d"}


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


def auto_color(name: str):
    """stable, well-separated box colour for an object PALETTE has no entry for (e.g. the 21 YCB-V
    objects): hue from the name's hash, full saturation, so every object reads apart in the video"""
    h = (zlib.crc32(name.encode()) % 180)
    return tuple(int(v) for v in cv2.cvtColor(np.uint8([[[h, 210, 255]]]), cv2.COLOR_HSV2BGR)[0, 0])


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


def hex_bgr(h):
    h = h.lstrip("#")
    return (int(h[4:6], 16), int(h[2:4], 16), int(h[0:2], 16))


def draw_objects(img, K, dist, rows, extents, max_age_s=float("inf"), tag=None, tag_color=None):
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
        col = PALETTE.get(r["name"]) or auto_color(r["name"])
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
    if tag:
        (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(img, (6, 6), (tw + 18, th + 18), (20, 20, 20), -1)
        cv2.putText(img, tag, (12, th + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, tag_color or (255, 255, 255), 2, cv2.LINE_AA)
    return img


def mat16(T):
    return None if T is None else [round(float(v), 6) for v in np.asarray(T).reshape(-1)]


def kabsch(P, Q):
    """rigid T (4x4) with Q ≈ R P + t, least squares over paired points (rows)"""
    P, Q = np.asarray(P, np.float64), np.asarray(Q, np.float64)
    mp, mq = P.mean(0), Q.mean(0)
    H = (P - mp).T @ (Q - mq)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = mq - R @ mp
    res = np.linalg.norm((R @ P.T).T + T[:3, 3] - Q, axis=1)
    return T, float(np.sqrt(np.mean(res ** 2)))


def parse_lane_spec(spec: str) -> dict:
    """'k=v,k=v' (values may contain / and :); params use ';' between Key=Value pairs"""
    out = {}
    for item in spec.split(","):
        if not item.strip():
            continue
        k, _, v = item.partition("=")
        out[k.strip()] = v.strip()
    return out


# ── SLAM lane ────────────────────────────────────────────────────────────────
class Lane:
    """one SLAM stream: launch, pose server, pose buffer, fusion, object memory, logs, display alignment"""

    def __init__(self, hub: "Hub", idx: int, spec: dict):
        self.hub, self.idx = hub, idx
        a = hub.a
        self.backend = spec.get("slam", a.slam)
        if self.backend not in BACKENDS:
            raise ValueError(f"lane {idx}: unknown slam backend {self.backend!r}")
        # --slam-name / --slam-label: the first lane's display name when its streamer is not the backend's usual one
        # (run_live.sh loc-nav feeds the "lidar" lane from the fixed-map localizer, not from KISS-ICP)
        self.name = spec.get("name") or (a.slam_name if idx == 0 else "") or BACKEND_NAME[self.backend]
        self.label = spec.get("label") or (a.slam_label if idx == 0 else "") or BACKEND_LABEL[self.backend]
        self.port = int(spec.get("port", a.slam_port))
        self.ssh = spec.get("ssh", a.mac_host)
        self.link = spec.get("link", a.link)
        if self.ssh == "local":
            self.link = "local"
        self.host = spec.get("host", a.pc_host if self.link == "local" else a.slam_host)
        self.session = spec.get("session", a.mac_slam_session)
        self.pc_session = spec.get("pc_session", a.slam_session if idx == 0 else "")
        self.features = int(spec.get("features", a.features))
        self.params = [p for p in spec.get("params", "").split(";") if p] if "params" in spec else list(a.slam_param)
        self.runner = spec.get("runner", a.mac_runner)
        used = {l.color for l in hub.lanes} if hasattr(hub, "lanes") else set()
        default = BACKEND_COLOR.get(self.backend, LANE_COLORS[idx % len(LANE_COLORS)])
        if default in used:
            default = next((c for c in LANE_COLORS if c not in used), LANE_COLORS[idx % len(LANE_COLORS)])
        self.color = spec.get("color", default)
        xpath = Path(spec.get("extrinsic", a.extrinsic))
        self.X, self.xsrc = load_extrinsic(xpath)
        xj = json.loads(xpath.read_text()) if xpath.suffix == ".json" else {}
        self.xj = xj
        self.tau_s = float(spec["tau"]) if "tau" in spec else float(xj.get("sam_tau_s", 0.0))
        self.is_lidar = self.backend in ("lidar", "hdl")
        # frame-index remapping of SLAM poses only exists for raw camera SLAM sessions
        self.slam_clock = (frame_clock(self.pc_session, hub.clock_dir)
                           if not self.is_lidar and self.pc_session
                           and (Path(self.pc_session) / "rgbd_timestamp_associations.json").exists() else None)
        self.poses = PoseBuffer(max_gap_s=a.max_gap)
        self.fusion: Fusion | None = None
        self.memory: ObjectMemory | None = None
        self.hello = None
        self.sock = None
        self.end = None
        self.unrefined = {}
        self.refined = 0
        self.lag = None
        self.track_ms = None
        self.dropped = 0
        self.loop_events = 0
        self.kf_anchoring = False
        self.last_kf_update = None
        self.proc = None
        self.logs = {}
        # display alignment into the reference lane's world: T_ref_lane
        self.align = None
        self.align_frozen = False
        self.align_info = {"pairs": 0, "rmse_cm": None, "span_s": 0.0, "frozen": False, "mode": "none"}
        self.sfx = ""

    def init_fusion(self, global_tau_ns: int, K, img_size, no_memory: bool, assoc_gate_m: float, depth_check=None):
        self.tau_ns = int(round(self.tau_s * 1e9))
        self.fusion = Fusion(self.X, self.poses, t_offset_ns=self.tau_ns - global_tau_ns)
        # position-only association: box-like objects flip 180 deg between SAM-6D estimates,
        # and a rotation gate split one object into two instances on this dataset
        self.memory = None if no_memory else ObjectMemory(self.fusion, K, img_size, depth_check=depth_check,
                                                          assoc_trans_gate_m=assoc_gate_m, assoc_rot_gate_deg=None)

    def open_logs(self, out: Path, sfx: str):
        self.sfx = sfx
        self.logs = {k: open(out / f"{k}{sfx}.jsonl", "w", encoding="utf-8")
                     for k in ("slam_poses", "sam6d_estimates", "display_objects", "kf_updates")}

    def write_log(self, key, obj):
        with self.hub.log_lock:
            self.logs[key].write(json.dumps(obj, ensure_ascii=False) + "\n")

    # ── launch ──────────────────────────────────────────────────────────────
    def connect_addr(self) -> str:
        if self.link in ("tunnel", "local"):
            return f"127.0.0.1:{self.port}"
        return f"{self.hub.a.pc_host}:{self.port}"

    def remote_command(self) -> str:
        connect = self.connect_addr()
        rate = self.hub.a.rate
        extra = " ".join(f"--{k.replace('_', '-')} {v}" for k, v in (x.split("=", 1) for x in self.params))
        if self.backend == "lidar":
            runner = self.hub.a.lidar_runner        # default: the Mac/.3 venv; the laptop uses run_lidar_stream.sh (conda env)
            return f"{runner} --session {self.session} --mode live --connect {connect} --rate {rate} {extra}"
        if self.backend == "hdl":
            if self.link == "local":
                runner = shlex.quote(str(REPO / "objpose" / "lidar" / "run_hdl_stream.sh"))
                extra += (f" --launch-log {shlex.quote(str(self.hub.out / f'hdl_launch{self.sfx}.log'))}"
                          f" --dump-dir {shlex.quote(str(self.hub.out / f'hdl_dump{self.sfx}'))}")
            else:
                runner = "~/objpose/lidar/run_hdl_stream.sh"
            return f"{runner} --session {self.session} --mode live --connect {connect} --rate {rate} {extra}"
        return (f"{self.runner} --slam {self.backend} --session {self.session} --features {self.features} "
                f"--time-source frame {' '.join('--param ' + x for x in self.params)} "
                f"--mode live --connect {connect} --rate {rate}")

    def start(self):
        out = self.hub.out
        remote = self.remote_command()
        if self.link == "manual":
            (out / f"mac_command{self.sfx}.txt").write_text(f"# run on the SLAM host ({self.host}) after this hub is up\n{remote}\n")
            log(f"[{self.name}] link=manual: SLAM streamer is NOT launched. Run this on {self.host} "
                f"(also in {out / f'mac_command{self.sfx}.txt'}):")
            log(f"    {remote}")
            return
        if self.link == "local":
            cmd = ["bash", "-c", remote]
        else:
            cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ExitOnForwardFailure=yes", "-o", "ServerAliveInterval=10"]
            if self.link == "tunnel":
                cmd += ["-R", f"{self.port}:127.0.0.1:{self.port}"]
            cmd += [self.ssh, remote]
        f = open(out / f"mac_slam{self.sfx}.log", "w")
        # ssh must not see a conda LD_LIBRARY_PATH (its libcrypto 3.6 breaks the system OpenSSH: "OpenSSL version mismatch")
        env = dict(os.environ)
        if cmd[0] == "ssh":
            env.pop("LD_LIBRARY_PATH", None)
        self.proc = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True, env=env)
        self.hub.procs.append(self.proc)
        (out / f"mac_command{self.sfx}.txt").write_text(" ".join(cmd) + "\n")
        log(f"[{self.name}] {self.backend} launched ({'locally' if self.link == 'local' else 'over ssh ' + self.ssh}), "
            f"link={self.link}: {remote}")

    # ── pose server ─────────────────────────────────────────────────────────
    def server(self):
        hub = self.hub
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        bind = hub.a.slam_bind or ("127.0.0.1" if self.link in ("tunnel", "local") else "0.0.0.0")
        srv.bind((bind, self.port))
        srv.listen(1)
        srv.settimeout(1.0)
        log(f"[{self.name}] waiting for SLAM stream on {bind}:{self.port} (link={self.link})")
        while not hub.stop.is_set():
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock = conn
            buf = b""
            conn.settimeout(1.0)
            while not hub.stop.is_set():
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
                        self.on_msg(json.loads(line))
            log(f"[{self.name}] SLAM stream closed")
            self.sock = None
        srv.close()

    def send(self, obj):
        self.sock.sendall((json.dumps(obj) + "\n").encode())

    def smoothing_streamer(self) -> bool:
        return bool(self.hello) and int((self.hello.get("params") or {}).get("smooth_scans", 1) or 1) > 1

    def on_msg(self, m):
        hub = self.hub
        kind = m.get("type")
        if kind == "hello":
            self.hello = m
            # the streamer may still stamp with association times: t0 is always on the frame clock
            self.hello["first_ns_frame"] = (int(self.slam_clock["frame_ns"][0]) if self.slam_clock is not None
                                            else int(m["first_ns"]))
            log(f"[{self.name}] SLAM hello: {m}")
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
                self.loop_events += 1
            lag = None
            if hub.wall0 is not None:
                lag = (recv - hub.wall0) - (int(m["t_ns"]) - hub.t0_ns) / 1e9 / hub.a.rate
            m["recv_lag_s"] = None if lag is None else round(lag, 4)
            if self.smoothing_streamer():
                self.unrefined[int(m["t_ns"])] = m      # logged once its smoothed pose arrives
            else:
                self.write_log("slam_poses", m)
            self.lag = lag
            self.track_ms = m.get("track_ms")
            self.dropped = m.get("dropped", 0)
        elif kind == "pose_refine":
            # delayed smoothed pose (lidar_stream --smooth-scans): replaces the raw one in the buffer; the log keeps
            # the final pose in T_wc and the streamed one in T_wc_raw
            recv = time.monotonic()
            ok = self.poses.refine(int(m["t_ns"]), m["T_wc"])
            p = self.unrefined.pop(int(m["t_ns"]), None)
            if p is not None:
                p["T_wc_raw"], p["T_wc"], p["smooth_n"] = p["T_wc"], m["T_wc"], m.get("smooth_n")
                if hub.wall0 is not None:
                    p["refine_lag_s"] = round((recv - hub.wall0) - (int(m["t_ns"]) - hub.t0_ns) / 1e9 / hub.a.rate, 4)
                self.write_log("slam_poses", p)
            self.refined += int(ok)
        elif kind == "kf_update":
            self.poses.apply_kf_update(m.get("map_id"), m.get("kfs", []))
            self.write_log("kf_updates", m)
            self.last_kf_update = {"t_ns": m.get("t_ns"), "reason": m.get("reason"), "n": len(m.get("kfs", []))}
            if m.get("reason") != "periodic":
                log(f"[{self.name}] kf_update ({m.get('reason')}): {len(m.get('kfs', []))} keyframes, "
                    f"max correction so far {self.poses.kfs.max_correction_m * 100:.1f} cm")
        elif kind == "end":
            for t in sorted(self.unrefined):
                self.write_log("slam_poses", self.unrefined.pop(t))
            self.end = m
            log(f"[{self.name}] SLAM end: {m}")

    # ── display alignment ───────────────────────────────────────────────────
    def A(self) -> np.ndarray:
        return np.eye(4) if self.align is None else self.align

    def source(self) -> str:
        return (self.hello or {}).get("source", self.backend)

    # ── stats ───────────────────────────────────────────────────────────────
    def stats(self):
        return {
            "name": self.name, "label": self.label, "color": self.color, "backend": self.backend,
            "link": self.link, "slam_host": self.host, "port": self.port,
            "extrinsic": self.xsrc, "sam_tau_s": self.tau_s,
            "features": self.features,
            "slam_source": self.source(),
            "slam_params": self.params,
            "slam_state": self.poses.last_state,
            "slam_poses": self.poses.count_all, "slam_ok": self.poses.count_ok,
            "slam_track_ms": self.track_ms,
            "slam_lag_s": None if self.lag is None else round(self.lag, 3),
            "slam_dropped": self.dropped,
            "slam_map_changes": self.poses.map_changes,
            "slam_loop_events": self.loop_events,
            "objects_unplaced": self.fusion.unplaced,
            "kf_anchoring": self.kf_anchoring,
            "kf_count": len(self.poses.kfs.T), "kf_updates": self.poses.kfs.updates,
            "kf_culled": self.poses.kfs.culled,
            "kf_max_correction_cm": round(self.poses.kfs.max_correction_m * 100, 2),
            "last_kf_update": self.last_kf_update,
            "near_loop_estimates": self.fusion.near_loop_estimates,
            "memory": None if self.memory is None else self.memory.stats(),
            "slam_refined_poses": self.refined,
            "align": self.align_info,
            "is_reference": self is self.hub.ref,
        }


# ── hub ──────────────────────────────────────────────────────────────────────
class Hub:
    def __init__(self, a):
        self.a = a
        self.out = Path(a.out).resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        self.stop = threading.Event()
        self.clock_dir = REPO / "objpose" / "rt" / "clock"
        self.procs: list[subprocess.Popen] = []
        self.log_lock = threading.Lock()
        self.lanes = []
        for i, spec in enumerate([{}] + [parse_lane_spec(x) for x in a.lane]):
            self.lanes.append(Lane(self, i, spec))
        names = [l.name for l in self.lanes]
        if len(set(names)) != len(names):
            raise ValueError(f"lane names must be unique: {names}")
        ports = [l.port for l in self.lanes]
        if len(set(ports)) != len(ports):
            raise ValueError(f"lane ports must be unique: {ports}")
        # the reference lane for display: the first LiDAR lane (z-up, metric), else lane 0
        self.ref = next((l for l in self.lanes if l.is_lidar), self.lanes[0])
        lane0 = self.lanes[0]
        # `--sam-session live`: the RealSense driver's ROS topics on this PC instead of a recorded session
        # (objpose/pc/live_session.py); --sam-tau-s is then the LiDAR-host clock minus this PC's clock
        self.live_sam = str(a.sam_session).strip().lower() == "live"
        self.raw_sam = (not self.live_sam) and (Path(a.sam_session) / "rgbd_timestamp_associations.json").exists()
        # a BOP scene folder (YCB-V / LM / LM-O / HOPE) — images, not a bag, so the clock is synthetic
        self.bop_sam = (not self.live_sam) and (Path(a.sam_session) / "scene_camera.json").exists()
        if self.live_sam:
            tau = 0.0 if a.sam_tau_s is None else a.sam_tau_s
            self.sam_tau_ns = int(round(float(tau) * 1e9))
            self.sam = LiveSamSession(offset_ns=self.sam_tau_ns)
            # the live SAM stamps are already moved onto the SLAM host clock by offset_ns; a lane tau taken from the
            # rig json (recorded-session clocks) would shift the pose lookup by -tau again (2026-09-29: +631 ms,
            # every SAM frame fell after the newest SLAM pose -> slam status null)
            for l in self.lanes:
                l.tau_s = self.sam_tau_ns / 1e9
        elif self.bop_sam:
            # BOP images carry no timestamps: bop_session stamps them at a fixed fps and the pose
            # stream for the same scene is built on that same clock (tools/bop_to_tum.py), so tau is 0
            tau = 0.0 if a.sam_tau_s is None else a.sam_tau_s
            self.sam_tau_ns = int(round(float(tau) * 1e9))
            self.sam = BopSession(a.sam_session, offset_ns=self.sam_tau_ns)
        elif self.raw_sam:
            # raw SDK sessions: association stamps of the SLAM session drift by seconds, so both
            # sides use the colour-metadata frame clock; SAM "System Time" is arrival time and the
            # rig calibration measured its lag (tau_s, SLAM time = SAM time + tau)
            xj = lane0.xj
            self.sam_clock = frame_clock(a.sam_session, self.clock_dir)
            self.sam_tau_ns = int(round(float(xj.get("tau_s", 0.0)) * 1e9)) if "ftime" in str(xj.get("inputs", "")) or "frame clock" in str(xj.get("clock", "")) else 0
            self.sam = RsSession(a.sam_session, offset_ns=-SAM_MINUS_SLAM_CLOCK_NS,
                                 times_ns=self.sam_clock["frame_ns"] + self.sam_tau_ns)
            for l in self.lanes:
                l.tau_s = self.sam_tau_ns / 1e9      # one clock for every lane on raw sessions
        else:
            # converted bags (260910): header stamps are already on the shared clock; the replay clock uses
            # lane 0's tau (or --sam-tau-s), other lanes get their tau difference as a pose-lookup offset
            tau = lane0.tau_s if a.sam_tau_s is None else a.sam_tau_s
            self.sam_tau_ns = int(round(float(tau) * 1e9))
            self.sam = ConvSession(a.sam_session, offset_ns=self.sam_tau_ns, cache_dir=self.clock_dir)
        self.K = self.sam.K
        self.dist = np.asarray(self.sam.kc, np.float64)
        for l in self.lanes:
            l.init_fusion(self.sam_tau_ns, self.sam.K, (self.sam.W, self.sam.H), a.no_memory, a.assoc_gate_m,
                          depth_check=None if a.no_depth_check else DepthCheckConfig(
                              free_margin_m=a.depth_free_margin_m, confirm_frames=a.depth_confirm_frames,
                              max_speed_mps=a.depth_max_speed, max_rot_dps=a.depth_max_rot))
            l.open_logs(self.out, "" if len(self.lanes) == 1 else f"_{l.name}")
        # box sizes from the model points SAM-6D uses (cad_extents.json only for objects without them)
        self.extents = load_object_extents(log=log)
        self.clients: list[queue.Queue] = []
        self.clients_lock = threading.Lock()
        self.frames = deque(maxlen=120)            # (t_ns, index, bgr)
        # downsampled aligned depth of the frames handed to SAM-6D, keyed by stamp, until their result arrives
        self.depth_small: "OrderedDict[int, np.ndarray]" = OrderedDict()
        self.depth_lock = threading.Lock()
        # OBJPOSE_ONLINE_DESIGNS=1: map designs (consensus fusion, depth confirm/refine, weak evidence) computed during the run
        # on the first lane's memory (online_designs.py). Needs the full-resolution depth of the frames SAM-6D processed.
        # pose of each map object shown in the viewer: consensus among its observations (consensus_pose.py). OBJPOSE_MAP_POSE=mean → running mean (old behaviour)
        self.map_pose = os.environ.get("OBJPOSE_MAP_POSE", "consensus")
        for l in self.lanes:
            l.consensus = None
            if self.map_pose == "consensus" and l.memory is not None:
                try:
                    from consensus_pose import ConsensusPose
                    l.consensus = ConsensusPose(l.memory)
                except Exception as e:                                  # noqa: BLE001
                    self.map_pose = "mean"; log(f"map pose: consensus unavailable ({type(e).__name__}: {e}), using mean")
        log(f"map pose: {self.map_pose}")
        self.online = None
        if os.environ.get("OBJPOSE_ONLINE_DESIGNS", "0") == "1" and self.lanes[0].memory is not None:
            from online_designs import OnlineDesigns
            self.online = OnlineDesigns(self.lanes[0].memory, self.sam.K, (self.sam.W, self.sam.H))
            log("online map designs: on")
        self.depth_full: "OrderedDict[int, np.ndarray]" = OrderedDict()
        self.pose_wait_stats = {"waited": 0, "timed_out": 0, "max_wait_s": 0.0}
        # where the robot stood when it recognised each object: name -> [(object xyz, base x, y, yaw, camera range, score)].
        # The viewer sends the robot back to one of these ("go to this object"): a place it has been, with the object in view.
        self.views: dict[str, list] = {}
        self.t0_ns = None
        self.wall0 = None
        self.replay_t_ns = 0
        self.replay_done = False
        self.fed = 0
        self.dropped_display = 0
        self.sam6d_runs = []                       # latency records
        self.sam6d_frames = 0
        self.last_result = None
        self.frames_log = open(self.out / "sam6d_frames.jsonl", "w", encoding="utf-8")
        self.status = "starting"
        log(f"SAM session {self.sam.dir} ({'live camera' if self.live_sam else 'BOP scene' if self.bop_sam else 'raw' if self.raw_sam else 'converted'}) frames={len(self.sam)} "
            f"SAM tau {self.sam_tau_ns / 1e6:+.1f} ms")
        for l in self.lanes:
            log(f"lane {l.idx} [{l.name}] {l.backend} host={l.host} link={l.link} port={l.port} "
                f"extrinsic={l.xsrc} tau {l.tau_s * 1e3:+.1f} ms{' (reference)' if l is self.ref else ''}")

    # ── subprocesses ────────────────────────────────────────────────────────
    def start_sam6d(self):
        cfg = yaml.safe_load((SAM6D / "realtime" / "run_split_example.yaml").read_text(encoding="utf-8"))
        cfg["output"] = {"dir": str(self.out / "sam6d"), "diagnostics": False}
        cfg["runtime"]["idle_exit_s"] = 0
        cfg.pop("bag", None)
        cfg["anchor"]["enabled"] = False
        if self.a.ism_config:
            # a different object set (e.g. configs/yolo_ism_ycbv.yaml for the BOP YCB-V objects)
            cfg["ism"]["config"] = self.a.ism_config
        if self.a.ism_objects:
            # look for these objects only. On BOP this is the protocol's own setting: a scene comes
            # with the list of objects to estimate (test_targets_bop19.json), it is not open-set.
            cfg["ism"]["objects"] = list(self.a.ism_objects)
            log(f"SAM-6D object set restricted to {len(self.a.ism_objects)}: {', '.join(self.a.ism_objects)}")
        # 속도 옵션은 opt-in 이다. off 면 YAML 에 키를 넣지 않아 예전 동작(fp32 / 캐시 off /
        # 예산제 off) 그대로다. fast 면 REPORT.md §9 권장값을 runtime 에 써 넣는다.
        preset = copy.deepcopy(SAM6D_SPEED_PRESETS[self.a.sam6d_speed])
        if preset:
            cfg["runtime"].update(preset)
        log(f"SAM-6D speed preset: {self.a.sam6d_speed} "
            f"({', '.join(f'{k}={v}' for k, v in preset.items()) if preset else 'no speed options'})")
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

    # ── coordinator + SAM replay ────────────────────────────────────────────
    def remember_depth(self, t_ns: int, depth):
        """keep depth[::8, ::8] (80x60 for 640x480, ~10 kB) of a frame sent to SAM-6D for the object memory's depth check"""
        if depth is not None and self.online is not None:
            with self.depth_lock:
                self.depth_full[int(t_ns)] = np.asarray(depth).astype(np.uint16, copy=True)
                while len(self.depth_full) > 200:
                    self.depth_full.popitem(last=False)
        if depth is None or self.a.no_depth_check:
            return
        small = np.ascontiguousarray(np.asarray(depth)[::DEPTH_DS, ::DEPTH_DS]).astype(np.uint16, copy=False)
        with self.depth_lock:
            self.depth_small[int(t_ns)] = small
            while len(self.depth_small) > 600:          # ~60 s of feeds; a result older than that is long processed
                self.depth_small.popitem(last=False)

    def run_live(self):
        """live camera: no timeline to replay — hand SAM-6D the newest frame at --sam-feed-hz until
        --duration-s (0 = until Ctrl+C). Both hosts run on wall clocks, so t0 is 'now' on the SLAM clock."""
        ready = self.out / "sam6d" / "READY"
        self.status = "loading SAM-6D and SLAM (" + ", ".join(l.label for l in self.lanes) + ")"
        while not self.stop.is_set() and not ((self.a.no_sam6d or ready.exists()) and all(l.hello and l.sock for l in self.lanes)):
            time.sleep(0.2)
        if self.stop.is_set():
            return
        # t0 = "now" on the SLAM host clock. No start offset: in live mode every lag figure is (receive time - wall0)
        # - (stamp - t0), so an offset here showed up as a constant fake +0.5 s lag (2026-09-17).
        t0 = time.time_ns() + self.sam_tau_ns
        self.t0_ns = t0
        for l in self.lanes:
            l.send({"type": "start", "t0_ns": t0, "rate": 1.0})
        self.wall0 = time.monotonic()
        self.status = "running (live)"
        log(f"START (live) t0_ns={t0} lanes={[l.name for l in self.lanes]} "
            f"duration={'until Ctrl+C' if self.a.duration_s <= 0 else f'{self.a.duration_s:.0f} s'} "
            f"camera {self.sam.W}x{self.sam.H} pairs so far {self.sam.n_pairs}")
        end_wall = None if self.a.duration_s <= 0 else self.wall0 + self.a.duration_s
        feed_period = 1.0 / self.a.sam_feed_hz
        next_feed = 0.0
        while not self.stop.is_set() and (end_wall is None or time.monotonic() < end_wall):
            fr = self.sam.latest()
            if fr is None:
                time.sleep(0.003)
                continue
            now = time.monotonic()
            self.frames.append((fr.t_ns, fr.index, fr.color_bgr))
            self.replay_t_ns = fr.t_ns
            if now >= next_feed:
                self.fw.write(np.ascontiguousarray(fr.color_bgr[:, :, ::-1]), fr.depth_raw, self.K, fr.t_ns, time.time(),
                              {"tracking_state": "TRACKING_LOST", "map_id": "primary_sam_camera"},
                              depth_stamp_ns=fr.t_ns)
                self.remember_depth(fr.t_ns, fr.depth_raw)
                self.fed += 1
                next_feed = now + feed_period
        self.replay_done = True
        self.status = "live finished"
        log(f"live camera finished: fed SAM-6D {self.fed} frames, camera pairs {self.sam.n_pairs}")
        for l in self.lanes:                      # a live streamer has no last scan: ask it to finish (it answers with `end`)
            try:
                l.send({"type": "stop"})
            except (OSError, AttributeError):
                pass

    def run_replay(self):
        if self.live_sam:
            return self.run_live()
        ready = self.out / "sam6d" / "READY"
        self.status = "loading SAM-6D and SLAM (" + ", ".join(l.label for l in self.lanes) + ")"
        while not self.stop.is_set() and not ((self.a.no_sam6d or ready.exists()) and all(l.hello and l.sock for l in self.lanes)):
            time.sleep(0.2)
        if self.stop.is_set():
            return
        start = self.a.start_s
        t0 = max([int(self.sam.t_ns[0])] + [int(l.hello["first_ns_frame"]) for l in self.lanes]) + int(max(start, 0.3) * 1e9)
        self.t0_ns = t0
        for l in self.lanes:
            l.send({"type": "start", "t0_ns": t0, "rate": self.a.rate})
        self.wall0 = time.monotonic()
        self.status = "running"
        log(f"START t0_ns={t0} (dataset +{(t0 - min(int(l.hello['first_ns_frame']) for l in self.lanes)) / 1e9:.2f}s) "
            f"rate={self.a.rate} lanes={[l.name for l in self.lanes]}")
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
                self.remember_depth(fr.t_ns, depth)
                self.fed += 1
                next_feed = now + feed_period
            i += 1
        self.replay_done = True
        self.status = "replay finished"
        log(f"SAM replay finished: fed SAM-6D {self.fed} frames, display dropped {self.dropped_display}")

    # ── SAM-6D results ──────────────────────────────────────────────────────
    def run_results(self):
        jr = None
        pending = deque()
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
            new = jr.read_new()
            if new is not None:
                self.sam6d_frames += 1
                done_wall = time.monotonic()
                stamp = int(new["stamp_ns"])
                new["_frame_age"] = ((done_wall - self.wall0) - (stamp - self.t0_ns) / 1e9 / self.a.rate) if self.wall0 else None
                pending.append((done_wall, new))
            # A SAM-6D result can arrive before the SLAM pose of its frame (slow or queued SLAM): placing it then finds no
            # pose and drops the detections (f3000 replay: 258 of 399 lost, 2026-09-17). Hold results, in order, until
            # every lane has a pose at or after the frame's stamp, a lane has ended, or --pose-wait-s passed.
            processed = False
            while pending:
                recv, r = pending[0]
                stamp = int(r["stamp_ns"])
                waited = time.monotonic() - recv
                ready = all(l.end is not None or l.poses.latest_t() >= l.fusion.slam_t(stamp) for l in self.lanes)
                if not ready and waited < self.a.pose_wait_s and not self.stop.is_set():
                    break
                pending.popleft()
                if waited > 0.02:
                    self.pose_wait_stats["waited"] += 1
                self.pose_wait_stats["max_wait_s"] = max(self.pose_wait_stats["max_wait_s"], round(waited, 3))
                if not ready:
                    self.pose_wait_stats["timed_out"] += 1
                self.process_result(r, waited)
                processed = True
            if not processed and new is None:
                time.sleep(0.01)

    def remember_view(self, l, d, res):
        """robot pose (map) at one placed estimate; only lanes whose SLAM pose is base_link (hdl-nav, loc-nav)"""
        if ((l.hello or {}).get("params") or {}).get("pose_frame") != "base_link":
            return
        T_sam_obj = np.eye(4); T_sam_obj[:3, :3] = np.asarray(d["R"], np.float64).reshape(3, 3)
        T_sam_obj[:3, 3] = np.asarray(d["t_mm"], np.float64) / 1000.0
        T_w_base = res["T_w_obj"] @ inv_se3(T_sam_obj) @ inv_se3(l.fusion.X)
        v = self.views.setdefault(d["object"], [])
        v.append((res["T_w_obj"][:3, 3].copy(), float(T_w_base[0, 3]), float(T_w_base[1, 3]),
                  float(np.arctan2(T_w_base[1, 0], T_w_base[0, 0])), float(np.linalg.norm(T_sam_obj[:3, 3])), float(d.get("score", 0.0))))
        del v[:-400]

    def best_view(self, name, xyz, want_m=0.9):
        """[x, y, yaw, range] of the stored robot pose that saw this object from nearest to want_m, or None"""
        c = [q for q in self.views.get(name, ()) if np.linalg.norm(q[0] - xyz) < 0.3]
        if not c:
            return None
        near = [q for q in c if abs(q[4] - want_m) <= 0.25] or c         # 0.65-1.15 m is where recognition works best
        xy = np.array([[q[1], q[2]] for q in near]); yaw = np.array([q[3] for q in near])
        # the usual viewing place, not a pose caught once while driving past: most other views within 15 cm and 15 deg
        cnt = [int(((np.linalg.norm(xy - xy[i], axis=1) < 0.15) & (np.abs(np.angle(np.exp(1j * (yaw - yaw[i])))) < 0.26)).sum())
               for i in range(len(near))]
        i = max(range(len(near)), key=lambda i: (cnt[i], -abs(near[i][4] - want_m), near[i][5]))
        q = near[i]
        return [round(q[1], 3), round(q[2], 3), round(q[3], 4), round(q[4], 2)]

    def process_result(self, r, waited):
            stamp = int(r["stamp_ns"])
            frame_age = r.get("_frame_age")
            with self.depth_lock:
                depth_small = self.depth_small.pop(stamp, None)
            dets = r.get("dets", [])
            placed_any = {}
            for l in self.lanes:
                placed = []
                for d in dets:
                    res = l.fusion.add_estimate(stamp, d["object"], d["R"], d["t_mm"], d.get("score", 0.0))
                    placed.append(res)
                    if res["placed"] and l is self.lanes[0]:
                        self.remember_view(l, d, res)
                    l.write_log("sam6d_estimates", {
                        "t_ns": stamp, "object": d["object"], "score": d.get("score"), "R": d["R"], "t_mm": d["t_mm"],
                        "placed": res["placed"], "slam_interp": res.get("slam"),
                        "anchor_kf": res.get("kf"), "near_loop_closure": res.get("near_loop"),
                        "T_w_obj": mat16(res.get("T_w_obj")), "result_age_s": frame_age, "pose_wait_s": round(waited, 3),
                        "ms": r.get("ms")})
                if l.memory is not None:
                    l.memory.add_frame(stamp, [(d["object"], d.get("score", 0.0), d["R"], d["t_mm"]) for d in dets],
                                       depth=depth_small)
                    if l.consensus is not None:
                        l.consensus.add(stamp, dets)
                placed_any[l.name] = placed
            if self.online is not None:
                with self.depth_lock:
                    depth_full = self.depth_full.pop(stamp, None)
                self.online.add(stamp, dets, r.get("weak", []), depth_full)
            with self.log_lock:
                self.frames_log.write(json.dumps({"t_ns": stamp, "dets": [{k: d.get(k) for k in ("object", "score", "R", "t_mm")}
                                                                          for d in dets]}, ensure_ascii=False) + "\n")
            p0 = placed_any[self.lanes[0].name]
            self.last_result = {"t_ns": stamp, "n": len(dets), "ms": r.get("ms"),
                                "age_s": frame_age, "objects": [p["name"] for p in p0 if p["placed"]],
                                "unplaced": [p["name"] for p in p0 if not p["placed"]],
                                "placed_by_lane": {k: sum(1 for p in v if p["placed"]) for k, v in placed_any.items()}}
            self.sam6d_runs.append(frame_age)

    # ── display alignment between lanes ─────────────────────────────────────
    def update_alignment(self):
        """T_ref_lane for every non-reference lane from the SAM-camera trajectories (same SAM frames)."""
        if self.t0_ns is None or len(self.lanes) < 2:
            return
        ref = self.ref
        t_now = self.replay_t_ns
        for l in self.lanes:
            if l is ref or l.align_frozen:
                continue
            step = 200_000_000
            ts = range(self.t0_ns, t_now + 1, step)
            P, Q, ts_ok = [], [], []
            for t in ts:
                Ta, _ = ref.fusion.T_w_sam(t)
                Tb, _ = l.fusion.T_w_sam(t)
                if Ta is None or Tb is None:
                    continue
                P.append(Tb[:3, 3])
                Q.append(Ta[:3, 3])
                ts_ok.append(t)
            span = (ts_ok[-1] - ts_ok[0]) / 1e9 if len(ts_ok) > 1 else 0.0
            # the fit needs real motion: a stationary start (260910: ~35 s) gives one point and an arbitrary
            # rotation, so gate on the SAM camera's path length and spatial spread, not on time alone
            path_m = float(np.sum(np.linalg.norm(np.diff(np.asarray(Q), axis=0), axis=1))) if len(Q) > 1 else 0.0
            spread_m = float(np.sqrt(np.linalg.eigvalsh(np.cov(np.asarray(Q).T))[-2])) if len(Q) > 3 else 0.0
            if len(P) >= 25 and span >= 4.0 and path_m >= self.a.align_path_m and spread_m >= 0.15:
                T, rmse = kabsch(P, Q)
                l.align = T
                l.align_info = {"pairs": len(P), "rmse_cm": round(rmse * 100, 2), "span_s": round(span, 1),
                                "path_m": round(path_m, 2), "frozen": False, "mode": "kabsch"}
                if span >= self.a.align_span_s and path_m >= 2 * self.a.align_path_m:
                    l.align_frozen = True
                    l.align_info["frozen"] = True
                    log(f"[{l.name}] display alignment to [{ref.name}] frozen: {len(P)} SAM-camera pairs over "
                        f"{span:.1f} s, RMSE {rmse * 100:.2f} cm")
            elif l.align is None or l.align_info.get("mode") == "single":
                Ta, _ = ref.fusion.T_w_sam(t_now)
                Tb, _ = l.fusion.T_w_sam(t_now)
                if Ta is not None and Tb is not None:
                    l.align = Ta @ inv_se3(Tb)
                    l.align_info = {"pairs": 1, "rmse_cm": None, "span_s": round(span, 1), "path_m": round(path_m, 2),
                                    "frozen": False, "mode": "single"}

    def lane_divergence_cm(self, l: Lane, t_ns: int):
        """distance between the SAM camera positions of lane l (aligned) and the reference lane at t"""
        if l is self.ref:
            return None
        Ta, _ = self.ref.fusion.T_w_sam(t_ns)
        Tb, _ = l.fusion.T_w_sam(t_ns)
        if Ta is None or Tb is None:
            return None
        pb = (l.A() @ Tb)[:3, 3]
        return round(float(np.linalg.norm(pb - Ta[:3, 3]) * 100), 1)

    # ── render + broadcast ──────────────────────────────────────────────────
    def lane_frame(self, l: Lane, t_ns, idx, img, send_traj: bool):
        A = l.A()
        T_ws, slam_status, rows = l.fusion.view(t_ns)
        if l.memory is not None:
            rows = self.memory_rows(l, t_ns, T_ws, rows)
        canvas = draw_objects(img.copy(), self.K, self.dist, rows, self.extents, self.a.overlay_max_age,
                              tag=l.label, tag_color=hex_bgr(l.color))
        ok, jpg = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 78])
        T_wslam, _ = l.poses.at(l.fusion.slam_t(t_ns))
        objs = []
        for r in rows:
            T_w_obj = A @ r["T_w_obj"]
            objs.append({"name": r["name"], "source": r["source"], "age_s": round(r["age_s"], 2),
                         "score": round(r["score"], 3), "est_count": r["est_count"],
                         "anchor_kf": r.get("anchor_kf"), "near_loop": r.get("near_loop"),
                         "status": r.get("status"), "instance": r.get("instance"), "multi": r.get("multi", False),
                         "confidence": r.get("confidence"), "view": r.get("view"),
                         "T_w_obj": mat16(T_w_obj), "T_w_obj_lane": mat16(r["T_w_obj"]), "T_cam_obj": mat16(r["T_cam_obj"]),
                         "size_mm": self.extents.get(r["name"], {}).get("size_mm"),
                         "center_mm": (np.add(self.extents[r["name"]]["min_mm"], self.extents[r["name"]]["max_mm"]) / 2).tolist()
                         if r["name"] in self.extents else None,
                         "history": [(A[:3, :3] @ np.asarray(h) + A[:3, 3]).tolist() for h in r["history"][-20:]]})
        l.write_log("display_objects", {"t_ns": t_ns, "frame_idx": idx, "slam": slam_status,
                                        "objects": [{"name": o["name"], "source": o["source"],
                                                     "age_s": o["age_s"], "T_cam_obj": o["T_cam_obj"]} for o in objs]})
        out = {"name": l.name, "label": l.label, "color": l.color, "backend": l.backend,
               "jpeg": base64.b64encode(jpg.tobytes()).decode(),
               "slam_status": slam_status,
               "T_w_sam": mat16(None if T_ws is None else A @ T_ws),
               "T_w_slam": mat16(None if T_wslam is None else A @ T_wslam),
               "slam_frame": ((l.hello or {}).get("params") or {}).get("pose_frame"),
               "objects": objs, "divergence_cm": self.lane_divergence_cm(l, t_ns)}
        if send_traj:
            pts = np.asarray(l.poses.positions(step=3), np.float64)
            out["trajectory"] = [] if pts.size == 0 else ((A[:3, :3] @ pts.T).T + A[:3, 3]).round(4).tolist()
        return out

    def run_render(self):
        period = 1.0 / self.a.view_hz
        last_traj = 0.0
        last_align = 0.0
        while not self.stop.is_set():
            t_loop = time.monotonic()
            if self.frames:
                if time.monotonic() - last_align > 0.5:
                    self.update_alignment()
                    last_align = time.monotonic()
                target = self.replay_t_ns - int(self.a.display_delay * 1e9)
                pick = None
                # a copy: the camera thread appends while this runs ("deque mutated during iteration" killed this thread
                # and froze the viewer image, 2026-10-14)
                frames = list(self.frames)
                for t_ns, idx, img in reversed(frames):
                    if t_ns <= target:
                        pick = (t_ns, idx, img)
                        break
                if pick is None:
                    pick = frames[0]
                t_ns, idx, img = pick
                send_traj = time.monotonic() - last_traj > 1.0
                lanes = [self.lane_frame(l, t_ns, idx, img, send_traj) for l in self.lanes]
                if send_traj:
                    last_traj = time.monotonic()
                l0 = lanes[0]
                payload = {
                    "type": "frame", "t_ns": t_ns, "t_rel_s": round((t_ns - self.t0_ns) / 1e9, 3) if self.t0_ns else 0,
                    "frame_idx": idx,
                    "lanes": lanes,
                    # lane 0 also at the top level (older tools / viewers)
                    "jpeg": l0["jpeg"], "slam_status": l0["slam_status"], "T_w_sam": l0["T_w_sam"],
                    "T_w_slam": l0["T_w_slam"], "objects": l0["objects"],
                    "stats": self.stats(),
                }
                if send_traj:
                    payload["trajectory"] = l0.get("trajectory")
                self.broadcast(payload)
                if self.a.record_events_hz > 0:
                    try:
                        self.record_event(payload)
                    except Exception as e:                       # noqa: BLE001  (recording must never stop the viewer)
                        log(f"record-events: {e}")
            time.sleep(max(0.0, period - (time.monotonic() - t_loop)))

    def run_nav_record(self):
        """--nav-bridge with --record-events-hz: what the navigation pane shows -> <out>/nav.jsonl.gz, for
        pc/replay_run.py. One line 5 times a second: {"t_ns" (same clock as the recorded frames), "state"}; the map and
        the local costmap are added (base64 PNG) only when their version changes."""
        import gzip
        import urllib.request
        base = self.a.nav_bridge.rstrip("/")
        f, map_ver, cm_ver = None, None, None

        def get(path):
            with urllib.request.urlopen(base + path, timeout=1.5) as r:
                return r.read()
        while not self.stop.is_set():
            t_loop = time.monotonic()
            try:
                if self.replay_t_ns:
                    st = json.loads(get("/state"))
                    line = {"t_ns": int(self.replay_t_ns - self.a.display_delay * 1e9), "wall_ns": time.time_ns(), "state": st}
                    if st.get("map_ver") and st["map_ver"] != map_ver:
                        line["meta"] = json.loads(get("/meta"))
                        line["map_png"] = base64.b64encode(get("/map.png")).decode()
                        map_ver = st["map_ver"]
                    cm = st.get("costmap")
                    if cm and cm.get("ver") != cm_ver:
                        line["costmap_png"] = base64.b64encode(get("/costmap.png")).decode()
                        cm_ver = cm["ver"]
                    if f is None:
                        f = gzip.open(self.out / "nav.jsonl.gz", "wt", compresslevel=1)
                    f.write(json.dumps(line, separators=(",", ":")) + "\n")
                    f.flush()
            except Exception:                                # noqa: BLE001  (bridge down / link lost: keep trying)
                pass
            time.sleep(max(0.0, 0.2 - (time.monotonic() - t_loop)))

    def run_memory(self):
        while not self.stop.is_set():
            for l in self.lanes:
                if l.memory is not None:
                    l.memory.update()
            time.sleep(0.2)

    def memory_rows(self, l: Lane, t_ns, T_ws, fusion_rows, exact_window_ns=20_000_000):
        """display rows from the object memory: one per kept instance, in the corrected map."""
        hist = {r["name"]: r["history"] for r in fusion_rows}
        lms = [m for m in l.memory.landmarks() if m["status"] in ("active", "lost", "remembered")
               and m["n_obs"] >= self.a.map_min_obs]
        per_class = {}
        for m in lms:
            per_class[m["name"]] = per_class.get(m["name"], 0) + 1
        rows = []
        cons = {}
        if l.consensus is not None:
            try:
                cons = l.consensus.poses()
            except Exception as e:                                      # noqa: BLE001  (fall back to the memory's own pose)
                log(f"map pose consensus failed: {type(e).__name__}: {e}")
        for m in lms:
            T_w_obj = cons.get(m["object_id"], m["T_w_obj"])
            rows.append({
                "name": m["name"], "instance": m["object_id"], "multi": per_class[m["name"]] > 1,
                "status": m["status"], "score": float(m["score"] or 0.0), "confidence": m["confidence"],
                "T_w_obj": T_w_obj, "age_s": (t_ns - m["last_seen_ns"]) / 1e9, "est_count": m["n_obs"],
                "view": self.best_view(m["name"], np.asarray(T_w_obj)[:3, 3]),
                "anchor_kf": None, "near_loop": False, "history": hist.get(m["name"], []),
                "T_cam_obj": None if T_ws is None else inv_se3(T_ws) @ T_w_obj,
                "source": ("no_slam" if T_ws is None else
                           "sam6d" if abs(t_ns - m["last_seen_ns"]) <= exact_window_ns else "slam_interp"),
            })
        return rows

    def stats(self):
        lags = [v for v in self.sam6d_runs if v is not None]
        lane_stats = [l.stats() for l in self.lanes]
        s = dict(lane_stats[0])          # lane 0 flat at the top level, as before
        s.update({
            "status": self.status,
            "sam_host": self.a.pc_host,
            "clock": (f"live camera, SAM tau {self.sam_tau_ns / 1e6:+.1f} ms" if self.live_sam
                      else f"frame clock, SAM tau {self.sam_tau_ns / 1e6:+.1f} ms" if self.raw_sam
                      else f"header stamps, SAM tau {self.sam_tau_ns / 1e6:+.1f} ms"),
            "sam6d_frames": self.sam6d_frames, "sam6d_fed": self.fed,
            "sam6d_speed": self.a.sam6d_speed,
            "sam6d_last": self.last_result,
            "sam6d_age_median_s": round(float(np.median(lags)), 2) if lags else None,
            "display_delay_s": self.a.display_delay,
            "overlay_max_age_s": self.a.overlay_max_age,
            "replay_t_rel_s": round((self.replay_t_ns - self.t0_ns) / 1e9, 2) if self.t0_ns else 0,
            "duration_s": round((int(self.sam.t_ns[-1]) - self.t0_ns) / 1e9, 1) if self.t0_ns else None,
            "lanes": lane_stats,
            "reference_lane": self.ref.name,
            "world_zup": self.ref.source().startswith("lidar") or self.ref.is_lidar,
        })
        return s

    def init_event(self):
        return {"type": "init", "K": self.K.tolist(), "width": self.sam.W, "height": self.sam.H,
                "X_slam_sam": mat16(self.lanes[0].fusion.X),
                "trajectory": self.lanes[0].poses.positions(step=3),
                "lanes": [{"name": l.name, "label": l.label, "color": l.color, "backend": l.backend,
                           "X_slam_sam": mat16(l.fusion.X), "host": l.host, "link": l.link}
                          for l in self.lanes],
                "reference_lane": self.ref.name, "world_zup": self.ref.is_lidar}

    def record_event(self, payload):
        """--record-events-hz: viewer events (camera image with overlays included) -> <out>/events.jsonl.gz, for
        pc/replay_run.py. Frames that carry the trajectory (one per second) are always kept."""
        now = time.monotonic()
        if "trajectory" not in payload and now - getattr(self, "_rec_last", -1e9) < 1.0 / self.a.record_events_hz:
            return
        self._rec_last = now
        if getattr(self, "_rec_f", None) is None:
            import gzip
            self._rec_f = gzip.open(self.out / "events.jsonl.gz", "wt", compresslevel=1)
            self._rec_f.write(json.dumps(self.init_event(), separators=(",", ":")) + "\n")
        self._rec_f.write(json.dumps(payload, separators=(",", ":")) + "\n")
        self._rec_f.flush()

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

            def nav_proxy(self, body):
                """/nav/* -> the Nav2 bridge on the robot's computer (nav/real/nav_bridge.py), so the page has one origin"""
                import urllib.error
                import urllib.request
                if not hub.a.nav_bridge:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                req = urllib.request.Request(hub.a.nav_bridge.rstrip("/") + self.path[4:], data=body,
                                             headers={"Content-Type": "application/json"} if body is not None else {})
                try:
                    with urllib.request.urlopen(req, timeout=2.0) as r:
                        data, ctype, code = r.read(), r.headers.get("Content-Type", "application/json"), r.status
                except urllib.error.HTTPError as e:
                    data, ctype, code = b"", "text/plain", e.code
                except Exception as e:                           # noqa: BLE001  (bridge not running / link down)
                    data, ctype, code = json.dumps({"ok": False, "error": f"nav bridge: {e}"}).encode(), "application/json", 502
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)

            def do_POST(self):
                if not self.path.startswith("/nav/"):
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                body = self.rfile.read(int(self.headers.get("Content-Length", 0) or 0)) or b"{}"
                if self.path.split("?")[0] in ("/nav/goal", "/nav/jog", "/nav/cancel", "/nav/route"):
                    log(f"viewer -> nav {self.path}: {body[:120].decode(errors='replace')}")
                self.nav_proxy(body)

            def do_GET(self):
                if self.path.startswith("/vendor/"):
                    # three.js is served from here (web/vendor, 0.169.0) instead of a CDN: a browser on a machine
                    # without internet showed an empty map pane (2026-09-17)
                    root = (HERE / "web" / "vendor").resolve()
                    f = (HERE / "web" / self.path.lstrip("/").split("?")[0]).resolve()
                    if root not in f.parents or not f.is_file():
                        self.send_error(HTTPStatus.NOT_FOUND)
                        return
                    body = f.read_bytes()
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "text/javascript; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "max-age=3600")
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path in ("/", "/index.html"):
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
                        init = hub.init_event()
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
                elif self.path.startswith("/nav/"):
                    self.nav_proxy(None)
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
        shown = self.a.pc_host if self.a.http_host in ("0.0.0.0", "") else self.a.http_host
        log(f"viewer: http://{shown}:{self.a.http_port}  (bound to {self.a.http_host}:{self.a.http_port})")
        srv.serve_forever()

    # ── lifecycle ───────────────────────────────────────────────────────────
    def run(self):
        self.fw = FrameWriter()
        threads = [threading.Thread(target=self.http, daemon=True),
                   threading.Thread(target=self.run_results, daemon=True),
                   threading.Thread(target=self.run_render, daemon=True),
                   threading.Thread(target=self.run_memory, daemon=True)]
        if self.a.nav_bridge and self.a.record_events_hz > 0:
            threads.append(threading.Thread(target=self.run_nav_record, daemon=True))
        if self.online is not None:
            threads.append(threading.Thread(target=self.online.run, args=(self.stop,), daemon=True))
        threads += [threading.Thread(target=l.server, daemon=True) for l in self.lanes]
        for t in threads:
            t.start()
        if not self.a.no_sam6d:
            self.start_sam6d()
        if not self.a.no_mac:
            for l in self.lanes:
                l.start()
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
            # a live run never reaches replay_done: it ends on SIGINT (Ctrl+C, launcher stop button), so write the
            # summary here too (2026-09-30: live runs stopped from the launcher had no summary.json)
            if not getattr(self, "_summary_written", False):
                try:
                    self.write_summary()
                    self._summary_written = True
                except Exception as e:                                  # noqa: BLE001
                    log(f"summary on stop failed: {type(e).__name__}: {e}")
            self.shutdown()

    def lane_summary(self, l: Lane):
        s = l.stats()
        s["objects"] = {k: {"est_count": v.est_count, "T_w_obj": mat16(l.fusion.world(v.current)),
                            "t_est_ns": v.current.t_ns, "anchor_kf": v.current.kf}
                        for k, v in l.fusion.objects.items()}
        s["observations_final"] = l.fusion.dump_observations()
        if l.memory is not None:
            l.memory.update()
            s["depth_deletions"] = l.memory.depth_deletions()
            s["depth_samples"] = l.memory.depth_samples()
            s["memory_landmarks"] = [{**{k: v for k, v in m.items() if k != "T_w_obj"}, "T_w_obj": mat16(m["T_w_obj"])}
                                     for m in l.memory.landmarks()]
            s["map_pose"] = self.map_pose
            if l.consensus is not None:                                 # poses the viewer showed (memory_landmarks keeps the memory's own running-mean pose)
                try:
                    s["memory_landmarks_consensus"] = {str(k): mat16(T) for k, T in l.consensus.poses().items()}
                    s["map_pose_update_ms"] = l.consensus.stats()
                except Exception as e:                                  # noqa: BLE001
                    log(f"map pose consensus summary failed: {type(e).__name__}: {e}")
        s["slam_end"] = l.end
        s["T_ref_lane"] = mat16(l.align) if l is not self.ref else None
        s["align_final"] = l.align_info
        return s

    def write_summary(self):
        s = self.stats()
        l0 = self.lane_summary(self.lanes[0])
        for k in ("objects", "observations_final", "memory_landmarks", "slam_end"):
            if k in l0:
                s[k] = l0[k]
        s["t0_ns"] = self.t0_ns
        s["pose_wait"] = dict(self.pose_wait_stats)
        s["lanes"] = [self.lane_summary(l) for l in self.lanes]
        (self.out / "summary.json").write_text(json.dumps(s, indent=1, default=str))
        if self.online is not None:
            try:
                R = self.online.final()
                (self.out / "online_designs.json").write_text(json.dumps(R, ensure_ascii=False))
                (self.out / "online_timeline.jsonl").write_text("".join(json.dumps(x) + "\n" for x in self.online.timeline))
                log(f"online map designs: {R['online']['passes']} passes, pass ms {R['online']['pass_ms']}, lag ms {R['online']['lag_ms']}")
            except Exception as e:                                      # noqa: BLE001
                log(f"online map designs failed: {type(e).__name__}: {e}")
        for l in self.lanes:
            for f in l.logs.values():
                f.flush()
        self.frames_log.flush()
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
        for l in self.lanes:
            for f in l.logs.values():
                f.close()
        self.frames_log.close()
        self.fw.close()
        log("stopped")


def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ds = "/home/jucpark/DeepLearning/Dataset/260826_etri_eightcircle_dark"
    ap.add_argument("--sam-session", default=f"{ds}/SAM")
    ap.add_argument("--slam-session", default=f"{ds}/SLAM", help="PC copy of the SLAM session (for its frame clock)")
    ap.add_argument("--extrinsic", default=str(REPO / "objpose" / "rt" / "X_slam_sam.json"))
    ap.add_argument("--out", default=str(REPO / "objpose" / "output" / time.strftime("run_%y%m%d_%H%M%S")))
    ap.add_argument("--mac-host", default="mac", help="ssh destination of the SLAM host (alias or user@ip)")
    ap.add_argument("--link", choices=["tunnel", "direct", "manual", "local"], default="tunnel",
                    help="tunnel: ssh -R (Mac connects to its own localhost); direct: ssh launches the streamer with "
                         "--connect <pc-host>:port; manual: print the command and wait; local: run the streamer on this PC")
    ap.add_argument("--slam-host", default="192.168.20.55", help="SLAM host address (shown in the viewer)")
    ap.add_argument("--pc-host", default="192.168.20.5", help="this PC's LAN address the Mac connects to (direct/manual)")
    ap.add_argument("--slam-bind", default=None, help="address the SLAM pose server binds (default 127.0.0.1 for tunnel, "
                                                     "0.0.0.0 for direct/manual)")
    ap.add_argument("--sam-tau-s", type=float, default=None,
                    help="converted SAM bag only: SLAM time = SAM header stamp + tau (default: extrinsic json sam_tau_s or 0)")
    ap.add_argument("--mac-runner", default="~/objpose/run_slam.sh")
    ap.add_argument("--lidar-runner", default="~/objpose/lidar/venv/bin/python -u ~/objpose/lidar/lidar_stream.py",
                    help="KISS-ICP streamer command on the SLAM host (laptop: ~/objpose/lidar/run_lidar_stream.sh)")
    ap.add_argument("--mac-slam-session", default="~/Documents/DefenseMeta/Dataset/260826_etri_eightcircle_dark/SLAM")
    ap.add_argument("--features", type=int, default=2000)
    ap.add_argument("--slam", choices=list(BACKENDS), default="orbslam3", help="SLAM backend of lane 0")
    ap.add_argument("--slam-param", action="append", default=[], metavar="Key=Value",
                    help="ORB-SLAM3 settings override / lidar_stream option, e.g. ORBextractor.nFeatures=1500")
    ap.add_argument("--slam-port", type=int, default=17001)
    ap.add_argument("--lane", action="append", default=[], metavar="k=v,k=v",
                    help="additional SLAM lane (keys: name slam port ssh link host session extrinsic tau features params runner color)")
    ap.add_argument("--align-span-s", type=float, default=15.0, help="overlap [s] after which a lane's display alignment is frozen")
    ap.add_argument("--align-path-m", type=float, default=1.0, help="SAM-camera path length [m] needed before the Kabsch fit "
                                                                    "(frozen at 2x); before that the current pose aligns the lanes")
    ap.add_argument("--http-host", default="0.0.0.0")
    ap.add_argument("--http-port", type=int, default=8765)
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--start-s", type=float, default=0.5, help="start this far into the overlap")
    ap.add_argument("--duration-s", type=float, default=0.0, help="0 = to the end")
    ap.add_argument("--sam-feed-hz", type=float, default=10.0)
    ap.add_argument("--nav-bridge", default="",
                    help="http://<robot computer>:8770 of nav/real/nav_bridge.py: the viewer then shows the Nav2 map and sets goals")
    ap.add_argument("--slam-name", default="", help="short lane name shown in the viewer (default: by backend)")
    ap.add_argument("--slam-label", default="", help="lane title shown in the viewer (default: by backend)")
    ap.add_argument("--view-hz", type=float, default=15.0)
    ap.add_argument("--record-events-hz", type=float, default=0.0,
                    help="save the viewer events (image included) at this rate to <out>/events.jsonl.gz for pc/replay_run.py (0 = off)")
    ap.add_argument("--display-delay", type=float, default=0.3)
    ap.add_argument("--overlay-max-age", type=float, default=60.0,
                    help="draw an object on the video only this long after its last SAM-6D estimate")
    ap.add_argument("--max-gap", type=float, default=0.25, help="max SLAM pose gap to interpolate across [s]")
    ap.add_argument("--tail-s", type=float, default=8.0)
    ap.add_argument("--no-mac", action="store_true", help="don't launch the SLAM hosts; wait for external SLAM clients")
    ap.add_argument("--no-sam6d", action="store_true")
    ap.add_argument("--ism-objects", nargs="*", default=None,
                    help="restrict the recogniser to these object names (BOP: the scene's target list)")
    ap.add_argument("--ism-config", default=None,
                    help="SAM-6D object set, relative to sam6d_realtime (default: the config in "
                         "realtime/run_split_example.yaml; BOP YCB-V: configs/yolo_ism_ycbv.yaml)")
    ap.add_argument("--sam6d-speed", choices=list(SAM6D_SPEED_PRESETS), default="fast",
                    help="SAM-6D opt-in speed options written into <run>/sam6d_config.yaml: "
                         "off = fp32 / no mask cache / no PEM budget (pre-2026-09-14 behaviour); "
                         "fast = fine_npoint 1024 + mask cache (the part measured to keep accuracy); "
                         "report = the literal _speed_opt_2026_09_13/REPORT.md §9 preset "
                         "(adds bf16 + PEM budget 1 — both measured to break detection accuracy here)")
    ap.add_argument("--no-memory", action="store_true", help="show raw latest estimates instead of the object memory")
    ap.add_argument("--map-min-obs", type=int, default=1,
                    help="show a map object only after this many observations (drops one-or-two-frame false objects)")
    ap.add_argument("--pose-wait-s", type=float, default=2.0,
                    help="hold a SAM-6D result up to this long for the SLAM pose of its frame before placing it")
    ap.add_argument("--no-depth-check", action="store_true",
                    help="disable the object memory's depth check (delete landmarks whose spot the depth image sees through)")
    ap.add_argument("--depth-free-margin-m", type=float, default=0.15, help="depth check: measured this much farther = free")
    ap.add_argument("--depth-confirm-frames", type=int, default=3, help="depth check: consecutive free frames before deleting")
    ap.add_argument("--depth-max-speed", type=float, default=0.6, help="depth check: skip frames above this camera speed [m/s]")
    ap.add_argument("--depth-max-rot", type=float, default=45.0, help="depth check: skip frames above this rotation rate [deg/s]")
    ap.add_argument("--assoc-gate-m", type=float, default=0.30,
                    help="object memory association distance gate (0.15 -> 0.30 on 2026-09-14: loop-closure deformation of a keyframe reached 0.29 m and split objects; docs/loop_split_analysis_260914.md F2)")
    ap.add_argument("--exit-when-done", action="store_true")
    Hub(ap.parse_args()).run()


if __name__ == "__main__":
    main()
