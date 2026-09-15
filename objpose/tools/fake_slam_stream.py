#!/usr/bin/env python3
"""fake_slam_stream.py — Mac 없이 PC 쪽(허브·SAM-6D·뷰어)을 검증하기 위한 SLAM 스트림 시뮬레이터.

Mac 의 slam_stream 이 하는 것과 같은 규약(hello → start 대기 → pose 실시간 전송 → end)으로
미리 저장된 ORB-SLAM3 TUM 궤적(T_wc, 스탬프는 SLAM 호스트 프레임 시계)을 재생한다.
표준 라이브러리만 쓴다 — 어느 파이썬에서도 뜬다.

    python3 objpose/tools/fake_slam_stream.py --connect 192.168.20.5:17001
    python3 objpose/tools/fake_slam_stream.py --connect 192.168.20.5:17001 --tum <TUM 파일> --rate 1.0

기본 궤적: objpose/rt/sim/slam_orb_frameclock_tum.txt (ORB-SLAM3 RGB-D, 260826_etri_eightcircle_dark SLAM 세션,
스탬프 = 컬러 Global Time = 변환본 bag 스탬프). 키프레임 정보는 보내지 않으므로 허브는 월드 고정으로 동작한다.
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent


def quat_to_R(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def load_tum(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        p = line.split()
        if len(p) != 8 or p[0].startswith("#"):
            continue
        t_ns = int(round(float(p[0]) * 1e9))
        tx, ty, tz, qx, qy, qz, qw = map(float, p[1:])
        R = quat_to_R(qx, qy, qz, qw)
        T = [R[0][0], R[0][1], R[0][2], tx, R[1][0], R[1][1], R[1][2], ty, R[2][0], R[2][1], R[2][2], tz, 0, 0, 0, 1]
        rows.append((t_ns, T))
    rows.sort()
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="192.168.20.5:17001", help="hub SLAM pose server host:port")
    ap.add_argument("--tum", default=str(HERE.parent / "rt" / "sim" / "slam_orb_frameclock_tum.txt"))
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--features", type=int, default=2000)
    ap.add_argument("--track-ms", type=float, default=11.0, help="reported per-frame tracking time")
    ap.add_argument("--session", default="fake:" + "slam_orb_frameclock_tum.txt")
    a = ap.parse_args()

    poses = load_tum(a.tum)
    if not poses:
        sys.exit(f"no poses in {a.tum}")
    host, port = a.connect.rsplit(":", 1)
    print(f"[fake_slam] {len(poses)} poses from {a.tum}; connecting to {host}:{port}", flush=True)
    sock = None
    for _ in range(600):
        try:
            sock = socket.create_connection((host, int(port)), timeout=5)
            break
        except OSError:
            time.sleep(1.0)
    if sock is None:
        sys.exit("[fake_slam] hub not reachable")
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.settimeout(None)      # the hub sends "start" only after SAM-6D has loaded (minutes): block indefinitely
    f = sock.makefile("rwb", buffering=0)

    def send(obj):
        f.write((json.dumps(obj, separators=(",", ":")) + "\n").encode())

    send({"type": "hello", "source": "orbslam3", "features": a.features, "session": a.session,
          "first_ns": poses[0][0], "last_ns": poses[-1][0], "n_frames": len(poses),
          "time_source": "frame", "time_domain": "Global Time", "kf_updates": False, "simulated": True})
    print("[fake_slam] hello sent, waiting for start", flush=True)
    start = None
    while start is None:
        line = f.readline()
        if not line:
            sys.exit("[fake_slam] hub closed before start")
        m = json.loads(line)
        if m.get("type") == "start":
            start = m
    t0 = int(start.get("t0_ns", poses[0][0]))
    rate = float(start.get("rate", a.rate))
    print(f"[fake_slam] start t0_ns={t0} rate={rate}", flush=True)
    wall0 = time.monotonic()
    sent = 0
    try:
        for i, (t_ns, T) in enumerate(poses):
            if t_ns < t0:
                continue
            due = wall0 + (t_ns - t0) / 1e9 / rate
            now = time.monotonic()
            if due > now:
                time.sleep(due - now)
            send({"type": "pose", "t_ns": t_ns, "state": "OK", "track_ms": a.track_ms, "prep_ms": 1.0,
                  "dropped": 0, "map_changed": False, "map_id": 0, "T_wc": T,
                  "ref_kf": None, "kf_map_id": None, "T_w_kf": None})
            sent += 1
        send({"type": "end", "processed": sent, "dropped": 0})
    except (BrokenPipeError, ConnectionResetError):
        print("[fake_slam] hub closed the connection", flush=True)
    print(f"[fake_slam] done, {sent} poses sent", flush=True)


if __name__ == "__main__":
    main()
