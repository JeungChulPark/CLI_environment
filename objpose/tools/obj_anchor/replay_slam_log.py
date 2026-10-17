#!/usr/bin/env python3
"""replay_slam_log.py — resend a recorded live run's SLAM stream to a hub in real time, with the same protocol
as the SLAM streamers (hello -> wait for start -> pose / kf_update messages on the data clock -> end).

The messages are the ones the hub logged (slam_poses.jsonl, kf_updates.jsonl): live T_wc with reference
keyframes, periodic keyframe refreshes and loop-closure updates, exactly as ORB-SLAM3 produced them. Used to test
the PC side (SAM-6D + object memory + object anchor) live when no SLAM host is reachable.

    python3 objpose/tools/obj_anchor/replay_slam_log.py --run <recorded run dir> --connect 127.0.0.1:17001
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
from pathlib import Path

DROP = ("recv_lag_s", "t_ns_streamer", "frame_idx", "T_wc_raw", "refine_lag_s")


def jl(p):
    return [json.loads(l) for l in Path(p).read_text().splitlines() if l.strip()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--connect", default="127.0.0.1:17001")
    a = ap.parse_args()
    run = Path(a.run)
    poses = jl(run / "slam_poses.jsonl")
    kfu = jl(run / "kf_updates.jsonl")
    msgs = [(int(m["t_ns"]), 0, {k: v for k, v in m.items() if k not in DROP}) for m in poses]
    msgs += [(int(u["t_ns"]), 1, u) for u in kfu if u.get("reason") != "end"]
    msgs.sort(key=lambda e: (e[0], e[1]))
    first, last = msgs[0][0], msgs[-1][0]
    host, port = a.connect.rsplit(":", 1)
    sock = None
    parent = os.getppid()
    for _ in range(900):
        if os.getppid() != parent:                   # the hub that started us is gone
            sys.exit("[replay_slam] hub exited before connecting")
        try:
            sock = socket.create_connection((host, int(port)), timeout=5)
            break
        except OSError:
            time.sleep(1.0)
    if sock is None:
        sys.exit("[replay_slam] hub not reachable")
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.settimeout(None)
    f = sock.makefile("rwb", buffering=0)

    def send(obj):
        f.write((json.dumps(obj, separators=(",", ":")) + "\n").encode())

    send({"type": "hello", "source": "orbslam3", "features": 2000, "session": f"replay:{run.name}",
          "first_ns": first, "last_ns": last, "n_frames": len(poses), "time_source": "frame",
          "time_domain": "Global Time", "kf_updates": True, "simulated": True})
    print(f"[replay_slam] {len(poses)} poses + {len(kfu)} kf updates from {run.name}; waiting for start", flush=True)
    start = None
    while start is None:
        line = f.readline()
        if not line:
            sys.exit("[replay_slam] hub closed before start")
        m = json.loads(line)
        if m.get("type") == "start":
            start = m
    t0 = int(start.get("t0_ns", first))
    rate = float(start.get("rate", 1.0))
    print(f"[replay_slam] start t0_ns={t0} rate={rate}", flush=True)
    wall0 = time.monotonic()
    sent = 0
    try:
        for t_ns, _, m in msgs:
            if t_ns < t0:
                if m.get("type") == "kf_update":        # keep the map state up to t0
                    send(m)
                continue
            due = wall0 + (t_ns - t0) / 1e9 / rate
            now = time.monotonic()
            if due > now:
                time.sleep(due - now)
            send(m)
            sent += 1
        send({"type": "kf_update", **{k: v for k, v in kfu[-1].items() if k != "type"}})
        send({"type": "end", "processed": sent, "dropped": 0})
    except (BrokenPipeError, ConnectionResetError):
        print("[replay_slam] hub closed the connection", flush=True)
    print(f"[replay_slam] done, {sent} messages sent", flush=True)


if __name__ == "__main__":
    main()
