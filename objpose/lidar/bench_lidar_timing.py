#!/usr/bin/env python3
"""bench_lidar_timing.py — why is KISS-ICP slower inside the live streamer than offline?

Runs the same scans under different loop disciplines and splits the per-scan time into
read (sqlite), parse (CDR -> numpy) and register (KISS-ICP):
  continuous : back-to-back, like the offline feasibility run
  paced_sleep: 10 Hz with time.sleep between scans, like lidar_stream.py
  paced_spin : 10 Hz with a busy-wait instead of sleeping
  paced_qos  : 10 Hz sleep, but the thread asks macOS for QOS_CLASS_USER_INTERACTIVE
"""
from __future__ import annotations

import argparse
import ctypes
import json
import sqlite3
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lidar_stream import header_stamp, parse_pc2  # noqa: E402


def set_qos_user_interactive():
    try:
        libc = ctypes.CDLL("/usr/lib/libSystem.dylib")
        QOS_CLASS_USER_INTERACTIVE = 0x21
        return libc.pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0) == 0
    except OSError:
        return False


def run(mode, frames, con, n, period):
    from kiss_icp.config import load_config
    from kiss_icp.kiss_icp import KissICP
    cfg = load_config(None)
    cfg.data.min_range, cfg.data.max_range, cfg.data.deskew = 0.5, 30.0, True
    cfg.mapping.voxel_size = 0.1
    odom = KissICP(cfg)
    if mode == "paced_qos":
        print("  qos set:", set_qos_user_interactive(), flush=True)
    rd, ps, rg = [], [], []
    t_start = time.monotonic()
    for i, (_, mid) in enumerate(frames[:n]):
        if mode != "continuous":
            due = t_start + i * period
            if mode == "paced_spin":
                while time.monotonic() < due:
                    pass
            elif mode.startswith("paced_hybrid"):
                spin_s = float(mode.split("_")[-1]) / 1000.0      # e.g. paced_hybrid_20 = spin the last 20 ms
                while True:
                    now = time.monotonic()
                    if now >= due - spin_s:
                        break
                    time.sleep(min(due - spin_s - now, 0.02))
                while time.monotonic() < due:
                    pass
            else:
                while True:
                    now = time.monotonic()
                    if now >= due:
                        break
                    time.sleep(min(due - now, 0.02))
        t0 = time.perf_counter()
        blob = con.execute("select data from messages where id=?", (mid,)).fetchone()[0]
        t1 = time.perf_counter()
        pts = parse_pc2(blob)
        xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
        r = np.linalg.norm(xyz, axis=1)
        ok = np.isfinite(r) & (r > 0.1)
        ts = pts["time"][ok].astype(np.float64)
        ts = (ts - ts.min()) / max(ts.max() - ts.min(), 1e-9)
        t2 = time.perf_counter()
        odom.register_frame(xyz[ok], ts)
        t3 = time.perf_counter()
        rd.append((t1 - t0) * 1e3); ps.append((t2 - t1) * 1e3); rg.append((t3 - t2) * 1e3)
    skip = 20        # ignore warm-up while the local map is being built
    f = lambda a: {"mean": round(float(np.mean(a[skip:])), 2), "p50": round(float(np.median(a[skip:])), 2),
                   "p99": round(float(np.percentile(a[skip:], 99)), 2)}
    return {"read_ms": f(rd), "parse_ms": f(ps), "register_ms": f(rg)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--n", type=int, default=300)
    ap.add_argument("--modes", default="continuous,paced_sleep,paced_spin,paced_qos")
    a = ap.parse_args()
    db = next(Path(a.session).expanduser().glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    tid = con.execute("select id from topics where name='/velodyne_points'").fetchone()[0]
    rows = con.execute("select id, substr(data, 1, 16) from messages where topic_id=?", (tid,)).fetchall()
    frames = sorted((header_stamp(h), mid) for mid, h in rows)
    out = {}
    for mode in a.modes.split(","):
        print(f"[{mode}] running {a.n} scans ...", flush=True)
        out[mode] = run(mode, frames, con, a.n, 0.1009)
        print(f"[{mode}] {json.dumps(out[mode])}", flush=True)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
