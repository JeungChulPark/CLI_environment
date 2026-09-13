#!/usr/bin/env python3
"""lidar_stream.py — LiDAR SLAM (KISS-ICP, VLP-16) streamer for the object-pose hub. Runs on the Mac.

Same live protocol as slam_stream / rtab_stream (newline JSON over TCP, the hub owns the socket
through the ssh -R tunnel):
    -> {"type":"hello","source":"lidar_kiss_icp", first_ns, last_ns, n_frames, ...}
    <- {"type":"start","t0_ns":..,"rate":1.0}
    -> {"type":"pose","t_ns":..,"state":"OK","T_wc":[16],"track_ms":..,"frame_idx":..,"dropped":..,
        "map_id":0,"map_changed":false,"ref_kf":null}
    -> {"type":"end","processed":..,"dropped":..}

T_wc is the LiDAR (velodyne frame) pose in the odometry world = the first processed scan. KISS-ICP is
odometry without loop closure, so there are no keyframes; the hub anchors objects in world
coordinates for this backend. Scans are replayed from the rosbag2 sqlite file at bag rate.

Jitter fixes (measured offline in lidar/eval_lidar_variants.py, README "라이다 궤적 흔들림"):
  --deskew-passes 2  KISS-ICP deskews a scan with the previous scan's motion, which is wrong at start / stop /
                     turn changes (1.5-2.7 cm spikes); the scan is deskewed again with its own estimated motion
                     and re-registered.
  --smooth-scans 5   centred moving average over 5 scans. The raw pose is still sent at once (display and
                     interpolation never wait), and {"type":"pose_refine","t_ns","T_wc","smooth_n"} replaces it once
                     the 2 following scans are registered (+0.2 s). A smoothed pose alone arrives too late for the
                     hub's display: interpolating a frame needs the *next* scan too (+0.1 s), 0.32 s > 0.3 s.
"""
from __future__ import annotations

import argparse
import json
import socket
import sqlite3
import struct
import sys
import time
from pathlib import Path

from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

DT = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def header_stamp(blob):
    sec, nsec = struct.unpack_from("<iI", blob, 4)
    return sec * 1_000_000_000 + nsec


def parse_pc2(blob):
    o = 12
    fl = struct.unpack_from("<I", blob, o)[0]; o += 4 + fl; o = (o + 3) & ~3
    o += 8
    nf = struct.unpack_from("<I", blob, o)[0]; o += 4
    names, offs, fmts = [], [], []
    for _ in range(nf):
        o = (o + 3) & ~3
        sl = struct.unpack_from("<I", blob, o)[0]; o += 4
        names.append(blob[o:o + sl - 1].decode()); o += sl; o = (o + 3) & ~3
        offs.append(struct.unpack_from("<I", blob, o)[0]); o += 4
        fmts.append("<" + DT[blob[o]]); o += 1; o = (o + 3) & ~3
        o += 4
    o += 1; o = (o + 3) & ~3
    step = struct.unpack_from("<I", blob, o)[0]; o += 8
    dl = struct.unpack_from("<I", blob, o)[0]; o += 4
    dtype = np.dtype({"names": names, "formats": fmts, "offsets": offs, "itemsize": step})
    return np.frombuffer(blob[o:o + dl], dtype)


def register(odom, frame, timestamps, passes):
    """KissICP.register_frame; passes > 1 re-deskews the scan with the motion just estimated for it"""
    if passes <= 1:
        odom.register_frame(frame, timestamps)
        return
    guess = odom.last_pose @ odom.last_delta
    delta, pose = odom.last_delta, guess
    sigma = odom.adaptive_threshold.get_threshold()
    for _ in range(passes):
        f = odom.preprocessor.preprocess(frame, timestamps, delta)
        source, frame_down = odom.voxelize(f)
        pose = odom.registration.align_points_to_map(points=source, voxel_map=odom.local_map, initial_guess=pose,
                                                     max_correspondance_distance=3 * sigma, kernel=sigma)
        delta = np.linalg.inv(odom.last_pose) @ pose
    odom.adaptive_threshold.update_model_deviation(np.linalg.inv(guess) @ pose)
    odom.local_map.update(frame_down, pose)
    odom.last_delta = delta
    odom.last_pose = pose


def window_mean(Ts):
    """centred box average of poses: mean position, sign-aligned normalised quaternion mean"""
    T = np.eye(4)
    T[:3, 3] = np.mean([M[:3, 3] for M in Ts], axis=0)
    q = Rotation.from_matrix([M[:3, :3] for M in Ts]).as_quat()
    q[(q @ q[len(q) // 2]) < 0] *= -1
    m = q.mean(0)
    T[:3, :3] = Rotation.from_quat(m / np.linalg.norm(m)).as_matrix()
    return T


class Line:
    def __init__(self, host, port):
        self.s = socket.create_connection((host, port))
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.buf = b""

    def send(self, obj):
        self.s.sendall((json.dumps(obj, separators=(",", ":")) + "\n").encode())

    def recv(self):
        while b"\n" not in self.buf:
            chunk = self.s.recv(65536)
            if not chunk:
                raise ConnectionError("hub closed the connection")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True, help="dataset lidar/ folder with the velodyne *.db3")
    ap.add_argument("--mode", default="live", choices=["live"])
    ap.add_argument("--connect", required=True)
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--offset-ns", type=int, default=0)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--min-range", type=float, default=0.5)
    ap.add_argument("--max-range", type=float, default=30.0)
    ap.add_argument("--voxel", type=float, default=0.10)
    ap.add_argument("--topic", default="/velodyne_points")
    ap.add_argument("--spin-ms", type=float, default=50.0,
                    help="busy-wait this long before each scan so macOS keeps the core fast (0 = plain sleep)")
    ap.add_argument("--deskew-passes", type=int, default=2, help="1 = stock KISS-ICP constant-velocity deskew")
    ap.add_argument("--smooth-scans", type=int, default=5, help="odd centred window; 1 = send poses unsmoothed")
    a, _unknown = ap.parse_known_args()      # accept and ignore slam_stream-only flags (--features ...)

    from kiss_icp.config import load_config
    from kiss_icp.kiss_icp import KissICP

    db = next(Path(a.session).expanduser().glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    tid = con.execute("select id from topics where name=?", (a.topic,)).fetchone()[0]
    rows = con.execute("select id, substr(data, 1, 16) from messages where topic_id=?", (tid,)).fetchall()
    frames = sorted((header_stamp(h) + a.offset_ns, mid) for mid, h in rows)
    print(f"[lidar_stream] {db.name} | {len(frames)} scans | topic {a.topic}", flush=True)

    cfg = load_config(None)
    cfg.data.min_range, cfg.data.max_range, cfg.data.deskew = a.min_range, a.max_range, True
    cfg.mapping.voxel_size = a.voxel
    odom = KissICP(cfg)

    half = max(0, a.smooth_scans) // 2
    host, port = a.connect.rsplit(":", 1)
    line = Line(host, int(port))
    line.send({"type": "hello", "source": "lidar_kiss_icp", "features": None, "session": str(db),
               "first_ns": frames[0][0], "last_ns": frames[-1][0], "n_frames": len(frames),
               "time_source": "header", "time_domain": "velodyne header stamp", "kf_updates": False,
               "params": {"min_range": a.min_range, "max_range": a.max_range, "voxel": a.voxel, "deskew": True,
                              "deskew_passes": a.deskew_passes, "smooth_scans": a.smooth_scans,
                              "smooth_delay_s": round(half * 0.1, 2)}})
    msg = line.recv()
    while msg.get("type") != "start":
        msg = line.recv()
    t0, rate = int(msg["t0_ns"]), float(msg.get("rate", 1.0))
    wall0 = time.monotonic()
    print(f"[lidar_stream] start t0_ns={t0} rate={rate}", flush=True)

    spin_s = max(0.0, a.spin_ms) / 1000.0
    idx = next((i for i, f in enumerate(frames) if f[0] >= t0), len(frames))
    processed = dropped = 0
    times = []

    def lag_s(t_ns):
        return round((time.monotonic() - wall0) - (t_ns - t0) / 1e9 / rate, 4)

    def send_refine(item, Ts):
        t_ns = item[0]
        line.send({"type": "pose_refine", "t_ns": int(t_ns), "T_wc": [round(float(v), 6) for v in window_mean(Ts).reshape(-1)],
                   "smooth_n": len(Ts), "send_lag_s": lag_s(t_ns)})

    hist = deque(maxlen=2 * half + 1)      # last registered scans: (t_ns, T_raw)
    while idx < len(frames):
        if a.max_frames and processed >= a.max_frames:
            break
        due = wall0 + (frames[idx][0] - t0) / 1e9 / rate
        now = time.monotonic()
        if due - spin_s > now:
            time.sleep(min(due - spin_s - now, 0.02))
            continue
        # macOS drops a thread that sleeps ~100 ms between scans to a slow core / low clock, and
        # everything (sqlite, numpy, KISS-ICP) then runs 5x slower; it takes > 20 ms to ramp back.
        # Busy-waiting the last spin_s before a scan keeps the core at full speed (3.8 vs 17.5 ms).
        while time.monotonic() < due:
            pass
        now = time.monotonic()
        # behind schedule: skip to the newest scan that is already due
        j = idx
        while j + 1 < len(frames) and wall0 + (frames[j + 1][0] - t0) / 1e9 / rate <= now:
            j += 1
        dropped += j - idx
        idx = j
        t_ns, mid = frames[idx]
        tp = time.perf_counter()
        pts = parse_pc2(con.execute("select data from messages where id=?", (mid,)).fetchone()[0])
        xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
        r = np.linalg.norm(xyz, axis=1)
        ok = np.isfinite(r) & (r > 0.1)
        ts = pts["time"][ok].astype(np.float64)
        ts = (ts - ts.min()) / max(ts.max() - ts.min(), 1e-9)
        register(odom, xyz[ok], ts, a.deskew_passes)
        ms = (time.perf_counter() - tp) * 1e3
        times.append(ms)
        T = odom.last_pose.copy()
        line.send({"type": "pose", "t_ns": int(t_ns), "state": "OK", "T_wc": [round(float(v), 6) for v in T.reshape(-1)],
                   "track_ms": round(ms, 2), "frame_idx": idx, "dropped": dropped, "map_id": 0, "send_lag_s": lag_s(t_ns),
                   "map_changed": False, "ref_kf": None, "T_w_kf": None, "kf_map_id": None})
        if half:
            hist.append((t_ns, T))
            # refine the scan `half` behind the newest one, averaged over its (possibly start-truncated) window
            if len(hist) > half:
                k = len(hist) - 1 - half
                send_refine(hist[k], [h[1] for h in list(hist)[max(0, k - half):k + half + 1]])
        processed += 1
        idx += 1
    # flush the last `half` scans with the window truncated at the end
    tail = list(hist)
    for k in range(max(0, len(tail) - half), len(tail)):
        send_refine(tail[k], [h[1] for h in tail[max(0, k - half):]])
    line.send({"type": "end", "processed": processed, "dropped": dropped})
    t = np.array(times) if times else np.zeros(1)
    print(f"[lidar_stream] live done: processed {processed} dropped {dropped} | register mean {t.mean():.2f} "
          f"p99 {np.percentile(t, 99):.2f} ms", flush=True)


if __name__ == "__main__":
    sys.exit(main())
