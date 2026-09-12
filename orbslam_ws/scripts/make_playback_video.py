#!/usr/bin/env python3
"""
Render the exact colour frames ORB-SLAM3 was fed into one H.264 file, so a web
page can play the sequence back at its true 30 Hz next to the estimated poses.

The pairing logic here is deliberately identical to rs_bag_bridge.py: colour and
depth are matched two-sided (nearest depth within tolerance, with one frame of
lookahead), and only paired frames are emitted. That makes video frame i the same
frame as trajectory row i, so the page can index one from the other with no
timestamp search at runtime.

Output is 30 fps regardless of the replay rate used during the measurement run —
the recording itself is 30 Hz, and that is what "real time" means for playback.
"""

import argparse
import subprocess
import sys
from collections import deque
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

COLOR = "/device_0/sensor_1/Color_0/image/data"
DEPTH = "/device_0/sensor_0/Depth_0/image/data"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--width", type=int, default=384)
    ap.add_argument("--height", type=int, default=288)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--crf", type=int, default=30)
    ap.add_argument("--max-skew-ms", type=float, default=20.0)
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    import cv2

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=[COLOR, DEPTH]))

    cmd = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "bgr24",
        "-s", f"{args.width}x{args.height}", "-r", str(args.fps),
        "-i", "-",
        "-c:v", "libx264", "-preset", "slow", "-crf", str(args.crf),
        "-pix_fmt", "yuv420p",
        # faststart so the browser can begin playing before the whole file lands
        "-movflags", "+faststart",
        args.out,
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    dbuf = deque(maxlen=8)
    cbuf = deque()
    skew_ns = int(args.max_skew_ms * 1e6)
    n = drop = 0

    while r.has_next():
        topic, data, ts = r.read_next()
        msg = deserialize_message(data, Image)
        if topic == DEPTH:
            dbuf.append(ts)
        else:
            cbuf.append((ts, msg))
        if not dbuf or not cbuf or dbuf[-1] <= cbuf[0][0]:
            continue
        c_ts, msg = cbuf.popleft()
        d_ts = min(dbuf, key=lambda t: abs(t - c_ts))
        if abs(c_ts - d_ts) > skew_ns:
            drop += 1
            continue
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        small = cv2.resize(img, (args.width, args.height), interpolation=cv2.INTER_AREA)
        proc.stdin.write(small.tobytes())
        n += 1
        if n % 1000 == 0:
            print(f"  {n} frames", flush=True)
        if args.limit and n >= args.limit:
            break

    proc.stdin.close()
    proc.wait()
    size = Path(args.out).stat().st_size
    print(f"wrote {args.out}: {n} frames ({drop} dropped), "
          f"{size/1e6:.2f} MB, {n/args.fps:.1f} s at {args.fps} fps")
    return 0


if __name__ == "__main__":
    sys.exit(main())
