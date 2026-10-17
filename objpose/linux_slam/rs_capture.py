#!/usr/bin/env python3
"""rs_capture.py — live D455f frames for `slam_stream --session live` (Linux SLAM host without root, 2026-09-29).

The Samsung laptop has no udev rules and no sudo, so the conda-forge librealsense (libusb/RSUSB backend) cannot open
the camera (RS2_USB_STATUS_ACCESS). The PyPI pyrealsense2 wheel uses the V4L2 backend, which works through the seat
user's /dev/video* ACL. This script therefore owns the camera and pipes frames to the C++ ORB-SLAM3 runner:

  stdout, line 1 : JSON {"W","H","fx","fy","ppx","ppy","coeffs":[5],"model","serial","depth_scale":0.001}
  stdout, frames : int64 t_ns (little endian) | W*H*3 BGR8 | W*H*2 uint16 depth aligned to colour, millimetres

t_ns = colour frame timestamp in RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME (host epoch clock), the same clock the hub's
clock_offset.py measures against. Runs until the reader closes the pipe.
"""
import argparse
import json
import struct
import sys
import time

import numpy as np
import pyrealsense2 as rs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", default="")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    a = ap.parse_args()

    cfg = rs.config()
    if a.serial:
        cfg.enable_device(a.serial)
    cfg.enable_stream(rs.stream.color, a.width, a.height, rs.format.bgr8, a.fps)
    cfg.enable_stream(rs.stream.depth, a.width, a.height, rs.format.z16, a.fps)
    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    dev = prof.get_device()
    scale = dev.first_depth_sensor().get_depth_scale()
    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    align = rs.align(rs.stream.color)
    out = sys.stdout.buffer
    hdr = {"W": intr.width, "H": intr.height, "fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy,
           "coeffs": list(intr.coeffs), "model": str(intr.model), "serial": dev.get_info(rs.camera_info.serial_number),
           "depth_scale": scale}
    out.write((json.dumps(hdr) + "\n").encode())
    out.flush()
    print(f"[rs_capture] {hdr}", file=sys.stderr, flush=True)
    to_mm = scale * 1000.0
    n, t_log = 0, time.monotonic()
    try:
        while True:
            fs = align.process(pipe.wait_for_frames(5000))
            c, d = fs.get_color_frame(), fs.get_depth_frame()
            if not c or not d:
                continue
            t_ns = int(c.get_timestamp() * 1e6)
            if c.get_frame_timestamp_domain() != rs.timestamp_domain.global_time:
                t_ns = time.time_ns()
            depth = np.asanyarray(d.get_data())
            if abs(to_mm - 1.0) > 1e-6:
                depth = np.clip(depth.astype(np.float32) * to_mm, 0, 65535).astype(np.uint16)
            out.write(struct.pack("<q", t_ns))
            out.write(np.asanyarray(c.get_data()).tobytes())
            out.write(depth.tobytes())
            out.flush()
            n += 1
            if time.monotonic() - t_log >= 10.0:
                print(f"[rs_capture] {n} frames", file=sys.stderr, flush=True)
                t_log = time.monotonic()
    except (BrokenPipeError, KeyboardInterrupt):
        pass
    finally:
        pipe.stop()


if __name__ == "__main__":
    main()
