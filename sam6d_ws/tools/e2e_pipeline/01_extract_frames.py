#!/usr/bin/env python3
"""01_extract_frames.py — ROS2 bag -> synchronized RGB-D frames + camera.json.

Reads color + aligned_depth + camera_info from a ROS2 bag, pairs each sampled
color frame (stride=N) with its nearest-timestamp aligned-depth frame, and writes
SAM-6D-ready inputs:

  <out>/rgb/{idx:06d}.png      bgr8 color
  <out>/depth/{idx:06d}.png    uint16 depth in millimeters (aligned to color)
  <out>/camera.json            {"cam_K":[9 floats], "depth_scale":1.0}

Run (ROS env):
  conda run -n sam6d_ros_humble python tools/e2e_pipeline/01_extract_frames.py \
      --bag data/ros2_bag/two_table_around --out outputs_e2e/_frames/two_table_around --stride 10
"""
import argparse
import glob
import json
import os
from collections import deque

import numpy as np
import cv2

COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/camera/camera/color/camera_info"


def resolve_bag(bag):
    """Accept .db3/.mcap file or a (possibly nested) bag directory."""
    if os.path.isfile(bag):
        path = bag
    else:
        cands = (glob.glob(os.path.join(bag, "**", "*.db3"), recursive=True)
                 + glob.glob(os.path.join(bag, "**", "*.mcap"), recursive=True))
        if not cands:
            raise FileNotFoundError(f"no .db3/.mcap under {bag}")
        path = sorted(cands)[0]
    storage_id = "mcap" if path.endswith(".mcap") else "sqlite3"
    return path, storage_id


def color_to_bgr(msg):
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    h, w = msg.height, msg.width
    if enc == "rgb8":
        return cv2.cvtColor(buf.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    if enc == "bgr8":
        return buf.reshape(h, w, 3)
    if enc == "mono8":
        return cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
    from cv_bridge import CvBridge
    return CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")


def depth_to_u16_mm(msg):
    """aligned_depth_to_color is 16UC1 in millimeters."""
    h, w = msg.height, msg.width
    d = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=10)
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--color-topic", default=COLOR_TOPIC)
    ap.add_argument("--depth-topic", default=DEPTH_TOPIC)
    ap.add_argument("--info-topic", default=INFO_TOPIC)
    args = ap.parse_args()

    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, CameraInfo

    uri, storage_id = resolve_bag(args.bag)
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id=storage_id), ConverterOptions("", ""))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}

    # Auto-discover topics if the configured names are absent.
    def pick(preferred, kind):
        if preferred in topics:
            return preferred
        cand = [n for n, ty in topics.items() if kind in ty]
        return cand[0] if cand else None

    color_topic = args.color_topic if args.color_topic in topics else None
    depth_topic = args.depth_topic if args.depth_topic in topics else None
    info_topic = args.info_topic if args.info_topic in topics else None
    if color_topic is None or depth_topic is None:
        imgs = [n for n, ty in topics.items() if "Image" in ty]
        if color_topic is None:
            color_topic = next((n for n in imgs if "color" in n and "depth" not in n), None)
        if depth_topic is None:
            depth_topic = next((n for n in imgs if "depth" in n), None)
    if info_topic is None:
        info_topic = pick(args.info_topic, "CameraInfo")

    if not color_topic or not depth_topic:
        raise SystemExit(f"[extract] missing color/depth topic in {uri}. "
                         f"image topics: {[n for n,ty in topics.items() if 'Image' in ty]}")

    rgb_dir = os.path.join(args.out, "rgb")
    depth_dir = os.path.join(args.out, "depth")
    os.makedirs(rgb_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)

    cam_K = None
    depth_ring = deque(maxlen=12)   # (t_ns, depth_u16)
    pending = deque()               # (t_ns, bgr_img, out_idx)
    latest_depth_t = None
    seen_color = saved = 0
    out_idx = 0

    def flush_ready(force=False):
        nonlocal saved
        while pending:
            ct, cimg, oidx = pending[0]
            if not force and (latest_depth_t is None or ct > latest_depth_t):
                break  # wait for a bracketing depth frame
            if not depth_ring:
                pending.popleft()
                continue
            dt, dimg = min(depth_ring, key=lambda kv: abs(kv[0] - ct))
            cv2.imwrite(os.path.join(rgb_dir, f"{oidx:06d}.png"), cimg)
            cv2.imwrite(os.path.join(depth_dir, f"{oidx:06d}.png"), dimg)
            pending.popleft()
            saved += 1

    print(f"[extract] bag={uri}\n          color={color_topic} depth={depth_topic} info={info_topic}",
          flush=True)
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic == info_topic and cam_K is None:
            msg = deserialize_message(data, CameraInfo)
            cam_K = [float(x) for x in msg.k]
        elif topic == depth_topic:
            msg = deserialize_message(data, Image)
            depth_ring.append((t_ns, depth_to_u16_mm(msg)))
            latest_depth_t = t_ns
            flush_ready()
        elif topic == color_topic:
            if seen_color % args.stride == 0:
                msg = deserialize_message(data, Image)
                pending.append((t_ns, color_to_bgr(msg), out_idx))
                out_idx += 1
                flush_ready()
                if args.max_frames and out_idx >= args.max_frames:
                    seen_color += 1
                    break
            seen_color += 1
    flush_ready(force=True)

    if cam_K is None:
        # Fallback: RealSense D435 color default if CameraInfo absent.
        print("[extract] WARN: no CameraInfo; writing placeholder cam_K", flush=True)
        cam_K = [615.0, 0.0, 320.0, 0.0, 615.0, 240.0, 0.0, 0.0, 1.0]
    with open(os.path.join(args.out, "camera.json"), "w") as f:
        json.dump({"cam_K": cam_K, "depth_scale": 1.0}, f, indent=2)

    print(f"[extract] color_seen={seen_color} saved_pairs={saved} -> {args.out}", flush=True)
    # Machine-readable result line for the orchestrator.
    print(json.dumps({"bag": os.path.basename(args.out.rstrip('/')),
                      "saved": saved, "color_topic": color_topic,
                      "depth_topic": depth_topic, "info_topic": info_topic}))


if __name__ == "__main__":
    main()
