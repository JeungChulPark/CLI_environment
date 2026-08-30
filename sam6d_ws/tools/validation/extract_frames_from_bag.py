#!/usr/bin/env python3
"""Generalized ROS2 bag -> PNG frame extractor (read-only, validation tool).

Reads an image topic from a ROS2 bag (file or directory) sequentially and writes
each frame to <out>/<prefix><index:06d>.png in message order. Unlike the sync-based
extract_bag_frames.py, this is a single-topic sequential reader (deterministic,
no node/playback needed) — reusable for any bag.

Two backends (auto-selected; identical PNG output):
  - rosbag2 : rosbag2_py + rclpy (ROS env, e.g. sam6d_ros_humble)
  - rosbags : pure-python `rosbags` lib, NO ROS install needed (e.g. sam_yolo env)

Run (ROS env):
  conda run -n sam6d_ros_humble python tools/validation/extract_frames_from_bag.py \
      --bag data/only_milk --out outputs/yolo_test/only_Milk/frames \
      --topic /camera/camera/color/image_raw

Run (no-ROS env, pure-python rosbags):
  conda run -n sam_yolo python tools/validation/extract_frames_from_bag.py --backend rosbags \
      --bag data/ros2_bag/high_texture_around --out outputs/yolo_test/high_texture_around/frames \
      --topic /camera/camera/color/image_raw --stride 2

Options: --backend {auto,rosbag2,rosbags}, --max-frames N, --stride K, --start-index S, --prefix STR
"""
import argparse
import os
import glob

import numpy as np
import cv2


def resolve_bag(bag):
    """Accept a .db3/.mcap file or a bag directory; return (uri, storage_id)."""
    if os.path.isfile(bag):
        path = bag
    else:
        cands = (glob.glob(os.path.join(bag, "*.db3"))
                 + glob.glob(os.path.join(bag, "*.mcap")))
        if not cands:
            raise FileNotFoundError(f"no .db3/.mcap under {bag}")
        path = sorted(cands)[0]
    storage_id = "mcap" if path.endswith(".mcap") else "sqlite3"
    return path, storage_id


def imgmsg_to_bgr(msg):
    """Minimal Image->BGR decode (no cv_bridge dependency on encoding quirks)."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    h, w = msg.height, msg.width
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(h, w, 3)
        if enc == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img
    if enc in ("mono8",):
        return cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
    if enc in ("16uc1", "mono16"):
        d = buf.view(np.uint16).reshape(h, w)
        return cv2.cvtColor(cv2.convertScaleAbs(d, alpha=0.03), cv2.COLOR_GRAY2BGR)
    # fallback: try cv_bridge
    from cv_bridge import CvBridge
    return CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")


def _save(img, args, out_idx):
    fn = os.path.join(args.out, f"{args.prefix}{out_idx:06d}.png")
    cv2.imwrite(fn, img)


def extract_rosbag2(args):
    """ROS-env backend: rosbag2_py + rclpy deserialization."""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image

    uri, storage_id = resolve_bag(args.bag)
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id=storage_id), ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in types:
        raise SystemExit(f"topic {args.topic} not in bag. available image topics: "
                         + ", ".join(n for n, ty in types.items() if "Image" in ty))

    seen = saved = 0
    out_idx = args.start_index
    print(f"[extract:rosbag2] bag={uri} topic={args.topic} -> {args.out}", flush=True)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != args.topic:
            continue
        if seen % args.stride == 0:
            _save(imgmsg_to_bgr(deserialize_message(data, Image)), args, out_idx)
            out_idx += 1; saved += 1
            if saved % 100 == 0:
                print(f"  saved {saved} (msg {seen})", flush=True)
            if args.max_frames and saved >= args.max_frames:
                break
        seen += 1
    return seen, saved, out_idx


def extract_rosbags(args):
    """No-ROS backend: pure-python `rosbags` lib with a ROS2 typestore."""
    from pathlib import Path
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores

    ts = get_typestore(Stores.ROS2_HUMBLE)
    uri, _ = resolve_bag(args.bag)
    seen = saved = 0
    out_idx = args.start_index
    print(f"[extract:rosbags] bag={uri} topic={args.topic} -> {args.out}", flush=True)
    with AnyReader([Path(uri)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == args.topic]
        if not conns:
            img_topics = ", ".join(c.topic for c in reader.connections if "Image" in c.msgtype)
            raise SystemExit(f"topic {args.topic} not in bag. available image topics: {img_topics}")
        for conn, _, raw in reader.messages(connections=conns):
            if seen % args.stride == 0:
                _save(imgmsg_to_bgr(reader.deserialize(raw, conn.msgtype)), args, out_idx)
                out_idx += 1; saved += 1
                if saved % 100 == 0:
                    print(f"  saved {saved} (msg {seen})", flush=True)
                if args.max_frames and saved >= args.max_frames:
                    break
            seen += 1
    return seen, saved, out_idx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="bag file (.db3/.mcap) or directory")
    ap.add_argument("--out", required=True, help="output PNG directory")
    ap.add_argument("--topic", default="/camera/camera/color/image_raw")
    ap.add_argument("--backend", choices=["auto", "rosbag2", "rosbags"], default="auto",
                    help="auto: try rosbag2_py, fall back to rosbags (pure-python, no ROS)")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--start-index", type=int, default=0)
    ap.add_argument("--prefix", default="")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    backend = args.backend
    if backend == "auto":
        try:
            import rosbag2_py  # noqa: F401
            backend = "rosbag2"
        except Exception:
            backend = "rosbags"

    seen, saved, out_idx = (extract_rosbag2 if backend == "rosbag2" else extract_rosbags)(args)

    print(f"[done] backend={backend} topic messages seen={seen}, frames saved={saved}")
    if saved:
        print(f"[done] sample: {args.prefix}{args.start_index:06d}.png .. "
              f"{args.prefix}{out_idx-1:06d}.png  in {args.out}")


if __name__ == "__main__":
    main()
