#!/usr/bin/env python3
"""yoloworld_sidecar.py — persistent YOLO-World box-proposal server (sam_yolo env).

The real-time SAM-6D node runs in `sam6d_ros_humble`, whose ultralytics (8.0)
lacks YOLOWorld. This sidecar runs in `sam_yolo` (ultralytics 8.4), loads
YOLO-World ONCE with the config's unique prompts, and answers per-frame box
requests over a Unix domain socket.

Protocol (length-prefixed, big-endian):
  request : 12 bytes (int32 h, w, c) + h*w*c uint8 BGR bytes
  reply   : 4 bytes (int32 L) + L bytes UTF-8 JSON
            = [{"cls": int, "box": [x1,y1,x2,y2], "conf": float}, ...]
            cls indexes into the config's unique prompt list (same order the
            node computes via yolo_ism_object_n.build_prompt_groups).
"""
import argparse
import json
import os
import socket
import struct
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
import yolo_ism_object_n as o_n

SOCK_DEFAULT = "/tmp/sam6d_yoloworld.sock"


def recvall(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=o_n.DEFAULT_CONFIG)
    ap.add_argument("--sock", default=SOCK_DEFAULT)
    ap.add_argument("--device", default="cuda:0")
    args = ap.parse_args()

    defaults, objs = o_n.load_config(args.config)
    unique_prompts, _ = o_n.build_prompt_groups(objs)
    min_score = min(float(o.get("score_threshold", 0.02)) for o in objs)

    from ultralytics import YOLOWorld
    weights = defaults.get("weights", "yolov8m-worldv2.pt")
    try:
        yolo = YOLOWorld(weights)
    except Exception as e:
        print(f"[sidecar] {weights} failed ({e}); fallback s", flush=True)
        yolo = YOLOWorld("yolov8s-worldv2.pt")
    yolo.set_classes(unique_prompts)
    print(f"[sidecar] YOLO-World ready, {len(unique_prompts)} prompts: {unique_prompts}", flush=True)

    if os.path.exists(args.sock):
        os.remove(args.sock)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(args.sock)
    srv.listen(1)
    print(f"[sidecar] listening on {args.sock} (min_score={min_score})", flush=True)

    while True:
        conn, _ = srv.accept()
        try:
            while True:
                hdr = recvall(conn, 12)
                if hdr is None:
                    break
                h, w, c = struct.unpack(">iii", hdr)
                raw = recvall(conn, h * w * c)
                if raw is None:
                    break
                bgr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, c)
                res = yolo.predict(bgr, conf=min_score, imgsz=640, verbose=False,
                                   device=args.device)
                out = []
                if len(res) and res[0].boxes is not None and len(res[0].boxes) > 0:
                    b = res[0].boxes
                    for j in range(len(b)):
                        xy = b.xyxy[j].tolist()
                        x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                        x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                        out.append({"cls": int(b.cls[j]) if b.cls is not None else 0,
                                    "box": [x1, y1, x2, y2], "conf": float(b.conf[j])})
                payload = json.dumps(out).encode("utf-8")
                conn.sendall(struct.pack(">i", len(payload)) + payload)
        except (ConnectionResetError, BrokenPipeError):
            pass
        finally:
            conn.close()


if __name__ == "__main__":
    main()
