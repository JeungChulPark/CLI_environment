#!/usr/bin/env python3
"""Extract aligned-depth frames synced (by timestamp) to the color frames the
yolo_ism pipeline processed, so geometric scoring can load depth per frame.

Mapping: processed frame `name` (in yolo_ism_final CSV) -> bag color-message
index -> nearest aligned_depth message by timestamp -> save uint16 PNG as
outputs/depth_synced/<bag>/<name>.png.
  reuse-frame bags: color_msg_idx = int(name) * stride_extract (= round(Ncolor/Nfiles))
  decode bag (milk_nomilk_bag): color_msg_idx = int(name)
"""
import bisect
import csv
import os
import sys

import cv2
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO, "outputs", "depth_synced")
FINAL = os.path.join(REPO, "outputs", "yolo_ism_final")
COLOR = "/camera/camera/color/image_raw"
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"

BAGS = ["high_texture_around", "high_texture_far_close", "two_table_around",
        "two_table_around_goback", "two_table_diagonal1", "two_table_diagonal2",
        "two_table_goback", "only_milk", "milk_nomilk_bag"]
FRAMES_DIR = {"only_milk": "outputs/yolo_test/only_Milk/frames"}


def bag_uri(bag):
    p = os.path.join(REPO, "data", "ros2_bag", bag)
    return os.path.join(p, "bag") if os.path.isdir(os.path.join(p, "bag")) else p


def main():
    ts = get_typestore(Stores.ROS2_HUMBLE)
    for bag in BAGS:
        csv_path = os.path.join(FINAL, bag, "yolo_ism_results.csv")
        names = [r["timestamp"] for r in csv.DictReader(open(csv_path))]
        frames_dir = os.path.join(REPO, FRAMES_DIR.get(bag,
                                  f"outputs/yolo_test/{bag}/frames"))
        reuse = os.path.isdir(frames_dir) and \
            len([f for f in os.listdir(frames_dir) if f.endswith((".png", ".jpg"))]) > 0
        nfiles = len([f for f in os.listdir(frames_dir)
                      if f.endswith((".png", ".jpg"))]) if reuse else 0

        out_dir = os.path.join(OUT, bag)
        os.makedirs(out_dir, exist_ok=True)
        with AnyReader([Path(bag_uri(bag))], default_typestore=ts) as r:
            ccon = [c for c in r.connections if c.topic == COLOR]
            dcon = [c for c in r.connections if c.topic == DEPTH]
            color_ts = [t for _, t, _ in r.messages(connections=ccon)]
            ncolor = len(color_ts)
            stride_ex = max(1, round(ncolor / nfiles)) if reuse else 1
            # depth timestamps (ordered)
            depth_ts = [t for _, t, _ in r.messages(connections=dcon)]

            # map each processed name -> target color ts -> nearest depth idx
            need = {}  # depth_idx -> save_name
            for nm in names:
                cidx = int(nm) * stride_ex if reuse else int(nm)
                cidx = min(cidx, ncolor - 1)
                tt = color_ts[cidx]
                j = bisect.bisect_left(depth_ts, tt)
                cand = [k for k in (j - 1, j) if 0 <= k < len(depth_ts)]
                didx = min(cand, key=lambda k: abs(depth_ts[k] - tt))
                need.setdefault(didx, nm)

            # 2nd pass: save needed depth frames
            saved = 0
            for i, (conn, t, raw) in enumerate(r.messages(connections=dcon)):
                if i not in need:
                    continue
                m = r.deserialize(raw, conn.msgtype)
                d = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)
                cv2.imwrite(os.path.join(out_dir, f"{need[i]}.png"), d)
                saved += 1
        print(f"[{bag}] reuse={reuse} stride_ex={stride_ex} ncolor={ncolor} "
              f"ndepth={len(depth_ts)} names={len(names)} depth_saved={saved}")


if __name__ == "__main__":
    main()
