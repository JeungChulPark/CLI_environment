#!/usr/bin/env python3
"""vlp16_publisher.py — live VLP-16 UDP packets -> sensor_msgs/PointCloud2 on /velodyne_points (2026-09-29).

Stand-in for `ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py` on hosts without the ROS velodyne packages
(the Samsung laptop: robostack Jazzy env, no internet). Decoding = vlp16_decode.py, validated against the driver's
own clouds to < 1 mm. One scan per revolution (cut where the block azimuth wraps), organized [firing, ring] like the
driver, header stamp = host time of the scan's last packet, frame "velodyne".
    python vlp16_publisher.py [--port 2368] [--topic /velodyne_points] [--frame velodyne]
Needs an IPv4 address on the LiDAR's subnet (factory: sensor 192.168.1.201 -> broadcast, e.g. host 192.168.1.77/24).
"""
import argparse
import socket
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

sys.path.insert(0, str(Path(__file__).resolve().parent))
from vlp16_decode import decode_scan  # noqa: E402

FIELDS = [("x", 0, PointField.FLOAT32), ("y", 4, PointField.FLOAT32), ("z", 8, PointField.FLOAT32),
          ("intensity", 12, PointField.FLOAT32), ("ring", 16, PointField.UINT16), ("time", 18, PointField.FLOAT32)]
DT = np.dtype({"names": [f[0] for f in FIELDS], "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
               "offsets": [f[1] for f in FIELDS], "itemsize": 22})


def to_msg(d, stamp_ns, frame):
    h, w = d["x"].shape
    arr = np.empty((h, w), DT)
    for k in ("x", "y", "z", "intensity", "ring", "time"):
        arr[k] = d[k]
    m = PointCloud2()
    m.header.stamp.sec, m.header.stamp.nanosec = stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000
    m.header.frame_id = frame
    m.height, m.width = h, w
    m.fields = [PointField(name=n, offset=o, datatype=t, count=1) for n, o, t in FIELDS]
    m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 22, 22 * w, False
    m.data = arr.tobytes()
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=2368)
    ap.add_argument("--topic", default="/velodyne_points")
    ap.add_argument("--frame", default="velodyne")
    ap.add_argument("--status-s", type=float, default=10.0)
    a = ap.parse_args()
    rclpy.init()
    node = Node("vlp16_publisher")
    pub = node.create_publisher(PointCloud2, a.topic, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    sock.bind(("", a.port))
    sock.settimeout(1.0)
    pkts, last_az, n_scans, n_pk, t_log = [], None, 0, 0, time.monotonic()
    first = True          # the first revolution starts wherever the sensor was: a partial scan (11..75 packets)
    print(f"[vlp16] listening udp {a.port} -> {a.topic}", flush=True)
    try:
        while rclpy.ok():
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                print("[vlp16] no packets for 1 s (LiDAR connected / IP on its subnet?)", flush=True)
                continue
            if len(data) != 1206 or data[0] != 0xFF or data[1] != 0xEE:
                continue                                      # position packets (port 8308) or garbage
            now = time.time_ns()
            az = data[2] | (data[3] << 8)
            n_pk += 1
            if last_az is not None and az < last_az and len(pkts) > 10:    # wrap: one revolution done
                # the partial first scan is dropped: the fixed-map localizer once took its global fix from it and
                # started 5 m away, 110 deg off (2026-10-07), and hdl_graph_slam made it keyframe 1
                if not first:
                    pub.publish(to_msg(decode_scan(pkts), last_stamp, a.frame))
                    n_scans += 1
                first = False
                pkts = []
            pkts.append(data)
            # compare FIRST-block azimuths: the 360 -> 0 wrap often falls inside a packet (its last block is already past 0)
            last_az, last_stamp = az, now
            if time.monotonic() - t_log >= a.status_s:
                print(f"[vlp16] {n_scans} scans, {n_pk} packets", flush=True)
                t_log = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
