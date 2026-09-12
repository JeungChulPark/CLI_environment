#!/usr/bin/env python3
"""
Record RTAB-Map odometry latency from /odom_info so it can be compared like-for-like
with ORB-SLAM3's own per-frame tracking time.

What is measured
----------------
`OdomInfo.time_estimation` is the wall time Odometry::process() spent on one frame.
That is RTAB-Map's equivalent of ORB-SLAM3's TrackRGBD() duration: the work that
must finish inside the frame period. Loop closure and graph optimisation run on the
separate RtabmapThread at Rtabmap/DetectionRate (1 Hz by default) and deliberately
do NOT appear here -- that separation is the whole point of RTAB-Map's design, so
the honest comparison is odometry-vs-tracking, with mapping cost reported apart.

Also recorded, because they explain the latency: feature count, matches, inliers,
local map size, local bundle time, and the `lost` flag.

Output: a CSV of every frame plus a summary with mean/p50/p95/p99/max and the count
over the 33.33 ms deadline.
"""

import argparse
import csv
import signal
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rtabmap_msgs.msg import OdomInfo


class Profiler(Node):
    def __init__(self, out_csv, topic):
        super().__init__("rtabmap_profile")
        self.rows = []
        self.out_csv = out_csv
        qos = QoSProfile(depth=50)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.sub = self.create_subscription(OdomInfo, topic, self.cb, qos)
        self.get_logger().info(f"listening on {topic}")

    def cb(self, m):
        self.rows.append({
            "time_estimation_ms": m.time_estimation * 1000.0,
            "time_filtering_ms": m.time_particle_filtering * 1000.0,
            "local_bundle_ms": m.local_bundle_time * 1000.0,
            "features": m.features,
            "matches": m.matches,
            "inliers": m.inliers,
            "local_map_size": m.local_map_size,
            "local_key_frames": m.local_key_frames,
            "lost": int(bool(m.lost)),
        })
        n = len(self.rows)
        if n % 500 == 0:
            a = np.array([r["time_estimation_ms"] for r in self.rows[-500:]])
            self.get_logger().info(
                f"{n} frames  odom {a.mean():.1f} ms avg / {np.percentile(a,95):.1f} p95")

    def report(self):
        if not self.rows:
            print("\nNO /odom_info MESSAGES RECEIVED - odometry never ran.")
            return
        with open(self.out_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            w.writeheader()
            w.writerows(self.rows)

        t = np.array([r["time_estimation_ms"] for r in self.rows])
        lost = sum(r["lost"] for r in self.rows)
        n = len(t)
        print("\n" + "=" * 68)
        print(f"RTAB-Map odometry  ({n} frames, {lost} lost)")
        print("=" * 68)
        print(f"  mean   {t.mean():8.2f} ms")
        print(f"  p50    {np.percentile(t,50):8.2f} ms")
        print(f"  p95    {np.percentile(t,95):8.2f} ms")
        print(f"  p99    {np.percentile(t,99):8.2f} ms")
        print(f"  max    {t.max():8.2f} ms")
        print(f"  min    {t.min():8.2f} ms")
        over = int((t > 33.33).sum())
        print(f"  over 33.33 ms: {over}/{n} ({100.0*over/n:.2f}%)")
        for name in ("local_bundle_ms", "features", "matches", "inliers",
                     "local_map_size", "local_key_frames"):
            a = np.array([r[name] for r in self.rows], dtype=float)
            print(f"  {name:<18} mean {a.mean():9.1f}  p95 {np.percentile(a,95):9.1f}"
                  f"  max {a.max():9.1f}")
        print(f"\n  csv -> {self.out_csv}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="rtabmap_odom_timing.csv")
    ap.add_argument("--topic", default="/odom_info")
    args = ap.parse_args()

    rclpy.init()
    node = Profiler(args.out, args.topic)

    # rclpy.spin() blocks inside C code holding the GIL, so a Python signal
    # handler installed on top of it never runs and the CSV is never written.
    # Spin in short slices instead so the interpreter gets a chance to deliver
    # the signal between calls.
    stop = {"now": False}

    def on_sig(_s, _f):
        stop["now"] = True

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)
    try:
        while rclpy.ok() and not stop["now"]:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.report()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
