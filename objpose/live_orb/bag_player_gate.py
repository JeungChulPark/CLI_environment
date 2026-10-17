#!/usr/bin/env python3
"""bag_player_gate.py — play a converted RGB-D bag for orb_live_stream.py's bag mode, starting on a trigger.

Runs in the `realsense` conda env (ROS 2 Jazzy, rosbag2_interfaces). It starts `ros2 bag play --start-paused
--start-offset OFFSET` so the 4 GB sqlite bag is already open and positioned, prints READY once the player's resume
service exists, then waits for one line on stdin and calls /rosbag2_player/resume through an rclpy client that is
already initialised. The resume therefore lands within milliseconds of the hub's start message, instead of the
1-2 s a fresh `ros2 bag play` (or a `ros2 service call`) would add — which would leave every SAM-6D result without a
SLAM pose at its stamp. Prints PLAYING after the resume and DONE when the player exits (end of bag).

    python bag_player_gate.py --bag DIR --start-offset 0.5 [--rate 1.0]
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import threading
import time

TOPICS = ["/camera/camera/color/image_raw", "/camera/camera/color/camera_info",
          "/camera/camera/aligned_depth_to_color/image_raw", "/camera/camera/aligned_depth_to_color/camera_info"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--start-offset", type=float, default=0.0)
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--log", default="")
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rosbag2_interfaces.srv import Resume

    cmd = ["ros2", "bag", "play", a.bag, "--start-paused", "--start-offset", f"{a.start_offset:.3f}", "--rate", str(a.rate),
           "--disable-keyboard-controls", "--read-ahead-queue-size", "2000", "--topics", *TOPICS]
    log = open(a.log, "w") if a.log else subprocess.DEVNULL
    player = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def stop(*_):
        if player.poll() is None:
            try:
                os.killpg(player.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        sys.exit(0)
    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, stop)

    rclpy.init(args=None)
    node = Node("objpose_bag_gate")
    cli = node.create_client(Resume, "/rosbag2_player/resume")
    t_end = time.monotonic() + 120
    while not cli.wait_for_service(timeout_sec=0.5):
        if player.poll() is not None or time.monotonic() > t_end:
            print(f"FAILED player rc={player.poll()}", flush=True)
            return 1
    print("READY", flush=True)

    line = sys.stdin.readline()
    if not line:
        stop()
    fut = cli.call_async(Resume.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    print(f"PLAYING {time.time_ns()}", flush=True)

    player.wait()
    print(f"DONE rc={player.returncode}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
