#!/usr/bin/env python3
"""Batch runner for yolo_ism.py over multiple ROS2 bags.

Does NOT modify yolo_ism.py inference logic; it only orchestrates per-bag runs.
For each bag:
  - delete outputs/yolo_ism/<bag>/ if present, recreate it
  - run yolo_ism.py (stride 10) writing into that dir
  - tee combined stdout/stderr to outputs/yolo_ism/<bag>/run.log
  - on non-zero exit, write outputs/yolo_ism/<bag>/run_error.txt and continue
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# (bag_name, frames_dir_override)  -- "" means let yolo_ism auto-resolve / bag-decode
BAGS = [
    ("high_texture_around", ""),
    ("high_texture_far_close", ""),
    ("two_table_around", ""),
    ("two_table_around_goback", ""),
    ("two_table_diagonal1", ""),
    ("two_table_diagonal2", ""),
    ("two_table_goback", ""),
    # frames extracted under capitalized dir name; point explicitly
    ("only_milk", "outputs/yolo_test/only_Milk/frames"),
    # no extracted frames -> yolo_ism decodes data/ros2_bag/milk_nomilk_bag/bag_0.db3
    ("milk_nomilk_bag", ""),
]

STRIDE = 10


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conf", type=float, default=0.05,
                    help="YOLO score-threshold passed to yolo_ism.py")
    ap.add_argument("--sim", type=float, default=None,
                    help="semantic similarity-threshold passed to yolo_ism.py")
    ap.add_argument("--out-root", default=os.path.join(REPO, "outputs", "yolo_ism"))
    ap.add_argument("--use-mask", action="store_true")
    ap.add_argument("--use-geometric", action="store_true")
    ap.add_argument("--appe-gate", type=float, default=None)
    args = ap.parse_args()
    out_root = args.out_root if os.path.isabs(args.out_root) \
        else os.path.join(REPO, args.out_root)

    summary = []
    for bag, frames_dir in BAGS:
        out_dir = os.path.join(out_root, bag)
        if os.path.isdir(out_dir):
            shutil.rmtree(out_dir)
        os.makedirs(out_dir, exist_ok=True)
        log_path = os.path.join(out_dir, "run.log")

        cmd = [
            sys.executable, os.path.join(REPO, "yolo_ism.py"),
            "--bag", os.path.join("data", "ros2_bag", bag),
            "--output-dir", out_dir + "/",
            "--stride", str(STRIDE),
            "--score-threshold", str(args.conf),
        ]
        if args.sim is not None:
            cmd += ["--similarity-threshold", str(args.sim)]
        if args.use_mask:
            cmd += ["--use-mask"]
        if args.use_geometric:
            cmd += ["--use-geometric"]
        if args.appe_gate is not None:
            cmd += ["--appe-gate", str(args.appe_gate)]
        if frames_dir:
            cmd += ["--frames-dir", os.path.join(REPO, frames_dir)]

        print(f"\n========== {bag} ==========", flush=True)
        print(" ".join(cmd), flush=True)
        t0 = time.time()
        with open(log_path, "w") as logf:
            logf.write(" ".join(cmd) + "\n\n")
            logf.flush()
            proc = subprocess.run(cmd, cwd=REPO, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, text=True)
            logf.write(proc.stdout)
        dt = time.time() - t0
        # mirror tail to console
        tail = "\n".join(proc.stdout.splitlines()[-15:])
        print(tail, flush=True)

        if proc.returncode != 0:
            err_path = os.path.join(out_dir, "run_error.txt")
            with open(err_path, "w") as ef:
                ef.write(f"yolo_ism.py exited with code {proc.returncode}\n")
                ef.write("---- last 40 log lines ----\n")
                ef.write("\n".join(proc.stdout.splitlines()[-40:]))
            print(f"[FAIL] {bag} rc={proc.returncode} ({dt:.1f}s) -> {err_path}", flush=True)
            summary.append((bag, "FAIL", dt))
        else:
            print(f"[OK]   {bag} ({dt:.1f}s)", flush=True)
            summary.append((bag, "OK", dt))

    print("\n===== BATCH SUMMARY =====", flush=True)
    for bag, status, dt in summary:
        print(f"{status:5s} {bag:26s} {dt:7.1f}s", flush=True)


if __name__ == "__main__":
    main()
