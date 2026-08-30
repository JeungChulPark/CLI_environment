#!/usr/bin/env python3
"""07_backfill.py — recover coverage for objects truncated by intermittent crashes.

The PEM path occasionally aborts with a glibc heap corruption (corrupted double-linked
list / corrupted size vs. prev_size) on Blackwell, truncating an object mid-run. This
re-runs only the (bag, object) pairs whose processed-frame coverage is below target,
using the engine's --skip_existing resume so each attempt fills in remaining frames.

Run (ROS env):
  conda run -n sam6d_ros_humble python tools/e2e_pipeline/07_backfill.py --gpus 0,1,2,3 --max-attempts 3
"""
import argparse
import glob
import os
import queue
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, os.path.dirname(__file__))
import pipeline_lib as L  # noqa: E402

FRAME_ROOT = os.path.join(L.OUT, "_frames")
RAW_ROOT = os.path.join(L.OUT, "_raw")
ENGINE_DIR = os.path.dirname(L.ENGINE)
_lock = threading.Lock()


def n_frames(bag):
    d = os.path.join(FRAME_ROOT, bag, "rgb")
    return len(glob.glob(os.path.join(d, "*.png"))) if os.path.isdir(d) else 0


def n_processed(bag, obj):
    raw = os.path.join(RAW_ROOT, bag, obj)
    if not os.path.isdir(raw):
        return 0
    return sum(1 for d in os.listdir(raw)
               if d[0:1].isdigit() and os.path.isfile(os.path.join(raw, d, "detection_ism.json")))


def find_incomplete(bags, objects, target):
    todo = []
    for bag in bags:
        nf = n_frames(bag)
        if nf == 0:
            continue
        for obj in objects:
            raw = os.path.join(RAW_ROOT, bag, obj)
            if not os.path.isdir(raw):
                continue  # object never started here; not a backfill case
            if n_processed(bag, obj) < int(nf * target):
                todo.append((bag, obj, nf))
    return todo


def run_attempt(bag, obj, gpu, det_thresh):
    raw_out = os.path.join(RAW_ROOT, bag, obj)
    frame_dir = os.path.join(FRAME_ROOT, bag)
    env = dict(os.environ)
    env["CUDA_VISIBLE_DEVICES"] = str(gpu)
    env["CUDA_LAUNCH_BLOCKING"] = "1"
    cmd = [sys.executable, L.ENGINE,
           "--rgb_dir", os.path.join(frame_dir, "rgb"),
           "--depth_dir", os.path.join(frame_dir, "depth"),
           "--cam_path", os.path.join(frame_dir, "camera.json"),
           "--cad_path", L.resolve_cad(obj), "--template_dir", L.template_path(obj),
           "--output_dir", raw_out, "--det_score_thresh", str(det_thresh),
           "--skip_existing", "--quiet"]
    return subprocess.run(cmd, cwd=ENGINE_DIR, env=env, capture_output=True, text=True).returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gpus", default="0,1,2,3")
    ap.add_argument("--max-attempts", type=int, default=3)
    ap.add_argument("--target", type=float, default=0.98, help="coverage fraction to reach")
    ap.add_argument("--det_score_thresh", type=float, default=0.2)
    ap.add_argument("--bags", default="ALL", help="comma list of bag names, or ALL")
    args = ap.parse_args()

    if args.bags == "ALL":
        bags = [b for b in L.TARGET_BAGS if os.path.isdir(os.path.join(RAW_ROOT, b))]
    else:
        want = [b.strip() for b in args.bags.split(",") if b.strip()]
        bags = [b for b in want if os.path.isdir(os.path.join(RAW_ROOT, b))]
    objects = L.list_objects()
    gpus = [g.strip() for g in args.gpus.split(",") if g.strip()]
    gpu_q = queue.Queue()
    for g in gpus:
        gpu_q.put(g)

    todo = find_incomplete(bags, objects, args.target)
    print(f"[backfill] incomplete pairs: {len(todo)}")
    for bag, obj, nf in todo:
        print(f"    - {bag}/{obj}: {n_processed(bag,obj)}/{nf}")
    if not todo:
        print("[backfill] nothing to do")
        return

    def work(item):
        bag, obj, nf = item
        for attempt in range(1, args.max_attempts + 1):
            before = n_processed(bag, obj)
            if before >= int(nf * args.target):
                break
            gpu = gpu_q.get()
            try:
                t0 = time.time()
                rc = run_attempt(bag, obj, gpu, args.det_score_thresh)
            finally:
                gpu_q.put(gpu)
            after = n_processed(bag, obj)
            with _lock:
                print(f"  [{bag}/{obj}] attempt {attempt} gpu{gpu} rc={rc} "
                      f"{before}->{after}/{nf} ({time.time()-t0:.0f}s)", flush=True)
            if after <= before:  # no progress this attempt
                if attempt >= 2:
                    break
        return (bag, obj, n_processed(bag, obj), nf)

    with ThreadPoolExecutor(max_workers=len(gpus)) as ex:
        results = list(ex.map(work, todo))

    done = sum(1 for _, _, p, nf in results if p >= int(nf * args.target))
    print(f"\n[backfill] complete {done}/{len(results)} pairs reached {args.target:.0%} coverage")
    for bag, obj, p, nf in results:
        if p < int(nf * args.target):
            print(f"    STILL PARTIAL {bag}/{obj}: {p}/{nf}")


if __name__ == "__main__":
    main()
