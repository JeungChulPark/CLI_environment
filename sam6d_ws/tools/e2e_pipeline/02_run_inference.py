#!/usr/bin/env python3
"""02_run_inference.py — orchestrate SAM-6D over (bag x object) on multiple GPUs.

For each target bag: ensures frames are extracted (stride=10), then runs every
preflight-OK object through run_batch_inference_fast.py, one subprocess per
(bag, object), round-robin across GPUs. Failures are isolated (object/frame/bag)
and logged; the run never aborts.

Sets CUDA_LAUNCH_BLOCKING=1 — required on RTX PRO 6000 (Blackwell sm_120) to avoid
an async TensorAdvancedIndexing kernel assert in the ISM path.

Run (ROS env):
  conda run -n sam6d_ros_humble python tools/e2e_pipeline/02_run_inference.py \
      --bags two_table_around --gpus 0,1,2,3
  # all target bags:
  conda run -n sam6d_ros_humble python tools/e2e_pipeline/02_run_inference.py --bags ALL --gpus 0,1,2,3
"""
import argparse
import csv
import json
import os
import queue
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, os.path.dirname(__file__))
import pipeline_lib as L  # noqa: E402

ENGINE_DIR = os.path.dirname(L.ENGINE)
LOG_DIR = os.path.join(L.OUT, "_logs")
FRAME_ROOT = os.path.join(L.OUT, "_frames")
RAW_ROOT = os.path.join(L.OUT, "_raw")

_print_lock = threading.Lock()


def log(line):
    with _print_lock:
        print(line, flush=True)
        with open(os.path.join(LOG_DIR, "run.log"), "a") as f:
            f.write(line + "\n")


def ensure_frames(bag, stride):
    out = os.path.join(FRAME_ROOT, bag)
    rgb_dir = os.path.join(out, "rgb")
    if os.path.isdir(rgb_dir) and len(os.listdir(rgb_dir)) > 0 and \
       os.path.isfile(os.path.join(out, "camera.json")):
        n = len(os.listdir(rgb_dir))
        log(f"[frames] {bag}: reuse {n} frames")
        return out, n
    bag_path = os.path.join(L.BAG_DIR, bag)
    cmd = [sys.executable, os.path.join(os.path.dirname(__file__), "01_extract_frames.py"),
           "--bag", bag_path, "--out", out, "--stride", str(stride)]
    log(f"[frames] {bag}: extracting (stride={stride}) ...")
    r = subprocess.run(cmd, capture_output=True, text=True)
    n = 0
    if os.path.isdir(rgb_dir):
        n = len(os.listdir(rgb_dir))
    if r.returncode != 0 or n == 0:
        log(f"[frames] {bag}: FAILED ({r.returncode}) {r.stderr.strip()[-300:]}")
        return None, 0
    log(f"[frames] {bag}: {n} frames")
    return out, n


def run_job(bag, obj, gpu, frame_dir, det_thresh, n_frames):
    raw_out = os.path.join(RAW_ROOT, bag, obj)
    os.makedirs(raw_out, exist_ok=True)
    cad = L.resolve_cad(obj)
    tmpl = L.template_path(obj)
    env = dict(os.environ)
    env["CUDA_VISIBLE_DEVICES"] = str(gpu)
    env["CUDA_LAUNCH_BLOCKING"] = "1"
    cmd = [sys.executable, L.ENGINE,
           "--rgb_dir", os.path.join(frame_dir, "rgb"),
           "--depth_dir", os.path.join(frame_dir, "depth"),
           "--cam_path", os.path.join(frame_dir, "camera.json"),
           "--cad_path", cad, "--template_dir", tmpl,
           "--output_dir", raw_out, "--det_score_thresh", str(det_thresh),
           "--quiet"]
    t0 = time.time()
    r = subprocess.run(cmd, cwd=ENGINE_DIR, env=env, capture_output=True, text=True)
    elapsed = time.time() - t0
    # Count PEM poses (vis_pem) and frames actually processed (per-frame ISM dirs).
    n_pem = 0
    vis_dir = os.path.join(raw_out, "vis_pem")
    if os.path.isdir(vis_dir):
        n_pem = len([f for f in os.listdir(vis_dir) if f.endswith(".png")])
    n_proc = 0
    if os.path.isdir(raw_out):
        for d in os.listdir(raw_out):
            if d[0:1].isdigit() and os.path.isfile(os.path.join(raw_out, d, "detection_ism.json")):
                n_proc += 1
    # A teardown crash (corrupted double-linked list at exit) returns nonzero but
    # leaves a full result set — treat coverage, not exit code, as success.
    covered = n_frames > 0 and n_proc >= int(n_frames * 0.98)
    ok = (r.returncode == 0) or covered
    if r.returncode == 0:
        err = ""
    elif covered:
        err = f"teardown-crash (covered {n_proc}/{n_frames}): " + \
              (r.stderr.strip().splitlines()[-1] if r.stderr.strip() else "nonzero exit")
    else:
        err = f"partial {n_proc}/{n_frames}: " + \
              (r.stderr.strip().splitlines()[-1] if r.stderr.strip() else "nonzero exit")
    return {"bag": bag, "obj": obj, "gpu": gpu, "elapsed": round(elapsed, 1),
            "n_pem": n_pem, "n_proc": n_proc, "ok": ok, "err": err}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags", default="ALL", help="comma list of bag names, or ALL / PILOT")
    ap.add_argument("--objects", default="ALL", help="comma list of object_ids, or ALL")
    ap.add_argument("--gpus", default="0,1,2,3")
    ap.add_argument("--stride", type=int, default=10)
    ap.add_argument("--det_score_thresh", type=float, default=0.2)
    args = ap.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    pf_path = os.path.join(LOG_DIR, "preflight.json")
    pf = json.load(open(pf_path)) if os.path.isfile(pf_path) else None

    if args.bags == "ALL":
        bags = pf["summary"]["bags_ok"] if pf else L.TARGET_BAGS
    elif args.bags == "PILOT":
        bags = [L.PILOT_BAG]
    else:
        bags = [b.strip() for b in args.bags.split(",") if b.strip()]

    if args.objects == "ALL":
        objects = pf["summary"]["objects_ok"] if pf else L.list_objects()
    else:
        objects = [o.strip() for o in args.objects.split(",") if o.strip()]

    gpus = [g.strip() for g in args.gpus.split(",") if g.strip()]
    log(f"\n{'='*60}\n[run] bags={bags}\n[run] objects={len(objects)} gpus={gpus} "
        f"stride={args.stride} det_thresh={args.det_score_thresh}\n{'='*60}")

    summary_rows, error_rows = [], []
    gpu_q = queue.Queue()
    for g in gpus:
        gpu_q.put(g)

    for bag in bags:
        frame_dir, n_frames = ensure_frames(bag, args.stride)
        if frame_dir is None:
            error_rows.append({"bag": bag, "obj": "*", "stage": "extract", "err": "frame extraction failed"})
            continue

        def worker(obj, bag=bag, frame_dir=frame_dir, n_frames=n_frames):
            gpu = gpu_q.get()
            try:
                res = run_job(bag, obj, gpu, frame_dir, args.det_score_thresh, n_frames)
            finally:
                gpu_q.put(gpu)
            tag = "OK" if res["ok"] else "FAIL"
            log(f"  [{tag}] {bag}/{obj} gpu{gpu} pem={res['n_pem']} proc={res['n_proc']}/{n_frames} "
                f"{res['elapsed']}s {res['err']}")
            res["n_frames"] = n_frames
            return res

        log(f"\n[bag] {bag}  ({n_frames} frames x {len(objects)} objects)")
        with ThreadPoolExecutor(max_workers=len(gpus)) as ex:
            for res in ex.map(worker, objects):
                summary_rows.append(res)
                if not res["ok"]:
                    error_rows.append({"bag": res["bag"], "obj": res["obj"],
                                       "stage": "inference", "err": res["err"]})

    with open(os.path.join(LOG_DIR, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["bag", "obj", "gpu", "n_frames", "n_pem", "n_proc",
                                          "elapsed", "ok", "err"])
        w.writeheader()
        w.writerows(summary_rows)
    with open(os.path.join(LOG_DIR, "error.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["bag", "obj", "stage", "err"])
        w.writeheader()
        w.writerows(error_rows)

    n_ok = sum(1 for r in summary_rows if r["ok"])
    log(f"\n{'='*60}\n[run] DONE  jobs={len(summary_rows)} ok={n_ok} fail={len(summary_rows)-n_ok}  "
        f"errors_logged={len(error_rows)}\n[run] logs in {LOG_DIR}\n{'='*60}")


if __name__ == "__main__":
    main()
