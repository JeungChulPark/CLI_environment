#!/usr/bin/env python3
"""Parallel PEM over a bag's manifest, distributing bundles across GPUs.
Usage: python parallel_pem.py <bag> [n_gpus] [workers_per_gpu]
Runs in sam6d_ros_humble env context is NOT required (spawns that python).
"""
import csv, os, sys, subprocess
from concurrent.futures import ThreadPoolExecutor

BAG = sys.argv[1]
NGPU = int(sys.argv[2]) if len(sys.argv) > 2 else 4
WPG = int(sys.argv[3]) if len(sys.argv) > 3 else 2
ROOT = "/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"
PYH = "/home/ldh9501/miniconda3/envs/sam6d_ros_humble/bin/python"
PEM = os.path.join(ROOT, "sam6d_master/SAM-6D/Pose_Estimation_Model")
MAN = os.path.join(ROOT, "outputs/pem_inputs", BAG, "manifest.csv")

rows = []
with open(MAN) as f:
    for r in csv.DictReader(f):
        rows.append(r)
print(f"{len(rows)} bundles, {NGPU} GPUs x {WPG} workers")

done = [0]
def run(i_row):
    i, r = i_row
    fdir = r["frame_dir"]; obj = r["object"]
    outdir = os.path.join(fdir, f"pem_{obj}")
    resj = os.path.join(outdir, "sam6d_results", "detection_pem.json")
    if os.path.isfile(resj):
        done[0] += 1; return  # already done (idempotent)
    gpu = i % NGPU
    env = dict(os.environ, CUDA_VISIBLE_DEVICES=str(gpu))
    for k in ("PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "LD_LIBRARY_PATH"):
        env.pop(k, None)
    cmd = [PYH, "run_inference_custom.py",
           "--output_dir", outdir, "--cad_path", r["cad_path"],
           "--rgb_path", os.path.join(fdir, "rgb.png"),
           "--depth_path", os.path.join(fdir, "depth.png"),
           "--cam_path", os.path.join(fdir, "camera.json"),
           "--seg_path", r["seg_json"].strip(),
           "--template_dir", r["template_dir"].strip(),
           "--config", "config/base.yaml", "--det_score_thresh", "0.2"]
    log = os.path.join(fdir, f"pem_{obj}.log")
    with open(log, "wb") as lf:
        rc = subprocess.run(cmd, cwd=PEM, env=env, stdout=lf, stderr=subprocess.STDOUT).returncode
    done[0] += 1
    tag = f"{os.path.basename(fdir)}:{obj}"
    print(f"[{done[0]}/{len(rows)}] gpu{gpu} {'OK' if rc==0 else 'FAIL'} {tag}", flush=True)

with ThreadPoolExecutor(max_workers=NGPU*WPG) as ex:
    list(ex.map(run, enumerate(rows)))
print("PARALLEL PEM DONE")
