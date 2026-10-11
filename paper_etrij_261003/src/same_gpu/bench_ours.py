#!/usr/bin/env python3
"""(B) Our recognizer on THIS GPU: production Sam6DCore.process (YOLO-World text prompts ->
DINOv2 + MobileSAM + sequential gates -> PEM -> pose verification), same setup as
paper_etrij_261003/src/real_stage/run_real_stage.py (production mode, no diagnostics),
restricted to the eight objects. Read-only use of repo code.

    /home/jucpark/anaconda3/envs/sam6d/bin/python bench_ours.py
"""
import json
import os
import sys
import time

import numpy as np
import torch
import yaml

import common as C

SR = C.ROOT / "sam6d_realtime"
RUN = C.ROOT / "objpose/output/live_260915_eightcircle_orbslam3"


def sync():
    torch.cuda.synchronize()


def main():
    frames, K = C.load_frames()
    sys.path.insert(0, str(SR / "realtime")); sys.path.insert(0, str(SR))
    cwd = os.getcwd(); os.chdir(SR)
    from sam6d_core import Sam6DCore
    run_cfg = yaml.safe_load(open(RUN / "sam6d_config.yaml"))
    rt = run_cfg["runtime"]
    np.random.seed(0); torch.manual_seed(0)
    core = Sam6DCore(run_cfg["ism"]["config"], C.OBJECTS, rt["device"], rt["det_score_thresh"],
                     appe_rerank=rt.get("appe_rerank"), verify=rt["verify"])
    os.chdir(cwd)
    assert sorted(o["name"] for o in core.objs) == sorted(C.OBJECTS)

    for i in C.WARMUP:
        bgr, depth, _ = frames[i]
        core.process(bgr, depth, K)
    sync(); torch.cuda.reset_peak_memory_stats()
    per = []
    for i in C.FRAMES:
        bgr, depth, t_ns = frames[i]
        sync(); t0 = time.perf_counter()
        rows, ms, nb, _ = core.process(bgr, depth, K)
        sync(); t1 = time.perf_counter()
        per.append({"frame": i, "t_ns": t_ns, "total_ms": round(1e3 * (t1 - t0), 2),
                    "core_ms": ms, "n_yolo_boxes": int(nb),
                    "n_ism_accepted": len(core.last_frame_diag.get("pem_candidates", [])),
                    "n_output": len(rows),
                    "output_objects": sorted(r.get("object", r.get("name", r.get("cad", "?"))) for r in rows)})
        print(per[-1]["frame"], per[-1]["total_ms"], ms, per[-1]["n_ism_accepted"], per[-1]["n_output"], flush=True)
    res = {
        "pipeline": "B_ours",
        "gpu": torch.cuda.get_device_name(0), "torch": torch.__version__,
        "objects": C.OBJECTS, "frames": C.FRAMES, "warmup_frames": C.WARMUP,
        "total_ms": C.stats([p["total_ms"] for p in per]),
        "stage_ms": {k: C.stats([p["core_ms"][k] for p in per]) for k in ("yolo", "ism", "pem")},
        "n_ism_accepted_per_frame": C.stats([p["n_ism_accepted"] for p in per]),
        "n_output_per_frame": C.stats([p["n_output"] for p in per]),
        "gpu_mem_peak_MiB": {"max_allocated": round(torch.cuda.max_memory_allocated() / 2**20, 1),
                             "max_reserved": round(torch.cuda.max_memory_reserved() / 2**20, 1)},
        "per_frame": per,
    }
    json.dump(res, open(C.OUT / "raw_B_ours.json", "w"), indent=1, default=str)
    print(json.dumps({k: v for k, v in res.items() if k != "per_frame"}, indent=1, default=str))


if __name__ == "__main__":
    main()
