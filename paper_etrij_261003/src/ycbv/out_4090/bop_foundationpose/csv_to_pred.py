#!/usr/bin/env python3
"""BOP CSV (scene_id,im_id,obj_id,score,R,t,time) -> pred_ladder_<name>.json for evaluate.py (our protocol).
Same structure as pred_ladder_gigapose.json: all 900 BOP19 target images are listed (empty preds if no answer);
time_ms per image = CSV time column (s) * 1000.
    python csv_to_pred.py <csv> <out_json> <method> <label> <pipeline> <timing_json>
"""
import csv, json, sys
import numpy as np
csv_path, out_path, method, label, pipeline, timing_path = sys.argv[1:7]
YCBV = "/home/jucpark/Dataset/bop/ycbv"
imgs = {}
for t in json.load(open(f"{YCBV}/test_targets_bop19.json")):
    imgs.setdefault((t["scene_id"], t["im_id"]), {"scene_id": t["scene_id"], "im_id": t["im_id"], "time_ms": 0.0, "preds": []})
for r in csv.DictReader(open(csv_path)):
    k = (int(r["scene_id"]), int(r["im_id"]))
    tm = float(r["time"]) * 1000.0
    imgs[k]["time_ms"] = tm
    imgs[k]["preds"].append({"obj_id": int(r["obj_id"]),
                             "R": np.array([float(v) for v in r["R"].split()]).reshape(3, 3).tolist(),
                             "t_mm": [float(v) for v in r["t"].split()], "score": float(r["score"]), "time_ms": tm})
times = [x["time_ms"] for x in imgs.values() if x["preds"]]
timing = json.load(open(timing_path))
out = {"method": method, "label": label, "gpu": timing["gpu"], "pipeline": pipeline,
       "time_ms": {"median": float(np.median(times)), "p90": float(np.percentile(times, 90)), "mean": float(np.mean(times)),
                   "n": len(times), "note": "CSV time column = sum of GPU-synchronised est.register() wall times of the image's "
                   "target objects; excludes CNOS detection (~0.19 s), image loading and per-object mesh preparation"},
       "timing_json": timing_path, "images": list(imgs.values())}
json.dump(out, open(out_path, "w"))
print(f"{out_path}: {len(out['images'])} images, {sum(len(x['preds']) for x in out['images'])} answers, "
      f"{sum(1 for x in out['images'] if not x['preds'])} images without answers, median {out['time_ms']['median']:.1f} ms")
