"""Run FM-Fusion's IntegrateInstanceMap on the prepared YCB-V scans and export its final
instances to the common map format (out/fmfusion/ycbv/<scene>/).

Each saved instance (<id>.ply + a line in instance_info.txt) becomes one object:
label = its fused class (a YCB name), points = the instance cloud (world frame, metres).
FM-Fusion's own size filter (shape_min_points) is applied by FM-Fusion before saving.

Usage: python fmfusion_export.py [--scenes 48 ...] [--skip-run]
"""
import argparse
import json
import subprocess
import time

import numpy as np

from common import OUT, save_map, seq_name
from fmfusion_prep import FMF_DATA, FMF_ROOT, config_path

EXE = FMF_ROOT / "build/src/IntegrateInstanceMap"
RAW = FMF_DATA / "output"


def read_ply_xyz(path):
    with open(path, "rb") as f:
        props, n, fmt = [], 0, None
        while True:
            line = f.readline().decode("ascii").strip()
            if line.startswith("format"):
                fmt = line.split()[1]
            elif line.startswith("element vertex"):
                n = int(line.split()[2])
            elif line.startswith("property"):
                props.append(line.split())
            elif line == "end_header":
                break
        if fmt == "ascii":
            return np.loadtxt(f, max_rows=n)[:, :3]
        tmap = {"float": "f4", "double": "f8", "uchar": "u1", "float32": "f4", "float64": "f8", "uint8": "u1"}
        dt = np.dtype([(p[2], "<" + tmap[p[1]]) for p in props])
        v = np.frombuffer(f.read(dt.itemsize * n), dtype=dt)
        return np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float64)


def run(scene, dataset="ycbv"):
    scan = FMF_DATA / "scans" / seq_name(dataset, scene)
    cfg = scan / "config.yaml" if dataset == "real" else config_path(dataset)
    cmd = [str(EXE), "--config", str(cfg),
           "--root", str(FMF_DATA / "scans" / seq_name(dataset, scene)), "--output", str(RAW),
           "--prediction", "prediction_no_augment", "--frame_gap", "2", "--max_frames", "20000", "--verbose", "2"]
    t0 = time.time()
    log = open(RAW / f"{seq_name(dataset, scene)}.log", "w")
    subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, check=True)
    return time.time() - t0


def export(scene, dataset="ycbv"):
    d = RAW / seq_name(dataset, scene)
    objs = []
    for line in open(d / "instance_info.txt"):
        if line.startswith("#") or not line.strip():
            continue
        f = line.split(";")
        label, score = f[1].rsplit("(", 1)
        objs.append({"label": label, "points": read_ply_xyz(d / f"{f[0]}.ply"),
                     "score": float(score.rstrip(")")), "n_obs": int(f[2])})
    save_map(OUT / "fmfusion" / dataset / f"{scene:06d}", objs,
             {"method": "FM-Fusion", "dataset": dataset, "scene": scene})
    return len(objs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", type=int, nargs="*", default=list(range(48, 60)))
    ap.add_argument("--skip-run", action="store_true")
    ap.add_argument("--dataset", default="ycbv", help="see common.seq_cameras")
    a = ap.parse_args()
    RAW.mkdir(parents=True, exist_ok=True)
    log = RAW / "map_times.json"
    rec = json.load(open(log)) if log.exists() else {}
    for s in a.scenes:
        if not a.skip_run:
            t = run(s, a.dataset)
            rec[seq_name(a.dataset, s)] = {"seconds": t}
            json.dump(rec, open(log, "w"), indent=1)
        n = export(s, a.dataset)
        k = seq_name(a.dataset, s)
        print(f"{k}: {n} instances" + ("" if a.skip_run else f", mapping {rec[k]['seconds']:.1f} s"))


if __name__ == "__main__":
    main()
