"""Lay the YCB-V test sequences out as DualMap Replica-style scenes.

DualMap's Replica loader reads <root>/<scene>/results/frame*.jpg + depth*.png and
<root>/<scene>/traj.txt (one row-major 4x4 camera->world matrix per line). Per scene:
  rgb    -> results/frameNNNNNN.jpg   (PNG re-encoded as JPEG q=95; the loader globs *.jpg)
  depth  -> results/depthNNNNNN.png   (symlink; 0.1 mm units -> png_depth_scale 10000)
  poses  -> traj.txt                  (re-fitted GT trajectory of gt/ycbv/<scene>/cameras.json)
plus a dataset config <root>/configs/<scene>.yaml (K of the scene, 640x480).
The class list (21 fixed prompts in YCB-V obj_id order, then 'unknown') goes to
<root>/ycbv_prompts.txt. DualMap gives FastSAM segments the last class of the list, and all
its shipped lists end with 'unknown', so ours does too (class id 21).

Usage: python dualmap_prep.py [--scenes 48 ...]
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np
from PIL import Image

from common import classes, seq_cameras

ROOT = Path(os.environ.get("DUALMAP_DATA", Path.home() / "DeepLearning/Dataset/bop/ycbv_work/dualmap"))


PREFIX = {"ycbv": "ycbv", "obchange_kitchen": "obkv", "obchange_kitchen_all": "obka", "real": "real"}


def prompts_path(dataset):
    return ROOT / ("real_prompts.txt" if dataset == "real" else "ycbv_prompts.txt")


def scene_id(s, dataset="ycbv"):
    return f"{PREFIX[dataset]}{s:06d}"   # the Replica loader strips '_' from scene ids


def prep_scene(s, dataset="ycbv"):
    """ObChange sequences share one JPEG per global frame id (ROOT/jpg_cache) through symlinks."""
    cams = seq_cameras(dataset, s)
    seq = Path(cams["seq_dir"])
    out = ROOT / "Replica" / scene_id(s, dataset)
    cache = ROOT / "jpg_cache"
    res = out / "results"
    res.mkdir(parents=True, exist_ok=True)
    with open(out / "traj.txt", "w") as fh:
        for k, fr in enumerate(cams["frames"]):
            f = fr["id"]
            jpg = res / f"frame{k:06d}.jpg"
            if not jpg.exists():
                if dataset in ("ycbv", "real"):
                    Image.open(seq / "rgb" / f"{f:06d}.png").convert("RGB").save(jpg, quality=95)
                else:
                    c = cache / f"{f:06d}.jpg"
                    if not c.exists():
                        cache.mkdir(parents=True, exist_ok=True)
                        Image.open(seq / "rgb" / f"{f:06d}.png").convert("RGB").save(c, quality=95)
                    jpg.symlink_to(c)
            dep = res / f"depth{k:06d}.png"
            if not dep.exists():
                dep.symlink_to(seq / "depth" / f"{f:06d}.png")
            fh.write(" ".join(f"{x:.9f}" for x in np.array(fr["T_wc"]).reshape(-1)) + "\n")
    json.dump([fr["id"] for fr in cams["frames"]], open(out / "frame_ids.json", "w"))

    K = cams["K"]
    (ROOT / "configs").mkdir(parents=True, exist_ok=True)
    (ROOT / "configs" / f"{scene_id(s, dataset)}.yaml").write_text(
        "dataset_name: 'replica'\n"
        "camera_params:\n"
        f"  image_height: {cams['height']}\n"
        f"  image_width: {cams['width']}\n"
        f"  fx: {K[0]}\n  fy: {K[4]}\n  cx: {K[2]}\n  cy: {K[5]}\n"
        f"  png_depth_scale: {1000.0 / cams['depth_scale_mm']}\n"
        "  crop_edge: 0\n")
    return len(cams["frames"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", type=int, nargs="*", default=list(range(48, 60)))
    ap.add_argument("--dataset", default="ycbv", help="see common.seq_cameras")
    a = ap.parse_args()
    ROOT.mkdir(parents=True, exist_ok=True)
    prompts_path(a.dataset).write_text("\n".join(list(classes(a.dataset)[0]) + ["unknown"]) + "\n")
    for s in a.scenes:
        print(s, prep_scene(s, a.dataset), "frames ->", ROOT / "Replica" / scene_id(s, a.dataset))


if __name__ == "__main__":
    main()
