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

from common import GT_DIR, PROMPTS

ROOT = Path(os.environ.get("DUALMAP_DATA", Path.home() / "DeepLearning/Dataset/bop/ycbv_work/dualmap"))


def scene_id(s):
    return f"ycbv{s:06d}"   # the Replica loader strips '_' from scene ids


def prep_scene(s):
    cams = json.load(open(GT_DIR / "ycbv" / f"{s:06d}" / "cameras.json"))
    seq = Path(cams["seq_dir"])
    out = ROOT / "Replica" / scene_id(s)
    res = out / "results"
    res.mkdir(parents=True, exist_ok=True)
    with open(out / "traj.txt", "w") as fh:
        for k, fr in enumerate(cams["frames"]):
            f = fr["id"]
            jpg = res / f"frame{k:06d}.jpg"
            if not jpg.exists():
                Image.open(seq / "rgb" / f"{f:06d}.png").convert("RGB").save(jpg, quality=95)
            dep = res / f"depth{k:06d}.png"
            if not dep.exists():
                dep.symlink_to(seq / "depth" / f"{f:06d}.png")
            fh.write(" ".join(f"{x:.9f}" for x in np.array(fr["T_wc"]).reshape(-1)) + "\n")
    json.dump([fr["id"] for fr in cams["frames"]], open(out / "frame_ids.json", "w"))

    K = cams["K"]
    (ROOT / "configs").mkdir(parents=True, exist_ok=True)
    (ROOT / "configs" / f"{scene_id(s)}.yaml").write_text(
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
    a = ap.parse_args()
    ROOT.mkdir(parents=True, exist_ok=True)
    (ROOT / "ycbv_prompts.txt").write_text("\n".join(PROMPTS + ["unknown"]) + "\n")
    for s in a.scenes:
        print(s, prep_scene(s), "frames ->", ROOT / "Replica" / scene_id(s))


if __name__ == "__main__":
    main()
