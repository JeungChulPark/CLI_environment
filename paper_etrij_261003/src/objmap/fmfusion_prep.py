"""Lay the YCB-V test sequences out as FM-Fusion (ScanNet-style) scans and write its config.

Per scene, $FMF_DATA/scans/<scene>/:
  color/frame-%06d.png, depth/frame-%06d.png   symlinks to the BOP rgb/depth (frame id = BOP id)
  pose/frame-%06d.txt                          4x4 T_wc (camera->world, metres), the re-fitted
                                               GT trajectory of gt/ycbv/<scene>/cameras.json
  prediction_no_augment/                       filled by fmfusion_detect.py
Also $FMF_DATA/config_ycbv.yaml and $FMF_DATA/likelihood_ycbv.txt (see below), splits/val.txt.

Config = FM-Fusion's config/scannet.yaml (the setting of its ScanNet benchmark) with only
the changes YCB-V needs:
  - camera: YCB-V K, depth_scale 10000 (BOP depth is in 0.1 mm)
  - label set: the 21 YCB-V objects. FM-Fusion's Bayesian label fusion maps open-set
    GroundingDINO phrases to a closed set through a likelihood matrix learned on ScanNet;
    here the open set is the 21 fixed prompts and the closed set the 21 YCB names, so the
    matrix is the identity (no confusion statistics exist for these prompts)
  - scale: voxel_length 0.02 -> 0.01, sdf_trunc 0.04 -> 0.02, shape_min_points 500 -> 50.
    With the room-scale defaults a 2 cm grid holds ~50-300 points per YCB object and the
    500-point export filter drops every one of them

Usage: python fmfusion_prep.py [--scenes 48 ...]
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np

from common import PROMPTS, YCB_NAMES, YCBV, seq_cameras, seq_name

FMF_ROOT = Path(os.environ.get("FMFUSION_ROOT", Path.home() / "DeepLearning/baselines/FM-Fusion"))
FMF_DATA = Path(os.environ.get("FMFUSION_DATA", Path.home() / "DeepLearning/Dataset/bop/ycbv_work/fmfusion"))
SEQ = YCBV / "full" / "test"

CHANGES = {
    "dataset": '"scannet"',
    "camera_fx": None, "camera_fy": None, "camera_cx": None, "camera_cy": None,
    "voxel_length": "0.01", "sdf_trunc": "0.02", "depth_scale": "10000.0",
    "shape_min_points": "50", "bayesian_semantic_likelihood": None,
}


def config_path(dataset):
    """One FM-Fusion config per camera: YCB-V, or ObChange (both ObChange sequence kinds share it)."""
    return FMF_DATA / ("config_ycbv.yaml" if dataset == "ycbv" else "config_obchange.yaml")


def write_config(K, dataset="ycbv", depth_scale_mm=0.1):
    vals = dict(CHANGES, camera_fx=K[0], camera_fy=K[4], camera_cx=K[2], camera_cy=K[5],
                depth_scale=f"{1000.0 / depth_scale_mm:.1f}",
                bayesian_semantic_likelihood=f'"{FMF_DATA / "likelihood_ycbv.txt"}"')
    out = []
    for line in open(FMF_ROOT / "config/scannet.yaml"):
        key = line.split(":")[0].strip()
        if key in vals and ":" in line:
            indent = line[:len(line) - len(line.lstrip())]
            comment = f"  # {dataset} (was" + line.split(":", 1)[1].split("#")[0].rstrip() + ")"
            line = f"{indent}{key}: {vals[key]}{comment}\n"
        out.append(line)
    config_path(dataset).write_text("".join(out))
    with open(FMF_DATA / "likelihood_ycbv.txt", "w") as f:
        f.write("# openset_names closet_names likelihood_matrix\n")
        f.write(",".join(PROMPTS) + "\n" + ",".join(YCB_NAMES) + "\n")
        for row in np.eye(len(PROMPTS)):
            f.write(",".join(f"{x:.6f}" for x in row) + "\n")


def prep_scene(scene, dataset="ycbv"):
    """ObChange sequences keep the global frame ids of the continuous sequence; their detections live once,
    in the scan of the whole sequence (obchange_kitchen_all, scene 6), and every other ObChange scan links
    its prediction_no_augment/ there."""
    cams = seq_cameras(dataset, scene)
    d = FMF_DATA / "scans" / seq_name(dataset, scene)
    shared = FMF_DATA / "scans" / seq_name("obchange_kitchen_all", 6) / "prediction_no_augment"
    for sub in ("color", "depth", "pose"):
        (d / sub).mkdir(parents=True, exist_ok=True)
    pred = d / "prediction_no_augment"
    if dataset == "ycbv" or d.parent / shared.parent.name == d:
        pred.mkdir(exist_ok=True)
    elif not pred.exists():
        shared.mkdir(parents=True, exist_ok=True)
        pred.symlink_to(shared)
    src = SEQ / f"{scene:06d}" if dataset == "ycbv" else Path(cams["seq_dir"])
    for fr in cams["frames"]:
        i = fr["id"]
        name = f"frame-{i:06d}"
        for sub, s in (("color", "rgb"), ("depth", "depth")):
            link = d / sub / f"{name}.png"
            if not link.exists():
                link.symlink_to(src / s / f"{i:06d}.png")
        np.savetxt(d / "pose" / f"{name}.txt", np.array(fr["T_wc"]), fmt="%.8f")
    return cams["K"], len(cams["frames"]), cams["depth_scale_mm"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", type=int, nargs="*", default=list(range(48, 60)))
    ap.add_argument("--dataset", default="ycbv", help="see common.seq_cameras")
    a = ap.parse_args()
    FMF_DATA.mkdir(parents=True, exist_ok=True)
    Ks = set()
    for s in a.scenes:
        K, n, dmm = prep_scene(s, a.dataset)
        Ks.add(tuple(K))
        print(f"{seq_name(a.dataset, s)}: {n} frames")
    assert len(Ks) == 1, "scenes with different K need separate configs"
    write_config(list(Ks.pop()), a.dataset, dmm)
    if a.dataset == "ycbv":
        (FMF_DATA / "splits").mkdir(exist_ok=True)
        (FMF_DATA / "splits/val.txt").write_text("".join(f"{s:06d}\n" for s in a.scenes))
    print("->", FMF_DATA)


if __name__ == "__main__":
    main()
