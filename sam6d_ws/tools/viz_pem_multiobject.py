#!/usr/bin/env python3
"""viz_pem_multiobject.py — per-frame UNIFIED multi-object 6D pose overlay.

PEM's own vis_pem.png is per-object (one CAD per image). This composites EVERY
object's estimated 6D pose for a frame onto a single RGB image: each object's
CAD is reprojected (3D bbox + sampled points) via its (R,t) and camera K, in a
distinct color, with a legend (object, pose_score).

Reads outputs/pem_inputs/<bag>/frame_*/pem_<obj>/sam6d_results/detection_pem.json
and the frame's rgb.png + camera.json + manifest (object->CAD). Writes
  outputs/pem_inputs/<bag>/frame_<ci>/vis_pem_all.png
Run in the sam6d_ros_humble env (uses PEM draw_utils).
"""
import argparse
import csv
import glob
import json
import os
import sys

import cv2
import numpy as np
import trimesh
from PIL import Image

REPO = "/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"
PEM = os.path.join(REPO, "sam6d_master", "SAM-6D", "Pose_Estimation_Model")
sys.path.append(os.path.join(PEM, "utils"))
from draw_utils import draw_detections   # noqa: E402

# distinct BGR colors per object (draw_detections takes color as given)
COLORS = [(0, 0, 255), (0, 200, 0), (255, 0, 0), (0, 200, 255),
          (255, 0, 255), (255, 200, 0), (128, 0, 255), (0, 128, 255)]


def parse_args():
    p = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--bags", nargs="+", default=["two_table_diagonal1"])
    p.add_argument("--n-points", type=int, default=1024, help="CAD points to reproject")
    return p.parse_args()


def load_cad_map(bag):
    """object -> cad_path from manifest."""
    m = {}
    with open(os.path.join(REPO, "outputs", "pem_inputs", bag, "manifest.csv"), newline="") as f:
        for r in csv.DictReader(f):
            m[r["object"].strip()] = r["cad_path"].strip()
    return m


def main():
    args = parse_args()
    mp_cache = {}   # cad_path -> sampled model points (mm)
    for bag in args.bags:
        cad_map = load_cad_map(bag)
        bag_dir = os.path.join(REPO, "outputs", "pem_inputs", bag)
        frames = sorted(glob.glob(os.path.join(bag_dir, "frame_*")))
        color_idx = {o: COLORS[i % len(COLORS)] for i, o in enumerate(sorted(cad_map))}
        n_done = 0
        for fdir in frames:
            rgb_p = os.path.join(fdir, "rgb.png")
            cam_p = os.path.join(fdir, "camera.json")
            if not (os.path.isfile(rgb_p) and os.path.isfile(cam_p)):
                continue
            bgr = cv2.imread(rgb_p)
            img = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            K = np.array(json.load(open(cam_p))["cam_K"]).reshape(3, 3)
            drawn = []
            for det_p in sorted(glob.glob(os.path.join(fdir, "pem_*", "sam6d_results", "detection_pem.json"))):
                # .../frame_xxx/pem_<obj>/sam6d_results/detection_pem.json
                obj = os.path.basename(os.path.dirname(os.path.dirname(det_p)))[len("pem_"):]
                d = json.load(open(det_p))[0]
                R = np.array(d["R"]).reshape(3, 3)
                t = np.array(d["t"]).reshape(3)
                cad = cad_map.get(obj)
                if cad is None:
                    continue
                if cad not in mp_cache:
                    mp_cache[cad] = trimesh.load_mesh(cad).sample(args.n_points).astype(np.float32)
                mp = mp_cache[cad]   # mm
                # draw_detections indexes intrinsics per instance -> needs (N,3,3)
                img = draw_detections(img, R[None, ...], t[None, ...], mp, K[None, ...],
                                      color=color_idx[obj])
                drawn.append((obj, float(d["score"])))
            # legend
            out = img.copy()
            y = 16
            for obj, sc in drawn:
                c = color_idx[obj]
                cv2.rectangle(out, (6, y - 10), (20, y), c, -1)
                cv2.putText(out, f"{obj} {sc:.2f}", (24, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (255, 255, 255), 1, cv2.LINE_AA)
                y += 18
            save = os.path.join(fdir, "vis_pem_all.png")
            Image.fromarray(np.uint8(out)).save(save)
            n_done += 1
        print(f"[{bag}] wrote {n_done} unified overlays (objects: {sorted(cad_map)})")


if __name__ == "__main__":
    main()
