#!/usr/bin/env python3
"""build_bop_assets.py — the per-object SAM-6D assets for a BOP dataset.

The realtime pipeline loads two files per object and refuses to start without them
(sam6d_realtime/realtime/sam6d_core.py):

  assets/model_points/<name>.npy    8192 surface points in millimetres
  assets/pem_templates/<name>.pt    {tp, tf} — PEM template points + features of the 42 renders

Both are derived here from the BOP models and the renders made by
objpose/tools/render_ycbv_templates.sh. Point colours (`tc`, used by the verification
stage) are added afterwards by sam6d_realtime/tools/add_template_colors.py, which must
run second because it keys off the saved `tp`.

    python objpose/tools/build_bop_assets.py --table objpose/rt/bop/ycbv_objects.json
    python objpose/tools/build_bop_assets.py --table objpose/rt/bop/lmo_objects.json --objects 1 5
"""
from __future__ import annotations

import argparse
import importlib
import json
import os
import sys
from pathlib import Path

import numpy as np
import trimesh

REPO = Path(__file__).resolve().parents[2]
SAM6D = REPO / "sam6d_realtime"
PEM_DIR = SAM6D / "sam6d_master" / "SAM-6D" / "Pose_Estimation_Model"
N_POINTS = 8192


def model_points(ply: Path, n=N_POINTS) -> np.ndarray:
    """n surface points in millimetres — BOP models are already in mm"""
    mesh = trimesh.load(str(ply), force="mesh")
    pts = trimesh.sample.sample_surface(mesh, n)[0]
    return np.asarray(pts, np.float32)


def load_pem(device):
    os.chdir(PEM_DIR)
    for sub in ("provider", "utils", "model", os.path.join("model", "pointnet2")):
        sys.path.append(str(PEM_DIR / sub))
    sys.path.insert(0, str(PEM_DIR))
    import gorilla
    import run_inference_custom as ric
    cfg = gorilla.Config.fromfile(str(PEM_DIR / "config" / "base.yaml"))
    cfg.model_name = "pose_estimation_model"
    MODEL = importlib.import_module(cfg.model_name)
    model = MODEL.Net(cfg.model).to(device).eval()
    gorilla.solver.load_checkpoint(model=model,
                                   filename=str(PEM_DIR / "checkpoints" / "sam-6d-pem-base.pth"))
    return model, cfg, ric


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--table", default=str(REPO / "objpose" / "rt" / "bop" / "ycbv_objects.json"),
                    help="the dataset's object table (rt/bop/<dataset>_objects.json)")
    ap.add_argument("--objects", type=int, nargs="*", default=[], help="BOP obj_ids (default: all ready)")
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()

    import torch
    device = torch.device(a.device)
    table = json.loads(Path(a.table).read_text())
    MODELS = Path(table["models_dir"])
    rows = table["objects"]
    if a.objects:
        rows = [o for o in rows if o["obj_id"] in set(a.objects)]

    pts_dir = SAM6D / "assets" / "model_points"
    tem_dir = SAM6D / "assets" / "pem_templates"
    pts_dir.mkdir(parents=True, exist_ok=True)
    tem_dir.mkdir(parents=True, exist_ok=True)

    todo = []
    for o in rows:
        tdir = SAM6D / "template" / o["name"] / "templates"
        n_views = len(list(tdir.glob("rgb_*.png"))) if tdir.is_dir() else 0
        if n_views < 42:
            print(f"[skip] {o['name']}: {n_views}/42 renders")
            continue
        todo.append((o, tdir))
    if not todo:
        raise SystemExit("no object has its 42 renders yet")

    # ── model points (cheap, no GPU) ─────────────────────────────────────────
    for o, _ in todo:
        out = pts_dir / f"{o['name']}.npy"
        if out.exists() and not a.force:
            print(f"[keep] {out.name}")
            continue
        ply = MODELS / f"obj_{o['obj_id']:06d}.ply"
        pts = model_points(ply)
        np.save(out, pts)
        ext = pts.max(0) - pts.min(0)
        print(f"[pts ] {o['name']}: {len(pts)} pts, extent {ext[0]:.1f}x{ext[1]:.1f}x{ext[2]:.1f} mm")

    # ── PEM template features ────────────────────────────────────────────────
    need = [(o, t) for o, t in todo if a.force or not (tem_dir / f"{o['name']}.pt").exists()]
    if not need:
        print("[tem ] every template feature already built")
        return
    cwd = os.getcwd()
    model, cfg, ric = load_pem(device)
    try:
        for o, tdir in need:
            all_tem, all_tem_pts, all_tem_choose = ric.get_templates(str(tdir), cfg.test_dataset)
            with torch.inference_mode():
                tp, tf = model.feature_extraction.get_obj_feats(all_tem, all_tem_pts, all_tem_choose)
            torch.save({"tp": tp.cpu(), "tf": tf.cpu(), "tdir": str(tdir)}, tem_dir / f"{o['name']}.pt")
            print(f"[tem ] {o['name']}: tp{tuple(tp.shape)} tf{tuple(tf.shape)}")
    finally:
        os.chdir(cwd)
    print(f"\nnext: python sam6d_realtime/tools/add_template_colors.py --objects "
          f"{','.join(o['name'] for o, _ in todo)}")


if __name__ == "__main__":
    main()
