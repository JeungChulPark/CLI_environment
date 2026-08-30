#!/usr/bin/env python3
"""pipeline_lib.py — shared paths / object & bag discovery for the E2E pipeline."""
import glob
import os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
TEMPLATE_DIR = os.path.join(ROOT, "template")
CAD_DIR = os.path.join(ROOT, "data", "cad")
BAG_DIR = os.path.join(ROOT, "data", "ros2_bag")
ENGINE = os.path.join(ROOT, "sam6d_master", "SAM-6D", "run_batch_inference_fast.py")
OUT = os.path.join(ROOT, "outputs_e2e")

# Target bags (spec). Auto-discovered under data/ros2_bag.
TARGET_BAGS = [
    "high_texture_around", "high_texture_far_close",
    "low_texture_around", "low_texture_far_close",
    "two_table_around", "two_table_around_goback",
    "two_table_diagonal1", "two_table_diagonal2", "two_table_goback",
]
PILOT_BAG = "two_table_around"

# template folder -> data/cad subfolder, when names differ.
CAD_NAME_MAP = {"Milk_scaled_195mm": "milk"}


def list_objects():
    """All template object_ids (folders under template/, excluding _logs/hidden)."""
    out = []
    for name in sorted(os.listdir(TEMPLATE_DIR)):
        p = os.path.join(TEMPLATE_DIR, name)
        if name.startswith(("_", ".")) or not os.path.isdir(p):
            continue
        out.append(name)
    return out


def template_path(obj):
    return os.path.join(TEMPLATE_DIR, obj, "templates")


def resolve_cad(obj):
    """Return the single .ply path for an object, or None if absent."""
    cad_sub = CAD_NAME_MAP.get(obj, obj)
    folder = os.path.join(CAD_DIR, cad_sub)
    if not os.path.isdir(folder):
        return None
    plys = sorted(glob.glob(os.path.join(folder, "*.ply")))
    return plys[0] if plys else None


def template_assets_ok(obj):
    """Check templates/ has rgb/mask/xyz views. Returns (ok, detail)."""
    td = template_path(obj)
    if not os.path.isdir(td):
        return False, "no templates/ dir"
    rgb = len(glob.glob(os.path.join(td, "rgb_*.png")))
    mask = len(glob.glob(os.path.join(td, "mask_*.png")))
    xyz = len(glob.glob(os.path.join(td, "xyz_*.npy")))
    if rgb == 0 or mask == 0 or xyz == 0:
        return False, f"incomplete views rgb={rgb} mask={mask} xyz={xyz}"
    return True, f"rgb={rgb} mask={mask} xyz={xyz}"


def resolve_bag_path(bag):
    """Return the .db3/.mcap path for a bag dir name (handles nested bag/)."""
    base = os.path.join(BAG_DIR, bag)
    cands = (glob.glob(os.path.join(base, "**", "*.db3"), recursive=True)
             + glob.glob(os.path.join(base, "**", "*.mcap"), recursive=True))
    return sorted(cands)[0] if cands else None
