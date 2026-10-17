"""object_extents.py — per-object 3D box size for drawing, from the model SAM-6D actually uses (2026-09-29).

The box the viewer draws must be the same object SAM-6D fits. SAM-6D's PEM uses the model points
`sam6d_realtime/assets/model_points/<name>.npy` (mm, object frame), so the box is computed from that file:
adding an object needs only its model points (which SAM-6D requires anyway) — nothing else to edit.
`integration/cad_extents.json` is kept as the fallback for objects without model points (e.g. the BOP lmo set).

Why: on 2026-08-18 the model points of choco_hazelnut_high (x0.81), Bear (x1.16), Dinosaur (x1.25) and milk
(height 195 -> 230 mm) were rescaled to the real objects, but cad_extents.json kept the old CAD sizes, so the
choco box was drawn ~23 % too large per side (depth on 260901_cbnu_longcircle: front face ~188 x 125 mm).
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
MODEL_POINTS = REPO / "sam6d_realtime" / "assets" / "model_points"
CAD_EXTENTS = REPO / "integration" / "cad_extents.json"


def extent_from_points(pts_mm: np.ndarray) -> dict:
    mn, mx = pts_mm.min(0), pts_mm.max(0)
    return {"min_mm": [round(float(v), 2) for v in mn], "max_mm": [round(float(v), 2) for v in mx],
            "size_mm": [round(float(v), 2) for v in mx - mn]}


def load_object_extents(model_dir: Path = MODEL_POINTS, fallback: Path = CAD_EXTENTS, log=None) -> dict:
    """{name: {"min_mm", "max_mm", "size_mm", "source"}} — model points first, cad_extents.json for the rest."""
    ext = {}
    if fallback.is_file():
        for k, v in json.loads(fallback.read_text()).items():
            ext[k] = {**v, "source": f"{fallback.name} (no model points)"}
    for p in sorted(model_dir.glob("*.npy")):          # <name>.npy only; backups (.npy.orig, ...) do not match
        old = ext.get(p.stem)
        e = extent_from_points(np.load(p).astype(np.float64))
        e["source"] = f"model_points/{p.name}"
        if log and old and "size_mm" in old:
            r = np.asarray(e["size_mm"]) / np.maximum(np.asarray(old["size_mm"], float), 1e-6)
            if np.any(np.abs(r - 1) > 0.02):
                log(f"[extents] {p.stem}: box from model points {e['size_mm']} mm (cad_extents.json had {old['size_mm']})")
        ext[p.stem] = e
    return ext
