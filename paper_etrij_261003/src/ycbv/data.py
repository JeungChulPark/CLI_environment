"""BOP YCB-V test list (test_targets_bop19.json, 900 images) loader + prediction file I/O."""
import json
import os
import statistics
import time
from collections import OrderedDict

import cv2
import numpy as np

import paths as P


def load_targets():
    """[(scene_id, im_id, [obj_id, ...]), ...] in file order."""
    t = json.load(open(P.YCBV / "test_targets_bop19.json"))
    by = OrderedDict()
    for e in t:
        by.setdefault((e["scene_id"], e["im_id"]), []).append(int(e["obj_id"]))
    items = [(s, i, sorted(o)) for (s, i), o in by.items()]
    k = int(os.environ.get("YCBV_SUBSET", "0") or 0)      # quick checks: every k-th image only
    return items[::k] if k > 1 else items


_scene_cache = {}


def scene(sid):
    if sid not in _scene_cache:
        d = P.YCBV / "test" / f"{sid:06d}"
        _scene_cache[sid] = {k: json.load(open(d / f"{k}.json"))
                             for k in ("scene_camera", "scene_gt", "scene_gt_info")}
    return _scene_cache[sid]


def load_image(sid, iid):
    """bgr uint8, depth_raw uint16, depth in mm (float32), K (3x3), depth_scale."""
    d = P.YCBV / "test" / f"{sid:06d}"
    bgr = cv2.imread(str(d / "rgb" / f"{iid:06d}.png"), cv2.IMREAD_COLOR)
    raw = cv2.imread(str(d / "depth" / f"{iid:06d}.png"), cv2.IMREAD_UNCHANGED)
    cam = scene(sid)["scene_camera"][str(iid)]
    ds = float(cam["depth_scale"])
    K = np.asarray(cam["cam_K"], np.float64).reshape(3, 3)
    return bgr, raw, raw.astype(np.float32) * ds, K, ds


def gt_instances(sid, iid):
    """[{obj_id, R, t_mm, bbox_obj, bbox_visib, visib_fract, px_count_visib}]"""
    sc = scene(sid)
    out = []
    for g, gi in zip(sc["scene_gt"][str(iid)], sc["scene_gt_info"][str(iid)]):
        out.append({"obj_id": int(g["obj_id"]), "R": np.asarray(g["cam_R_m2c"]).reshape(3, 3),
                    "t_mm": np.asarray(g["cam_t_m2c"], float), "bbox_obj": gi["bbox_obj"],
                    "bbox_visib": gi["bbox_visib"], "visib_fract": gi["visib_fract"],
                    "px_count_visib": gi["px_count_visib"]})
    return out


def stats(xs):
    xs = [float(x) for x in xs]
    if not xs:
        return None
    return {"median": round(statistics.median(xs), 1), "p90": round(float(np.percentile(xs, 90)), 1),
            "mean": round(statistics.fmean(xs), 1), "n": len(xs)}


def header(method, extra=None):
    import torch
    return {"method": method, "label": P.LABEL, "gpu": torch.cuda.get_device_name(0),
            "torch": torch.__version__, "started": time.strftime("%Y-%m-%d %H:%M:%S"),
            "dataset": str(P.YCBV), "targets": "test_targets_bop19.json", **(extra or {})}
