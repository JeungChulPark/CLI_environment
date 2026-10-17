#!/usr/bin/env python3
"""run_profiled.py — run_compare.py with per-module timers (same arguments, same outputs).

Nothing of the estimators is changed: the inner stages are wrapped with timers (CUDA synchronised
at entry and exit) and every frame's `ms` gets a `sub` dict of EXCLUSIVE times in ms:

  proposed   seg          MobileSAM masks of the candidate boxes
             dino         DINOv2 descriptors of the candidates (candidate verification)
             pem_prep     point cloud of the frame + per-object crops / point sampling
             pem_feat     PEM feature extraction (ViT)
             pem_coarse   PEM coarse matching, without the candidate verification inside it
             verify_cand  pose verification over the coarse candidates (mask -> texture -> convergence)
             pem_fine     PEM fine matching
             verify_fine  check of the refined pose
  orig       seg          FastSAM segment-everything
             dino         DINOv2 descriptors of every proposal
             pem_feat / pem_coarse / pem_fine

What is left of `ism` and `pem` after these is matching / scoring / bookkeeping (the report calls it "기타").

    python objpose/tools/baselines/run_profiled.py --method proposed --dataset ycbv --mode still --out <dir>
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_compare as RC          # noqa: E402

ACC: dict[str, float] = {}
_STACK: list[list] = []           # [key, t_enter, child_seconds]


def _sync():
    if torch.cuda.is_available():
        torch.cuda.synchronize()


def timed(key, fn):
    def w(*a, **k):
        _sync(); fr = [key, time.perf_counter(), 0.0]; _STACK.append(fr)
        try:
            return fn(*a, **k)
        finally:
            _sync(); el = time.perf_counter() - fr[1]; _STACK.pop()
            ACC[key] = ACC.get(key, 0.0) + el - fr[2]
            if _STACK:
                _STACK[-1][2] += el
    return w


def _wrap_attr(obj, name, key):
    if hasattr(obj, name):
        setattr(obj, name, timed(key, getattr(obj, name)))


def _wrap_pem(pem):
    for name, key in (("feature_extraction", "pem_feat"), ("coarse_point_matching", "pem_coarse"), ("fine_point_matching", "pem_fine")):
        m = getattr(pem, name)
        m.forward = timed(key, m.forward)


def instrument(core, method):
    if method.startswith("orig"):
        if getattr(core, "ism", None) is not None:
            _wrap_attr(core.ism.segmentor_model, "generate_masks", "seg")
            _wrap_attr(core.ism.descriptor_model, "forward", "dino")
        if getattr(core, "mobilesam", None) is not None:
            core.mobilesam = timed("seg", core.mobilesam)
        _wrap_pem(core.pem)
    else:
        import yolo_ism as yi
        import yolo_ism_object_n as o_n
        for n in ("segment_box", "segment_boxes"):
            _wrap_attr(yi, n, "seg")
        for n in ("dinov2_forward", "dinov2_forward_batch"):
            _wrap_attr(yi, n, "dino")
        _wrap_attr(o_n, "dinov2_blocks_forward", "dino")
        for n in ("prepare_frame", "get_instance_data"):
            _wrap_attr(core.ric, n, "pem_prep")
        _wrap_pem(core.pem)
        net_mod = sys.modules[type(core.pem).__module__]
        _wrap_attr(net_mod, "validate_refined_poses", "verify_fine")
        cm = sys.modules[type(core.pem.coarse_point_matching).__module__]
        mu = sys.modules[cm.compute_coarse_Rt.__module__]
        _wrap_attr(mu, "independent_candidate_verify", "verify_cand")
    proc = core.process

    def process(*a, **k):
        ACC.clear(); _STACK.clear()
        rows, ms, n_boxes, lab = proc(*a, **k)
        ms = dict(ms, sub={kk: round(v * 1e3, 1) for kk, v in ACC.items()})
        return rows, ms, n_boxes, lab
    core.process = process


_build = RC.build


def build(method, *a, **k):
    core, used = _build(method, *a, **k)
    instrument(core, method)
    return core, used


RC.build = build

if __name__ == "__main__":
    RC.main()
