"""Original SAM-6D as the recogniser of the live system (baseline for the live comparison).

sam6d_infer.py loads this class instead of Sam6DCore when OBJPOSE_RECOGNIZER points at this
file. Everything else in the live system (frame feed, SLAM, object map, display, logs) is the
same, so the two recognisers are compared on the same video, the same GPU and the same map.

The frame pipeline is run_orig.run_frame unchanged (the single-image YCB-V baseline):
FastSAM-x -> DINOv2 ViT-L/14 -> upstream semantic / appearance / geometric score -> NMS per
object -> top-1 per object -> PEM on every top-1 with ISM score > 0.2. As in our recogniser
in the live runs, the query set is all objects of the object list (21 YCB-V objects); the
live system does not know which objects are in the scene.

Supplementary settings (environment):
  OBJPOSE_ORIG_THRESH  ISM score threshold for running PEM / answering (upstream default 0.2)
  OBJPOSE_ORIG_OIDS    comma-separated obj ids to query (e.g. the scene's objects) instead of all 21
  OBJPOSE_ORIG_DESC    descriptor model (upstream dinov2_vitl14; dinov2_vits14 = ladder step L1)
"""
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ycbv"))
import run_orig as RO  # noqa: E402


class LiveCore:
    def __init__(self, cfg):
        np.random.seed(0); RO.torch.manual_seed(0)
        self.desc = os.environ.get("OBJPOSE_ORIG_DESC", "dinov2_vitl14")
        self.ism, bank = RO.build_ism(model_name=self.desc)
        self.net, self.tcfg, self.pobj = RO.build_pem()
        self.thresh = float(os.environ.get("OBJPOSE_ORIG_THRESH", 0.2))
        ids = os.environ.get("OBJPOSE_ORIG_OIDS")
        self.oids = [int(x) for x in ids.split(",")] if ids else list(RO.OIDS)
        RO.select_bank(self.ism, bank, self.oids, {})
        self.objs = [{"name": f"ycbv_{o:02d}"} for o in self.oids]
        self.verify = {}
        self.last_frame_diag = {"pem_candidates": [], "rejections": [], "pem_error": None}
        self.extra_proposals = None
        print(f"[orig] original SAM-6D ready, {len(self.objs)} objects, ISM threshold {self.thresh}, {self.desc}", flush=True)

    def process(self, bgr, depth, K, want_mask=False, diagnostic_references=None, slam_context=None):
        self.extra_proposals = None
        raw = np.asarray(depth).astype(np.int32)            # depth in mm -> depth_scale 1.0
        T, out = RO.run_frame(self.ism, self.net, self.tcfg, self.pobj, self.oids, bgr, raw,
                              np.asarray(depth, np.float32), np.asarray(K, np.float64), 1.0,
                              det_thresh=self.thresh)
        rows = [{"object": f"ycbv_{p['obj_id']:02d}", "score": round(float(p["score"]), 4),
                 "R": [[round(float(v), 6) for v in r] for r in p["R"]],
                 "t_mm": [round(float(v), 4) for v in p["t_mm"]], "ism_score": p["ism_score"],
                 "pose_source": "sam6d_original"} for p in out["preds"]]
        self.last_frame_diag = {"pem_candidates": [], "rejections": [], "pem_error": None,
                                "n_fastsam": out["n_fastsam"], "ism_top1": out["ism_top1"]}
        ms = {"yolo": round(T.get("proposals", 0)), "ism": round(T.get("descriptors", 0) + T.get("matching", 0)),
              "pem": round(T.get("pem", 0)), "total": round(T["total"])}
        return rows, ms, out["n_fastsam"], None
