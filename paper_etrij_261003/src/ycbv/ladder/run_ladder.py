#!/usr/bin/env python3
"""Ablation ladder from the original SAM-6D to our recogniser, one change per step (YCB-V, 900 images).

  L0  original SAM-6D (run_orig.py as is): FastSAM-x proposals, DINOv2 ViT-L/14, upstream score,
      top-1 per object, PEM if ISM score > 0.2
  L1  L0 with the descriptor DINOv2 ViT-L/14 -> ViT-S/14 (the model our recogniser uses)
  L2  L1 with the proposals FastSAM-x -> YOLO-World text boxes (the deployed yolov8m-worldv2,
      paper prompts, top-3 per prompt, score >= 0.02) + MobileSAM box masks; scoring unchanged
  -- the rest of the ladder runs our recogniser (run_ours.py):
  L3  our ISM (semantic / appearance / HSV gates, box owner election) + PEM, pose verification off
      (run_ours.py --mode text --no-verify --tag ladder_L3)
  L4  our recogniser as deployed (run_ours.py --mode text --tag ladder_L4)
  L5  L4 + re-selected prompts (run_ours.py --mode text --proposer text:yolov8m-worldv2.pt:<best> --tag ladder_L5)

This script runs L0-L2; everything else (protocol, timing region, warm-up, output format) is
run_orig.py's, so L0 here is the original SAM-6D run again in the same session.

    python ladder/run_ladder.py --level L0|L1|L2 [--thresh 0.2]   (option B: --level L1 --thresh 0.4|0.5)
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / "det_study"))
import data as D  # noqa: E402
import paths as P  # noqa: E402
import run_orig as RO  # noqa: E402


class YoloMobileSAM:
    """FastSAM stand-in: YOLO-World text boxes of the image's targets -> MobileSAM masks."""

    def __init__(self):
        import proposers
        sys.path.insert(0, str(P.SR))
        import yolo_ism as yi
        self.yi = yi
        self.prop = proposers.TextProposer("yolov8m-worldv2.pt")
        self.seg = yi.build_segmentor(str(P.SR / "mobile_sam.pt"), "cuda:0")
        self.n_boxes = 0

    def activate(self, oids):
        self.prop.activate(oids)

    @torch.no_grad()
    def generate_masks(self, image_rgb):
        bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        dets = self.prop.predict(bgr, conf=0.02)
        top = {}
        for b, o, s in sorted(dets, key=lambda d: -d[2]):
            if len(top.setdefault(o, [])) < 3:
                top[o].append(b)
        boxes = []
        for bl in top.values():
            for b in bl:
                if all(self._iou(b, u) < 0.95 for u in boxes):
                    boxes.append(b)
        self.n_boxes = len(boxes)
        if not boxes:
            return None
        h, w = bgr.shape[:2]
        boxes = [[max(0, int(b[0])), max(0, int(b[1])), min(w, int(b[2])), min(h, int(b[3]))] for b in boxes]
        masks = self.yi.segment_boxes(self.seg, bgr, boxes, "cuda:0")
        keep = [(b, m) for b, m in zip(boxes, masks) if m is not None and m.any()]
        if not keep:
            return None
        if len(keep) == 1:
            # the upstream descriptor squeezes a single proposal to 2-D and fails; FastSAM never
            # yields one. A duplicate is removed again by the per-object NMS (IoU 1.0).
            keep = keep * 2
        return {"masks": torch.from_numpy(np.stack([m for _, m in keep])).float().to(RO.DEV),
                "boxes": torch.tensor([b for b, _ in keep], dtype=torch.float32, device=RO.DEV)}

    @staticmethod
    def _iou(a, b):
        ix = max(0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
        i = ix * iy; u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i
        return i / u if u > 0 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--level", choices=["L0", "L1", "L2"], required=True)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--thresh", type=float, default=0.2, help="ISM score threshold for PEM (upstream 0.2)")
    a = ap.parse_args()
    tag = a.level + (f"_th{int(round(a.thresh * 10)):02d}" if abs(a.thresh - 0.2) > 1e-9 else "")
    out_path = P.OUT / f"pred_ladder_{tag}.json"
    items = D.load_targets()
    if a.limit:
        items = items[:a.limit]
    np.random.seed(0); torch.manual_seed(0)
    model = "dinov2_vitl14" if a.level == "L0" else "dinov2_vits14"
    seg = YoloMobileSAM() if a.level == "L2" else None
    ism, bank = RO.build_ism(model_name=model, segmentor=seg)
    net, tcfg, pobj = RO.build_pem()
    cache = {}

    def run(sid, iid, oids):
        bgr, raw, depth_mm, K, ds = D.load_image(sid, iid)
        RO.select_bank(ism, bank, oids, cache)
        if seg is not None:
            seg.activate(oids)            # set_classes outside the timed region, as in run_ours.py
        return RO.run_frame(ism, net, tcfg, pobj, oids, bgr, raw, depth_mm, K, ds, det_thresh=a.thresh)

    for sid, iid, oids in items[:a.warmup]:
        run(sid, iid, oids)
    res = D.header(f"ladder_{tag}", {
        "pipeline": {"L0": "original SAM-6D (FastSAM-x, ViT-L)", "L1": "original SAM-6D with ViT-S",
                     "L2": "original SAM-6D scoring with ViT-S on YOLO-World + MobileSAM proposals"}[a.level],
        "descriptor": model, "proposals": "YOLO-World text + MobileSAM" if seg else "FastSAM-x",
        "det_score_thresh": a.thresh, "warmup_images": a.warmup})
    res["images"] = []
    for n, (sid, iid, oids) in enumerate(items):
        T, o = run(sid, iid, oids)
        for p in o["preds"]:
            p["time_ms"] = round(T["total"], 2)
        res["images"].append({"scene_id": sid, "im_id": iid, "targets": oids, "time_ms": round(T["total"], 2),
                              "stage_ms": {k: round(v, 2) for k, v in T.items()}, **o})
        if n % 50 == 0 or n == len(items) - 1:
            print(f"[{n + 1}/{len(items)}] {sid}/{iid} {T['total']:.0f} ms proposals {o['n_fastsam']} "
                  f"top1 {len(o['ism_top1'])} out {len(o['preds'])}", flush=True)
    res["time_ms"] = D.stats([x["time_ms"] for x in res["images"]])
    res["stage_ms"] = {k: D.stats([x["stage_ms"].get(k, 0.0) for x in res["images"]])
                       for k in ("proposals", "descriptors", "matching", "pem")}
    res["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
    json.dump(res, open(out_path, "w"), indent=1, default=float)
    print("->", out_path, res["time_ms"])


if __name__ == "__main__":
    main()
