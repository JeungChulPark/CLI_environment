#!/usr/bin/env python3
"""Diagnostic (not part of the protocol): ISM decision per target object, text vs GT boxes.

Runs only the ISM stage of the deployed core (same calls as Sam6DCore.process up to
recognize_frame_auto) and tallies each target object's decision string
(detected / no-object(below-sim|below-appe|below-hsv|no-proposal) / cross-object NMS loss).
For GT boxes, also records which object the GT box of a target was elected to.

    python diag_ism_gates.py [--step 1] -> out/diag_ism_gates.json
"""
import argparse
import collections
import json

import cv2
import numpy as np
import torch

import data as D
import paths as P
import run_ours as RO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=int, default=1)
    a = ap.parse_args()
    core, o_n = RO.build_core()
    import yolo_ism as yi
    items = D.load_targets()[::a.step]
    tally = {m: collections.defaultdict(collections.Counter) for m in ("text", "gtbox")}
    for sid, iid, oids in items:
        bgr, raw, depth_mm, K, ds = D.load_image(sid, iid)
        RO.activate(core, o_n, oids)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = bgr.shape[:2]
        for mode in ("text", "gtbox"):
            pb = {i: [] for i in range(len(core.unique_prompts))}
            if mode == "text":
                with o_n.multi_label_nms(core.multi_label):
                    res = core.yolo.predict(bgr, conf=core.min_score, imgsz=core.imgsz, verbose=False,
                                            device=core.device)
                b = res[0].boxes
                for j in range(len(b) if b is not None else 0):
                    xy = b.xyxy[j].tolist()
                    x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                    x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                    pb[int(b.cls[j])].append(([x1, y1, x2, y2], float(b.conf[j])))
            else:
                for inst in D.gt_instances(sid, iid):
                    if inst["obj_id"] in oids:
                        x, y, bw, bh = inst["bbox_obj"]
                        x1 = max(0, min(int(x), w - 1)); y1 = max(0, min(int(y), h - 1))
                        x2 = max(x1 + 1, min(int(x + bw), w)); y2 = max(y1 + 1, min(int(y + bh), h))
                        pb[core.unique_prompts.index(P.PROMPTS[inst["obj_id"] - 1])].append(([x1, y1, x2, y2], 1.0))
            for k in pb:
                pb[k].sort(key=lambda t: t[1], reverse=True)
            r = o_n.recognize_frame_auto(core.groups, pb, bgr, rgb, yi.normalize_rgb(rgb), core.model,
                                         core.device, core.segmentor, core.pool, core.tsim)
            for o in oids:
                d = r[P.obj_name(o)]
                tally[mode][P.YCB_NAMES[o - 1]][d.get("decision", "?") + (" [accepted]" if d.get("accepted") else "")] += 1
    out = {m: {k: dict(v) for k, v in t.items()} for m, t in tally.items()}
    tot = {m: dict(sum((collections.Counter(v) for v in t.values()), collections.Counter())) for m, t in out.items()}
    json.dump({"label": P.LABEL, "images": len(items), "total": tot, "per_object": out},
              open(P.OUT / "diag_ism_gates.json", "w"), indent=1)
    print(json.dumps(tot, indent=1))


if __name__ == "__main__":
    main()
