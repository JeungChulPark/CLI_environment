#!/usr/bin/env python3
"""diag_bop_ism.py — why an object is never accepted: which gate drops it, on real frames.

Runs the recogniser exactly as the live pipeline does (same config, same YOLO pass, same
DINOv2 verification) over sampled frames of a scene and prints, per object, the histogram of
`decision` values and the distribution of the scores the gates compare against. A prompt
problem and a verification problem look identical from the outside — this separates them.

    python objpose/tools/diag_bop_ism.py --config configs/yolo_ism_lmo.yaml \
        --scene /.../lmo/test/000002 --frames 40
"""
from __future__ import annotations

import argparse
import collections
import os
import sys
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[2]
SAM6D = REPO / "sam6d_realtime"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/yolo_ism_lmo.yaml")
    ap.add_argument("--scene", required=True)
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--objects", nargs="*", default=[])
    ap.add_argument("--device", default="cuda:0")
    a = ap.parse_args()
    scene = Path(a.scene).resolve()

    sys.path.insert(0, str(SAM6D))
    os.chdir(SAM6D)
    import torch
    import yolo_ism as yi
    import yolo_ism_object_n as o_n
    import ultralytics.nn.text_model as _tm
    _tm.WEIGHTS_DIR = SAM6D / "assets"
    from ultralytics import YOLOWorld

    defaults, objs = o_n.load_config(str(SAM6D / a.config))
    if a.objects:
        objs = [o for o in objs if o["name"] in set(a.objects)]
    device = a.device            # the ISM helpers want the string form, as sam6d_core passes it
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, rebuild=False)
    segmentor = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    prompts, groups = o_n.build_prompt_groups(objs)
    tsim = o_n.template_similarity(objs)
    yolo = YOLOWorld(o_n._abspath(defaults.get("weights", "yolov8m-worldv2.pt")))
    yolo.set_classes(prompts)
    min_score = min(float(o.get("score_threshold", 0.02)) for o in objs)
    multi = o_n._multi_label_on(objs[0])

    fs = sorted((scene / "rgb").glob("*.png")) or sorted((scene / "rgb").glob("*.jpg"))
    fs = [fs[i] for i in np.linspace(0, len(fs) - 1, a.frames).astype(int)]
    dec = collections.defaultdict(collections.Counter)
    sem = collections.defaultdict(list)
    appe = collections.defaultdict(list)
    for f in fs:
        bgr = cv2.imread(str(f))
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        with o_n.multi_label_nms(multi):
            r = yolo.predict(bgr, conf=min_score, imgsz=int(defaults.get("imgsz", 640)),
                             verbose=False, device=device)
        h, w = bgr.shape[:2]
        pb = {i: [] for i in range(len(prompts))}
        if len(r) and r[0].boxes is not None:
            b = r[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                ci = int(b.cls[j]) if b.cls is not None else 0
                if ci in pb:
                    pb[ci].append(([x1, y1, x2, y2], float(b.conf[j])))
        for k in pb:
            pb[k].sort(key=lambda t: t[1], reverse=True)
        res = o_n.recognize_frame_auto(groups, pb, bgr, rgb, yi.normalize_rgb(rgb),
                                       model, device, segmentor, pool, tsim)
        for name, v in res.items():
            dec[name][v.get("decision", "?")] += 1
            if v.get("best_sem"):
                sem[name].append(float(v["best_sem"]))
            for key in ("masked_appe", "appe"):
                if v.get(key):
                    appe[name].append(float(v[key]))
                    break

    print(f"\n{len(fs)} frames of {scene.name}   sim_thr={objs[0]['similarity_threshold']} "
          f"appe_gate={objs[0].get('appe_v2_gate', objs[0].get('appe_gate'))}\n")
    for o in objs:
        n = o["name"]
        s = np.asarray(sem[n]) if sem[n] else np.zeros(0)
        p = np.asarray(appe[n]) if appe[n] else np.zeros(0)
        print(f"{n:22s} sem p50/p90 {np.median(s) if len(s) else 0:.3f}/"
              f"{np.percentile(s, 90) if len(s) else 0:.3f}  "
              f"appe p50/p90 {np.median(p) if len(p) else 0:.3f}/"
              f"{np.percentile(p, 90) if len(p) else 0:.3f}")
        for d, c in dec[n].most_common():
            print(f"    {c:4d}  {d}")


if __name__ == "__main__":
    main()
