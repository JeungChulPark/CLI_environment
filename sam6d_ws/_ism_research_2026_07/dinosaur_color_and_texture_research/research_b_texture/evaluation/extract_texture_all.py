#!/usr/bin/env python3
"""extract_texture_all.py — P2 texture (block11 x42-template max) for every HSV-rejected
accepted candidate (rescue set, all objects) + every choco accepted candidate (human-GT
revalidation). READ-ONLY, GPU. Human GT = shadow_decisions 'visible' (frame-level). Object-
agnostic algorithm (same for all). Output: results/texture_all_candidates.csv
"""
import csv, os, sys
from collections import defaultdict
from pathlib import Path
import numpy as np
import cv2
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
RB = os.path.dirname(HERE); ROOT = os.path.dirname(RB)
RSRCH = os.path.dirname(ROOT); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
import yolo_ism as yi                # noqa
import yolo_ism_object_n as o_n      # noqa

CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
DEC = os.path.join(RSRCH, "phase1b_validation", "shadow_decisions.csv")
OUT = os.path.join(RB, "results"); os.makedirs(OUT, exist_ok=True)

rows = list(csv.DictReader(open(DEC)))
# candidate set: HSV-rejected accepted (rescue) + all choco accepted (revalidation)
cand = [r for r in rows if r["b1_accept"] == "1" and
        (r["would_hsv_reject"] == "1" or r["object"] == "choco_hazelnut_high")]
print(f"candidates to score: {len(cand)}")

defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
device = "cuda:0" if torch.cuda.is_available() else "cpu"
model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
TPL = {}
for o in objs:
    tpb = o_n.build_template_appe_blocks(o["template_dir"], model, device,
                                         o["appe_cache"].replace("_appe.pt", "_appe_b11.pt"), [11], False)
    TPL[o["name"]] = [t.to(device) for t in tpb[11]]


def p2(q, tl):
    if q.shape[0] == 0:
        return 0.0
    per = [(q @ tp.T).max(1).values for tp in tl]
    return float(torch.stack(per, 1).max(1).values.mean().clamp(0, 1))


by = defaultdict(list)
for r in cand:
    by[r["dataset"]].append(r)


def read_frames(ds, want):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    ts = get_typestore(Stores.ROS2_HUMBLE)
    hi = max(want); out = {}
    with AnyReader([Path(os.path.join(CONV, ds))], default_typestore=ts) as rd:
        conns = [c for c in rd.connections if c.topic == "/camera/camera/color/image_raw"]
        i = -1
        for conn, t, raw in rd.messages(connections=conns):
            i += 1
            if i > hi:
                break
            if i not in want:
                continue
            m = rd.deserialize(raw, conn.msgtype)
            b = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
            out[i] = cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else b.copy()
    return out


res = []
for ds, items in by.items():
    fr = read_frames(ds, set(int(r["frame_id"]) for r in items))
    for r in items:
        bgr = fr.get(int(r["frame_id"]))
        if bgr is None:
            continue
        box = [int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"])]
        crop = yi.crop_resize_pad(yi.normalize_rgb(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)), box)
        if crop is None:
            continue
        _, pb = o_n.dinov2_blocks_forward(model, [crop], device, [11])
        mask = yi.segment_box(seg, bgr, box, device)
        q = (yi.masked_query_patches(pb[11][0].cpu(), mask, box, pool)[0].to(device)
             if mask is not None else pb[11][0].to(device))
        res.append(dict(object=r["object"], dataset=ds, frame=r["frame_id"], visible=r["visible"],
                        hsv_score=r["hsv_score"], would_hsv_reject=r["would_hsv_reject"],
                        texture_p2=round(p2(q, TPL[r["object"]]), 5)))

with open(os.path.join(OUT, "texture_all_candidates.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(res[0].keys())); w.writeheader(); w.writerows(res)
print(f"scored {len(res)} -> {OUT}/texture_all_candidates.csv")
