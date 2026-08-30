#!/usr/bin/env python3
"""eval_dino_rescue.py — SECONDARY: can the same texture method (P2 block11 all-templates)
rescue the 22 HSV-rejected real Dinosaur? READ-ONLY, GPU. Same algorithm/params as Research B
(object-agnostic). Compares texture score of HSV-passed vs HSV-rejected real Dino.
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
import yolo_ism as yi                # noqa: E402
import yolo_ism_object_n as o_n      # noqa: E402

CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
DEC = os.path.join(RSRCH, "phase1b_validation", "shadow_decisions.csv")
OUT = os.path.join(RB, "results"); os.makedirs(OUT, exist_ok=True)

# real Dino candidates: passed vs would_reject (both visible TP in B1)
items = []
for r in csv.DictReader(open(DEC)):
    if r["object"] != "Dinosaur" or r["visible"] != "1" or r["b1_accept"] != "1":
        continue
    box = (int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"]))
    items.append((r["dataset"], int(r["frame_id"]), box, int(r["would_hsv_reject"])))
print(f"real Dino B1-TP: {len(items)} (would_reject={sum(i[3] for i in items)})")

defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
device = "cuda:0" if torch.cuda.is_available() else "cpu"
model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
dino = [o for o in objs if o["name"] == "Dinosaur"][0]
dino["tcls"], _ = yi.build_template_cls(dino["template_dir"], model, device, dino["cls_cache"], False)
tpb = o_n.build_template_appe_blocks(dino["template_dir"], model, device,
                                     dino["appe_cache"].replace("_appe.pt", "_appe_b2-11.pt"), [2, 11], False)
seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
TPL11 = [t.to(device) for t in tpb[11]]


def p2_score(q):
    if q.shape[0] == 0:
        return 0.0
    per = [(q @ tp.T).max(1).values for tp in TPL11]
    return float(torch.stack(per, 1).max(1).values.mean().clamp(0, 1))


by = defaultdict(set)
for ds, f, *_ in items:
    by[ds].add(f)


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


pas, rej = [], []
for ds, want in by.items():
    fr = read_frames(ds, want)
    for dds, f, box, wr in [i for i in items if i[0] == ds]:
        bgr = fr.get(f)
        if bgr is None:
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        crop = yi.crop_resize_pad(yi.normalize_rgb(rgb), list(box))
        if crop is None:
            continue
        _, pb = o_n.dinov2_blocks_forward(model, [crop], device, [2, 11])
        mask = yi.segment_box(seg, bgr, list(box), device)
        q = (yi.masked_query_patches(pb[11][0].cpu(), mask, list(box), pool)[0].to(device)
             if mask is not None else pb[11][0].to(device))
        s = p2_score(q)
        (rej if wr else pas).append(s)

pas, rej = np.array(pas), np.array(rej)
# use the choco-side operating threshold ~0.62 (same algorithm) and Dino's own passing dist
thr_choco = 0.62
rescued = int((rej >= thr_choco).sum())
print(f"\nHSV-passed Dino texture P2: mean {pas.mean():.3f} (n{len(pas)})")
print(f"HSV-rejected Dino texture P2: mean {rej.mean():.3f} (n{len(rej)})")
print(f"rescued @thr(choco 0.62) = {rescued}/{len(rej)}")
# how many rejected have texture >= min passing (a Dino-consistent rescue)
thr_dino = float(np.percentile(pas, 10)) if len(pas) else 0.62
print(f"rescued @thr(Dino p10 {thr_dino:.3f}) = {int((rej>=thr_dino).sum())}/{len(rej)}"
      f"  [needs Dino-specific thr -> generalization caveat]")
with open(os.path.join(OUT, "dinosaur_secondary_results.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["group", "n", "texture_mean", "texture_p10", "texture_p90"])
    w.writerow(["HSV_passed_Dino", len(pas), round(float(pas.mean()), 3),
                round(float(np.percentile(pas, 10)), 3), round(float(np.percentile(pas, 90)), 3)])
    w.writerow(["HSV_rejected_Dino", len(rej), round(float(rej.mean()), 3),
                round(float(np.percentile(rej, 10)), 3), round(float(np.percentile(rej, 90)), 3)])
    w.writerow(["rescued_at_choco_thr_0.62", rescued, "", "", ""])
    w.writerow(["rescued_at_Dino_p10", int((rej >= thr_dino).sum()), f"thr={round(thr_dino,3)}", "", ""])
print(f"-> {OUT}")
