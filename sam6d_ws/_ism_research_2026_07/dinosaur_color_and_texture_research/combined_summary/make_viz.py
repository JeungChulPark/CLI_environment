#!/usr/bin/env python3
"""make_viz.py — Research A (template color correction) + Research B (choco vs brownbox texture).
READ-ONLY, GPU (for texture scores). Outputs to _bmad-output/.../dinosaur-color-and-texture-visualizations/
"""
import csv, os, sys
from collections import defaultdict
from pathlib import Path
import numpy as np
import cv2
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))     # sam6d_ws
RSRCH = os.path.join(REPO, "_ism_research_2026_07")
sys.path.insert(0, REPO)
import ism_hsv, yolo_ism as yi, yolo_ism_object_n as o_n   # noqa
CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
OUT = os.path.expanduser("~/temp_ws/CLI_environment/sam6d_ws/_bmad-output/implementation-artifacts/dinosaur-color-and-texture-visualizations")
os.makedirs(OUT, exist_ok=True)
BLK = (0, 0, 0)


def txt(img, s, o, c, sc=0.42, th=1):
    cv2.putText(img, s, o, cv2.FONT_HERSHEY_SIMPLEX, sc, BLK, th + 2, cv2.LINE_AA)
    cv2.putText(img, s, o, cv2.FONT_HERSHEY_SIMPLEX, sc, c, th, cv2.LINE_AA)


defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
dino = [o for o in objs if o["name"] == "Dinosaur"][0]

# ---------- VIZ A: render template original vs +9 hue-corrected ----------
td = dino["template_dir"]
rgb = cv2.imread(f"{td}/rgb_0.png"); mk = cv2.imread(f"{td}/mask_0.png", cv2.IMREAD_GRAYSCALE) > 0
ys, xs = np.where(mk)
crop = rgb[ys.min():ys.max(), xs.min():xs.max()]


def hue_shift_img(bgr, d):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).astype(np.int16)
    hsv[..., 0] = (hsv[..., 0] + d) % 180
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)


orig = cv2.resize(crop, (150, 150)); corr = cv2.resize(hue_shift_img(crop, 9), (150, 150))
h0 = float(np.median(cv2.cvtColor(orig, cv2.COLOR_BGR2HSV).reshape(-1, 3)[:, 0])) * 2
h9 = float(np.median(cv2.cvtColor(corr, cv2.COLOR_BGR2HSV).reshape(-1, 3)[:, 0])) * 2


def panelA(img, title, sub, col):
    p = np.full((30, 150, 3), 245, np.uint8); txt(p, title, (4, 20), BLK, 0.5, 1)
    c = np.full((44, 150, 3), 245, np.uint8)
    for i, (s, cc) in enumerate(sub):
        txt(c, s, (4, 18 + 20 * i), cc, 0.42, 1)
    return np.vstack([p, img, c])


gap = np.full((224, 10, 3), 255, np.uint8)
pa = np.hstack([panelA(orig, "current render", [(f"Hue {h0:.0f}", BLK), ("HSV reject Dino", (0, 0, 255))], BLK),
                gap,
                panelA(corr, "A1: +9deg (Dino-only)", [(f"Hue {h9:.0f} (greener)", (0, 130, 0)),
                                                       ("recovers 6 TP, 0 FP", (0, 130, 0))], BLK)])
top = np.full((26, pa.shape[1], 3), 255, np.uint8)
txt(top, "Research A: Dinosaur template color correction (real Dino Hue ~100, current render 78)", (6, 18), BLK, 0.44, 1)
cv2.imwrite(f"{OUT}/research_a_template_color_correction.png", np.vstack([top, pa]))
print("wrote research_a_template_color_correction.png")

# ---------- VIZ B: choco vs brownbox crops + texture score ----------
device = "cuda:0" if torch.cuda.is_available() else "cpu"
model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
choco = [o for o in objs if o["name"] == "choco_hazelnut_high"][0]
choco["tcls"], _ = yi.build_template_cls(choco["template_dir"], model, device, choco["cls_cache"], False)
tpb = o_n.build_template_appe_blocks(choco["template_dir"], model, device,
                                     choco["appe_cache"].replace("_appe.pt", "_appe_b2-11.pt"), [2, 11], False)
seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
TPL11 = [t.to(device) for t in tpb[11]]


def p2(q):
    if q.shape[0] == 0:
        return 0.0
    per = [(q @ tp.T).max(1).values for tp in TPL11]
    return float(torch.stack(per, 1).max(1).values.mean().clamp(0, 1))


rows = list(csv.DictReader(open(os.path.join(RSRCH, "ism_accuracy_observation", "labels", "box_labels_merged.csv"))))
choco_c = [r for r in rows if r["true_class"] == "choco_hazelnut_high" and "ACC:choco_hazelnut_high" in r["claims"]]
carton_c = [r for r in rows if r["true_class"] == "carton" and "ACC:choco_hazelnut_high" in r["claims"]]


def frame(ds, fi):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    ts = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(os.path.join(CONV, ds))], default_typestore=ts) as rd:
        conns = [c for c in rd.connections if c.topic == "/camera/camera/color/image_raw"]
        i = -1
        for conn, t, raw in rd.messages(connections=conns):
            i += 1
            if i < fi:
                continue
            m = rd.deserialize(raw, conn.msgtype)
            b = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
            return cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else b.copy()


def tile(r, is_choco):
    ds = r["dataset"]; f = int(r["frame_id"]); box = tuple(int(v) for v in r["uid"].split("|")[2].split("_"))
    bgr = frame(ds, f)
    if bgr is None:
        return None
    crop = yi.crop_resize_pad(yi.normalize_rgb(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)), list(box))
    if crop is None:
        return None
    _, pb = o_n.dinov2_blocks_forward(model, [crop], device, [2, 11])
    mask = yi.segment_box(seg, bgr, list(box), device)
    q = (yi.masked_query_patches(pb[11][0].cpu(), mask, list(box), pool)[0].to(device)
         if mask is not None else pb[11][0].to(device))
    s = p2(q)
    x1, y1, x2, y2 = box
    im = cv2.resize(bgr[y1:y2, x1:x2], (130, 130))
    col = (0, 130, 0) if is_choco else (0, 0, 255)
    verdict = "KEEP" if s >= 0.625 else "REJECT"
    txt(im, f"tex {s:.2f}", (2, 122), col, 0.45, 1)
    lab = np.full((36, 130, 3), 245, np.uint8)
    txt(lab, ("REAL choco" if is_choco else "brown box (FP)"), (2, 15), col, 0.42, 1)
    txt(lab, f"-> {verdict} @0.625", (2, 31), col, 0.42, 1)
    return np.vstack([im, lab])


def strip(items, isc, n=3):
    ts = [t for r in items[:n] if (t := tile(r, isc)) is not None]
    if not ts:
        return None
    row = ts[0]
    for t in ts[1:]:
        row = np.hstack([row, np.full((166, 6, 3), 255, np.uint8), t])
    return row


sc = strip(choco_c, True); sb = strip(carton_c, False)
W = max(sc.shape[1], sb.shape[1])
def pad(a): return np.hstack([a, np.full((a.shape[0], W - a.shape[1], 3), 255, np.uint8)])
hd1 = np.full((24, W, 3), 255, np.uint8); txt(hd1, "REAL choco (texture high -> KEEP)", (6, 17), (0, 130, 0), 0.5, 1)
hd2 = np.full((24, W, 3), 255, np.uint8); txt(hd2, "brown box mis-accepted as choco (texture low -> REJECT); HSV cannot (same color)", (6, 17), (0, 0, 255), 0.44, 1)
out = np.vstack([hd1, pad(sc), np.full((6, W, 3), 255, np.uint8), hd2, pad(sb)])
cv2.imwrite(f"{OUT}/research_b_choco_vs_brownbox_texture.png", out)
print("wrote research_b_choco_vs_brownbox_texture.png")
