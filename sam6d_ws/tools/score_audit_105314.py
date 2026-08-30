#!/usr/bin/env python
"""Per-frame ISM(sem/appe) + PEM score overlay + score table for one bag.

Reads the ALREADY-BUILT bundles under outputs/pem_inputs/<bag> (ISM detections and
PEM results) and adds the one number they do not persist: the semantic (cls) score.
The detection json only stores `score` = masked_appe, so sem is recomputed here from
the same DINOv2 cls template caches the production run used. Nothing is re-detected:
boxes, masks and PEM poses are taken as-is, so the overlay shows the production
pipeline's real decisions.

PEM's published score is `pred_pose_score * ism_score` (run_inference_custom.py:296),
so pose-only is recovered by dividing out appe.

Writes:
  <out_dir>/frame_XXXXXX.png   every frame, all objects, sem/appe/pem per detection
  <out_dir>/scores.csv         one row per (frame, object, detection)
"""
import argparse
import csv
import glob
import json
import os
import sys

import cv2
import numpy as np
import torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
import yolo_ism as yi              # noqa: E402
import yolo_ism_object_n as o_n    # noqa: E402

# same palette as build_ism_inputs_imu, extended to every configured object
OBJ_COLORS = {
    "milk": (255, 80, 80), "Bear": (60, 170, 255), "Rabbit": (200, 120, 255),
    "Mugcup_high": (80, 220, 80), "saffron": (40, 200, 255),
    "choco_hazelnut_high": (130, 90, 60), "Sauce_high": (60, 60, 200),
    "Febreze_high": (255, 200, 60), "Dinosaur": (80, 255, 180),
    "Sikhye_high": (0, 215, 255),
}


def rle_to_mask(rle):
    """SAM mask_to_rle_pytorch inverse: counts are column-major runs starting with 0."""
    h, w = rle["size"]
    flat = np.zeros(h * w, dtype=bool)
    idx, val = 0, False
    for c in rle["counts"]:
        if val:
            flat[idx:idx + c] = True
        idx += c
        val = not val
    return flat.reshape(w, h).T  # permute(0,2,1) on encode -> transpose back


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--bag", default="sam_105314")
    p.add_argument("--config", default=o_n.DEFAULT_CONFIG)
    p.add_argument("--input-root", default=os.path.join(REPO_ROOT, "outputs", "pem_inputs"))
    p.add_argument("--out-dir", required=True)
    p.add_argument("--device", default="cuda:0")
    p.add_argument("--crop-dir", default="", help="if set, dump a per-detection crop for labelling")
    return p.parse_args()


@torch.no_grad()
def main():
    args = parse_args()
    device = args.device if torch.cuda.is_available() else "cpu"
    bag_dir = os.path.join(args.input_root, args.bag)
    os.makedirs(args.out_dir, exist_ok=True)
    if args.crop_dir:
        os.makedirs(args.crop_dir, exist_ok=True)

    defaults, objs = o_n.load_config(args.config)
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    tcls = {}
    for o in objs:
        if os.path.exists(o["cls_cache"]):
            # cache is a blob; the [N_tmpl,384] bank lives under "features"
            tcls[o["name"]] = torch.load(o["cls_cache"], map_location="cpu")["features"]
    print(f"[templates] cls caches loaded: {sorted(tcls)}")

    frames = sorted(glob.glob(os.path.join(bag_dir, "frame_*")))
    rows = []
    for fi, fdir in enumerate(frames):
        idx = int(os.path.basename(fdir).split("_")[1])
        bgr = cv2.imread(os.path.join(fdir, "rgb.png"))
        if bgr is None:
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        chw = yi.normalize_rgb(rgb)   # already [3,H,W], ImageNet-normalized

        vis = bgr.copy()
        lines = []
        for det_path in sorted(glob.glob(os.path.join(fdir, "detection_*.json"))):
            name = os.path.basename(det_path)[len("detection_"):-len(".json")]
            dets = json.load(open(det_path))
            # PEM result for this object, matched back to its ISM box
            pem_by_box = {}
            pem_path = os.path.join(fdir, f"pem_{name}", "sam6d_results", "detection_pem.json")
            if os.path.exists(pem_path):
                try:
                    for pd in json.load(open(pem_path)):
                        pem_by_box[tuple(pd["bbox"])] = float(pd["score"])
                except (json.JSONDecodeError, KeyError):
                    pass

            color = OBJ_COLORS.get(name, (0, 200, 0))
            for di, d in enumerate(dets):
                x, y, w, h = d["bbox"]
                box = [x, y, x + w, y + h]
                appe = float(d["score"])
                # sem: DINOv2 cls of the crop vs this object's template cls bank
                sem = float("nan")
                if name in tcls:
                    crop = yi.crop_resize_pad(chw, box)
                    if crop is not None:
                        cls, _ = yi.dinov2_forward(model, crop, device)
                        sem = yi.semantic_score(cls, tcls[name], 5)
                pem = pem_by_box.get(tuple(d["bbox"]), float("nan"))
                # PEM publishes pose_score * ism_score -> divide out to see pose alone
                pose_only = pem / appe if (pem == pem and appe > 1e-6) else float("nan")

                mask = rle_to_mask(d["segmentation"])
                vis = draw(vis, mask, box, name, sem, appe, pem, color)
                lines.append((name, sem, appe, pem))

                if args.crop_dir:
                    cx1, cy1 = max(0, x - 8), max(0, y - 8)
                    cx2, cy2 = min(bgr.shape[1], x + w + 8), min(bgr.shape[0], y + h + 8)
                    crop_img = bgr[cy1:cy2, cx1:cx2]
                    if crop_img.size:
                        cv2.imwrite(os.path.join(
                            args.crop_dir, f"{name}__f{idx:06d}__d{di}.png"), crop_img)

                rows.append({
                    "frame": idx, "object": name, "det_idx": di,
                    "bbox_xywh": json.dumps([x, y, w, h]),
                    "sem": round(sem, 4), "appe": round(appe, 4),
                    "pem": round(pem, 4) if pem == pem else "",
                    "pose_only": round(pose_only, 4) if pose_only == pose_only else "",
                    "mask_area": int(mask.sum()),
                })

        vis = header(vis, idx, len(frames), lines)
        cv2.imwrite(os.path.join(args.out_dir, f"frame_{idx:06d}.png"), vis)
        if fi % 50 == 0:
            print(f"  [{fi}/{len(frames)}] frame_{idx:06d}  {len(lines)} det")

    csv_path = os.path.join(args.out_dir, "scores.csv")
    with open(csv_path, "w", newline="") as f:
        wtr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wtr.writeheader()
        wtr.writerows(rows)
    print(f"[done] {len(frames)} frames, {len(rows)} detections -> {args.out_dir}")
    print(f"[done] scores -> {csv_path}")


def draw(vis, mask, box, name, sem, appe, pem, color):
    m = mask.astype(bool)
    tint = np.zeros_like(vis)
    tint[:] = color
    vis[m] = (0.55 * vis[m] + 0.45 * tint[m]).astype(np.uint8)
    cnts, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(vis, cnts, -1, color, 2)
    x1, y1, x2, y2 = [int(v) for v in box]
    cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)

    t1 = name
    t2 = f"sem {sem:.2f} appe {appe:.2f}"
    t3 = f"pem {pem:.2f}" if pem == pem else "pem --"
    texts = [t1, t2, t3]
    fs, th_ = 0.45, 1
    sizes = [cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, fs, th_)[0] for t in texts]
    bw = max(s[0] for s in sizes) + 6
    bh = sum(s[1] + 5 for s in sizes) + 4
    by = y1 - bh - 2
    if by < 0:
        by = y2 + 2
    cv2.rectangle(vis, (x1, by), (x1 + bw, by + bh), color, -1)
    yy = by + 2
    for t, s in zip(texts, sizes):
        yy += s[1] + 3
        cv2.putText(vis, t, (x1 + 3, yy), cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 255, 255), th_)
    return vis


def header(vis, idx, total, lines):
    pad = np.zeros((30 + 16 * max(1, len(lines)), vis.shape[1], 3), np.uint8)
    cv2.putText(pad, f"frame {idx}  ({total} frames)  detections={len(lines)}",
                (6, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    yy = 22
    for name, sem, appe, pem in sorted(lines, key=lambda t: -t[2]):
        yy += 16
        pstr = f"{pem:.3f}" if pem == pem else "  -- "
        cv2.putText(pad, f"{name:24s} sem={sem:.3f}  appe={appe:.3f}  pem={pstr}",
                    (6, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    OBJ_COLORS.get(name, (200, 200, 200)), 1)
    return np.vstack([vis, pad])


if __name__ == "__main__":
    main()
