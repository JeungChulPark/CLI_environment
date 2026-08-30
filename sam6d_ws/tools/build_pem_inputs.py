#!/usr/bin/env python3
"""build_pem_inputs.py — ISM -> PEM bridge: assemble PEM inputs from a ROS2 bag.

For selected color frames of a bag this:
  1. reads RGB (color/image_raw) + time-synced ALIGNED depth
     (aligned_depth_to_color/image_raw, uint16 mm) + camera intrinsics
     (color/camera_info) DIRECTLY from the bag (timestamp-synced by construction);
  2. runs the existing multi-object ISM recognition (yolo_ism_object_n.recognize,
     reused read-only) to get per-object accepted detections + full-image masks;
  3. writes, per frame, the PEM input bundle:
       outputs/pem_inputs/<bag>/frame_<ci>/rgb.png
                                          /depth.png        (uint16 mm)
                                          /camera.json      {cam_K[9], depth_scale:1.0}
                                          /detection_<obj>.json  (BOP list, 1 obj, RLE seg)
  + a manifest.csv (frame_dir, object, cad_path, template_dir, seg_json) for run_pem.

Masks are encoded as SAM uncompressed RLE (mask_to_rle_pytorch) — verified to
round-trip through PEM's cocomask decode path, so no pycocotools needed here.
Runs in the `sam_yolo` env (ISM/DINOv2). PEM itself runs in `sam6d_ros_humble`.
"""
import argparse
import bisect
import csv
import json
import os
import sys

import cv2
import numpy as np
import torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(
    REPO_ROOT, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))

import yolo_ism as yi
import yolo_ism_object_n as o_n
from segment_anything.utils.amg import mask_to_rle_pytorch

COLOR = "/camera/camera/color/image_raw"
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"
CAMINFO = "/camera/camera/color/camera_info"


def parse_args():
    p = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--bag", default="two_table_diagonal1")
    p.add_argument("--config", default=o_n.DEFAULT_CONFIG)
    p.add_argument("--objects", nargs="+", default=[],
                   help="restrict to these object names (default: all enabled w/ valid CAD)")
    p.add_argument("--stride", type=int, default=0,
                   help="every Nth color message (0 = evenly spread max-frames across bag)")
    p.add_argument("--max-frames", type=int, default=6)
    p.add_argument("--frame-indices", nargs="+", type=int, default=[],
                   help="explicit color message indices (overrides stride/max-frames)")
    p.add_argument("--output-root", default=os.path.join(REPO_ROOT, "outputs", "pem_inputs"))
    p.add_argument("--device", default="")
    return p.parse_args()


def decode_color(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(buf, cv2.COLOR_RGB2BGR) if msg.encoding.lower() == "rgb8" else buf.copy()


def decode_depth(msg):
    return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).copy()


def main():
    args = parse_args()
    device = args.device or "cuda:0"
    device = device if torch.cuda.is_available() else "cpu"

    defaults, objs = o_n.load_config(args.config)
    if args.objects:
        objs = [o for o in objs if o["name"] in set(args.objects)]
    # drop objects whose CAD is missing (e.g. milk moved to fail/)
    kept = []
    for o in objs:
        cad = o_n._abspath(o.get("cad_ply", ""))
        if cad and os.path.isfile(cad):
            o["cad_abs"] = cad
            kept.append(o)
        else:
            print(f"[skip-cad] {o['name']}: CAD not found ({o.get('cad_ply')})")
    objs = kept
    if not objs:
        raise SystemExit("[bridge] no objects with valid CAD")

    print("[dinov2] loading")
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, rebuild=False)
    segmentor = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    unique_prompts, groups = o_n.build_prompt_groups(objs)
    min_score = min(float(o.get("score_threshold", 0.02)) for o in objs)
    img_sz = int(defaults.get("imgsz", 640))
    tsim = o_n.template_similarity(objs)

    from ultralytics import YOLOWorld
    try:
        yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    except Exception:
        yolo = YOLOWorld("yolov8s-worldv2.pt")
    yolo.set_classes(unique_prompts)

    # ---- read bag: timestamps, then decode only what we need ----
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    from pathlib import Path
    bag_dir = os.path.join(REPO_ROOT, "data", "ros2_bag", args.bag)
    uri = os.path.join(bag_dir, "bag") if os.path.isdir(os.path.join(bag_dir, "bag")) else bag_dir
    ts = get_typestore(Stores.ROS2_HUMBLE)

    out_bag = os.path.join(args.output_root, args.bag)
    os.makedirs(out_bag, exist_ok=True)
    manifest = []

    with AnyReader([Path(uri)], default_typestore=ts) as reader:
        ccon = [c for c in reader.connections if c.topic == COLOR]
        dcon = [c for c in reader.connections if c.topic == DEPTH]
        icon = [c for c in reader.connections if c.topic == CAMINFO]
        if not (ccon and dcon and icon):
            raise SystemExit(f"[bridge] missing topics in {uri}: "
                             f"color={bool(ccon)} aligned_depth={bool(dcon)} caminfo={bool(icon)}")
        # camera intrinsics (first message)
        K = None
        for con, _t, raw in reader.messages(connections=icon):
            m = reader.deserialize(raw, con.msgtype)
            K = [float(v) for v in m.k]
            break
        cam_json = {"cam_K": K, "depth_scale": 1.0}
        # timestamps
        color_ts = [t for _, t, _ in reader.messages(connections=ccon)]
        depth_ts = [t for _, t, _ in reader.messages(connections=dcon)]
        ncolor = len(color_ts)
        if args.frame_indices:
            sel = [i for i in args.frame_indices if 0 <= i < ncolor]
        elif args.stride > 0:
            sel = list(range(0, ncolor, args.stride))[:args.max_frames]
        else:
            # evenly spread max_frames across the whole bag (verification sampling)
            sel = [int(round(x)) for x in np.linspace(0, ncolor - 1, args.max_frames)]
            sel = sorted(set(sel))
        sel_set = set(sel)
        # map each selected color -> nearest depth index
        need = {}
        for ci in sel:
            j = bisect.bisect_left(depth_ts, color_ts[ci])
            cand = [k for k in (j - 1, j) if 0 <= k < len(depth_ts)]
            need[ci] = min(cand, key=lambda k: abs(depth_ts[k] - color_ts[ci]))
        need_didx = set(need.values())
        print(f"[bag] {args.bag} ncolor={ncolor} ndepth={len(depth_ts)} "
              f"selected={len(sel)} frames K={K[:1]}...")

        # decode selected color + needed depth
        color_img, i = {}, -1
        for con, _t, raw in reader.messages(connections=ccon):
            i += 1
            if i in sel_set:
                color_img[i] = decode_color(reader.deserialize(raw, con.msgtype))
        depth_img, i = {}, -1
        for con, _t, raw in reader.messages(connections=dcon):
            i += 1
            if i in need_didx:
                depth_img[i] = decode_depth(reader.deserialize(raw, con.msgtype))

    # ---- per selected frame: ISM -> bundle ----
    for ci in sel:
        bgr = color_img[ci]
        depth = depth_img[need[ci]]
        h, w = bgr.shape[:2]
        if depth.shape != (h, w):
            print(f"[warn] frame {ci}: depth {depth.shape} != rgb {(h, w)}; skipping")
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        # imgsz used to be hard-coded 640 here while yolo_ism_object_n honoured the
        # config (960) -- so every integration run silently lost the small/distant
        # recall the config comment documents. Follow the config in both places.
        with o_n.multi_label_nms(o_n._multi_label_on(objs[0])):
            res = yolo.predict(bgr, conf=min_score, imgsz=img_sz, verbose=False, device=device)
        prompt_boxes = {k: [] for k in range(len(unique_prompts))}
        if len(res) and res[0].boxes is not None and len(res[0].boxes) > 0:
            b = res[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                cls_i = int(b.cls[j]) if b.cls is not None else 0
                if cls_i in prompt_boxes:
                    prompt_boxes[cls_i].append(([x1, y1, x2, y2], float(b.conf[j])))

        fdir = os.path.join(out_bag, f"frame_{ci:06d}")
        accepted = []
        norm_full = yi.normalize_rgb(rgb)
        for k in prompt_boxes:
            prompt_boxes[k].sort(key=lambda t: t[1], reverse=True)
        results = o_n.recognize_frame_auto(groups, prompt_boxes, bgr, rgb, norm_full,
                                           model, device, segmentor, pool, tsim)
        for pi, objs_here in groups.items():
            for o in objs_here:
                r = results.get(o["name"], {})
                if not r.get("accepted") or r.get("mask") is None:
                    continue
                mask = r["mask"].astype(bool)
                rle = mask_to_rle_pytorch(torch.from_numpy(mask).unsqueeze(0))[0]
                rle["counts"] = [int(c) for c in rle["counts"]]
                x1, y1, x2, y2 = r["box"]
                det = [{
                    "scene_id": 0, "image_id": ci, "category_id": 1,
                    "bbox": [int(x1), int(y1), int(x2 - x1), int(y2 - y1)],
                    "score": float(r["masked_appe"]),
                    "segmentation": {"size": [int(h), int(w)], "counts": rle["counts"]},
                }]
                os.makedirs(fdir, exist_ok=True)
                seg_path = os.path.join(fdir, f"detection_{o['name']}.json")
                json.dump(det, open(seg_path, "w"))
                manifest.append((fdir, o["name"], o["cad_abs"], o["template_dir"], seg_path))
                accepted.append(o["name"])
        if accepted:
            cv2.imwrite(os.path.join(fdir, "rgb.png"), bgr)
            cv2.imwrite(os.path.join(fdir, "depth.png"), depth)  # uint16 mm preserved
            json.dump(cam_json, open(os.path.join(fdir, "camera.json"), "w"))
            print(f"[frame {ci:06d}] accepted: {accepted}")
        else:
            print(f"[frame {ci:06d}] no accepted objects")

    man_path = os.path.join(out_bag, "manifest.csv")
    with open(man_path, "w", newline="") as f:
        wcsv = csv.writer(f)
        wcsv.writerow(["frame_dir", "object", "cad_path", "template_dir", "seg_json"])
        wcsv.writerows(manifest)
    print(f"\n[done] {len(manifest)} (frame,object) bundles -> {out_bag}\n[manifest] {man_path}")


if __name__ == "__main__":
    main()
