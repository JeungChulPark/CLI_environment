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
import time

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

def _sync():
    """CUDA is async: a GPU call returns before the work is done, so a bare
    time.time() around it measures dispatch, not compute. Every timer boundary
    below must sync or the stage split is fiction."""
    if torch.cuda.is_available():
        torch.cuda.synchronize()


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
    p.add_argument("--nms-rank", choices=["yolo", "appe", "sem"], default="appe",
                   help="which score decides the WINNER when two objects claim the same box. "
                        "'yolo' is the historical behaviour and is measurably WRONG: it ranks by "
                        "YOLO-World prompt confidence, discarding the better-evidenced label. In "
                        "110104 f19/22/72/75 the real white rabbit scored highest on BOTH DINOv2 "
                        "gates (sem .49-.65, appe .73-.78) and still lost to a marginally louder "
                        "'green dinosaur doll'/'brown bear doll' prompt -- the mechanism behind "
                        "the audit's Rabbit FP=0 / FN=258. Measured on 75 frames (2026-07-15): "
                        "doll label correct 14/22 -> 22/22 on a visually-confirmed control set, "
                        "detection count unchanged (198 -> 198), 0 regressions on real bears/"
                        "dinosaurs; over all 18 label flips, 16 improved, 1 neutral, 1 regressed "
                        "(105652 f288, a real dinosaur relabelled Bear).")
    p.add_argument("--multi", action="store_true",
                   help="emit EVERY proposal passing an object's gates instead of only the "
                        "argmax-semantic one (see yolo_ism_object_n.recognize_multi). MEASURED "
                        "AND REJECTED as a default (2026-07-15): on the 61 audit-named frames it "
                        "added 9 choco boxes and recovered 0 missed objects -- all 9 were the "
                        "plain brown shipping carton. The single slot picks argmax-SEMANTIC, "
                        "which was already picking the real box, so dropping it only releases "
                        "the carton FP. Kept for research/second-instance work.")
    return p.parse_args()


def decode_color(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(buf, cv2.COLOR_RGB2BGR) if msg.encoding.lower() == "rgb8" else buf.copy()


def decode_depth(msg):
    return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).copy()


def draw_ism_overlay(bgr, mask, box, label, score, color=(0, 200, 0)):
    """Return a copy of bgr with the segmentation mask tinted, bbox + label drawn."""
    vis = bgr.copy()
    m = mask.astype(bool)
    tint = np.zeros_like(vis)
    tint[:] = color
    vis[m] = (0.45 * vis[m] + 0.55 * tint[m]).astype(np.uint8)
    # mask contour for a crisp edge
    cnts, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(vis, cnts, -1, color, 2)
    x1, y1, x2, y2 = [int(v) for v in box]
    cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
    txt = f"{label} {score:.2f}"
    (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    ty = max(0, y1 - 6)
    cv2.rectangle(vis, (x1, ty - th - 4), (x1 + tw + 4, ty + 2), color, -1)
    cv2.putText(vis, txt, (x1 + 2, ty - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return vis


# distinct BGR colors per object name (stable across frames)
_OBJ_COLORS = {
    "milk": (255, 80, 80), "Bear": (60, 170, 255), "Rabbit": (200, 120, 255),
    "Mugcup_high": (80, 220, 80), "saffron": (40, 200, 255),
    "choco_hazelnut_high": (130, 90, 60), "Sauce_high": (60, 60, 200),
    "Febreze_high": (255, 200, 60),
}


def _iou_xyxy(a, b):
    """IoU of two [x1,y1,x2,y2] boxes."""
    ax1, ay1, ax2, ay2 = a; bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / union if union > 0 else 0.0


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
    img_sz = int(defaults.get("imgsz", 640))   # YOLO-World inference resolution (config-driven)
    print(f"[yolo] imgsz={img_sz}")

    from ultralytics import YOLOWorld
    weights = defaults.get("weights", "yolov8m-worldv2.pt")
    # SEPARATE YOLO pass per unique prompt: a shared set_classes(all) pass lets a
    # stronger prompt's box suppress a visually-similar object's box via NMS/argmax
    # (e.g. 'brown bear doll' swallowed the white rabbit). One model per prompt so
    # each object sees its OWN prompt's true confidence; recognize() then applies
    # the per-object score_threshold guard.
    def _mk(p):
        try:
            y = YOLOWorld(weights)
        except Exception:
            y = YOLOWorld("yolov8s-worldv2.pt")
        y.set_classes([p])
        return y
    yolo_per_prompt = [_mk(p) for p in unique_prompts]
    print(f"[yolo] {len(unique_prompts)} separate prompt passes/frame")

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
    timing = []   # one row per frame -> timing.csv
    for ci in sel:
        bgr = color_img[ci]
        depth = depth_img[need[ci]]
        h, w = bgr.shape[:2]
        if depth.shape != (h, w):
            print(f"[warn] frame {ci}: depth {depth.shape} != rgb {(h, w)}; skipping")
            continue
        _sync(); t_frame0 = time.perf_counter()
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        prompt_boxes = {k: [] for k in range(len(unique_prompts))}
        for pk, yolo in enumerate(yolo_per_prompt):
            res = yolo.predict(bgr, conf=min_score, imgsz=img_sz, verbose=False, device=device)
            if not (len(res) and res[0].boxes is not None and len(res[0].boxes) > 0):
                continue
            b = res[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                prompt_boxes[pk].append(([x1, y1, x2, y2], float(b.conf[j])))

        _sync(); t_yolo = time.perf_counter() - t_frame0

        fdir = os.path.join(out_bag, f"frame_{ci:06d}")
        os.makedirs(fdir, exist_ok=True)
        # save the input bundle for EVERY sampled frame (even with no detection)
        t0 = time.perf_counter()
        cv2.imwrite(os.path.join(fdir, "rgb.png"), bgr)
        cv2.imwrite(os.path.join(fdir, "depth.png"), depth)  # uint16 mm preserved
        json.dump(cam_json, open(os.path.join(fdir, "camera.json"), "w"))
        t_io = time.perf_counter() - t0

        # collect every object's accepted recognition, then dedup across objects
        t0 = time.perf_counter()
        accepted_res = []
        for pi, objs_here in groups.items():
            cand = sorted(prompt_boxes.get(pi, []), key=lambda t: t[1], reverse=True)
            for o in objs_here:
                if args.multi:
                    for r in o_n.recognize_multi(o, cand, bgr, rgb, model, device,
                                                 segmentor, pool):
                        if r["mask"] is not None:
                            accepted_res.append((o, r))
                    continue
                r = o_n.recognize(o, cand, bgr, rgb, model, device, segmentor, pool)
                if not r["accepted"] or r["mask"] is None:
                    continue
                accepted_res.append((o, r))
        # cross-object dedup: one physical toy can pass two toys' gates (bear vs
        # rabbit look alike to DINOv2). If two accepted boxes overlap heavily,
        # keep only ONE label -- --nms-rank picks which score decides the winner.
        # 'appe' ranks by res["rank_appe"], which config's nms_rank_blocks may compute
        # from different DINOv2 blocks than the gate uses. Falls back to masked_appe
        # (identical when nms_rank_blocks is unset).
        _sync(); t_recog = time.perf_counter() - t0
        t0 = time.perf_counter()
        _rank = {"yolo": "best_yolo", "appe": "rank_appe", "sem": "best_sem"}[args.nms_rank]
        accepted_res.sort(key=lambda t: t[1].get(_rank) or t[1]["masked_appe"], reverse=True)
        kept = []
        for o, r in accepted_res:
            if any(_iou_xyxy(r["box"], rk["box"]) > 0.5 for _, rk in kept):
                continue
            kept.append((o, r))

        # one detection_<obj>.json per object, holding EVERY kept box for it
        # (BOP format is already a list; --multi can put >1 entry in it)
        by_obj = {}
        for o, r in kept:
            by_obj.setdefault(o["name"], (o, []))[1].append(r)

        accepted = []
        for name, (o, rs) in by_obj.items():
            det, vis = [], bgr.copy()
            color = _OBJ_COLORS.get(name, (0, 200, 0))
            for r in rs:
                mask = r["mask"].astype(bool)
                rle = mask_to_rle_pytorch(torch.from_numpy(mask).unsqueeze(0))[0]
                rle["counts"] = [int(c) for c in rle["counts"]]
                x1, y1, x2, y2 = r["box"]
                det.append({
                    "scene_id": 0, "image_id": ci, "category_id": 1,
                    "bbox": [int(x1), int(y1), int(x2 - x1), int(y2 - y1)],
                    "score": float(r["masked_appe"]),
                    "segmentation": {"size": [int(h), int(w)], "counts": rle["counts"]},
                })
                vis = draw_ism_overlay(vis, mask, r["box"], name,
                                       float(r["masked_appe"]), color)
            seg_path = os.path.join(fdir, f"detection_{name}.json")
            json.dump(det, open(seg_path, "w"))
            cv2.imwrite(os.path.join(fdir, f"ism_{name}.png"), vis)
            manifest.append((fdir, name, o["cad_abs"], o["template_dir"], seg_path))
            accepted.append(f"{name}x{len(rs)}" if len(rs) > 1 else name)
        _sync(); t_post = time.perf_counter() - t0
        t_total = time.perf_counter() - t_frame0
        timing.append((ci, len(kept), round(t_yolo, 4), round(t_recog, 4),
                       round(t_io, 4), round(t_post, 4), round(t_total, 4)))
        print(f"[frame {ci:06d}] accepted: {accepted if accepted else 'none'}"
              f"  ({t_total*1000:.0f}ms)")

    tim_path = os.path.join(out_bag, "timing.csv")
    with open(tim_path, "w", newline="") as f:
        wcsv = csv.writer(f)
        wcsv.writerow(["frame", "n_kept", "t_yolo", "t_recog", "t_io", "t_post", "t_total"])
        wcsv.writerows(timing)
    print(f"[timing] {tim_path}")

    man_path = os.path.join(out_bag, "manifest.csv")
    with open(man_path, "w", newline="") as f:
        wcsv = csv.writer(f)
        wcsv.writerow(["frame_dir", "object", "cad_path", "template_dir", "seg_json"])
        wcsv.writerows(manifest)
    print(f"\n[done] {len(manifest)} (frame,object) bundles -> {out_bag}\n[manifest] {man_path}")


if __name__ == "__main__":
    main()
