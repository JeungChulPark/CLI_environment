#!/usr/bin/env python3
"""viz_prompt_scores.py — per-frame, per-object YOLO-World + ISM score overlays.

For one bag, for every processed frame and every enabled object, render an image
that shows EVERY YOLO-World candidate box routed to that object's prompt, each
labelled with its scores AND the gate at which it was filtered:

  [cut@yolo]  gray    — dropped before ISM (below score_threshold or beyond top_k)
  [cut@sem]   yellow  — reached ISM but lost the semantic(cls) ranking
  [cut@sem<thr] orange(SEL) — best semantic, but below similarity_threshold
  [cut@appe]  red-orange(SEL) — passed semantic, failed masked-appe gate
  [PASS]      green(SEL) — accepted

Labels are clamped to stay fully inside the image (edge boxes no longer lose
their scores). This unrolls yolo_ism_object_n.recognize() to keep per-candidate
scores while reproducing its EXACT selection/acceptance logic. Read-only reuse
of yolo_ism helpers.

Output: outputs/prompt_test/<bag>/<object>/frame_<idx>.png

--only      restrict the OBJECT SET (and thus YOLO prompts) to these names.
--save-only keep the FULL enabled prompt set (real cross-talk) but only WRITE
            images for these object names.
"""
import argparse
import os
import sys

import cv2
import numpy as np
import torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

import yolo_ism as yi
import yolo_ism_object_n as o_n

FONT = cv2.FONT_HERSHEY_SIMPLEX
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
# stage -> (BGR color, thick?)
STAGE_STYLE = {
    "yolo":     ((150, 150, 150), 1),   # gray
    "sem":      ((0, 230, 230), 1),     # yellow
    "sem<thr":  ((0, 170, 255), 3),     # orange  (selected, below sim)
    "appe":     ((0, 110, 255), 3),     # red-orange (selected, below appe)
    "pass":     ((0, 200, 0), 3),       # green  (accepted)
}
STAGE_TAG = {"yolo": "cut@yolo", "sem": "cut@sem", "sem<thr": "cut@sem<thr",
             "appe": "cut@appe", "pass": "PASS"}


def parse_args():
    p = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--config", default=o_n.DEFAULT_CONFIG)
    p.add_argument("--bag", default="two_table_diagonal1")
    p.add_argument("--frames-dir", default="")
    p.add_argument("--output-root", default=os.path.join(REPO_ROOT, "outputs", "prompt_test"))
    p.add_argument("--stride", type=int, default=0, help="0 = config defaults.stride")
    p.add_argument("--max-frames", type=int, default=0)
    p.add_argument("--device", default="")
    p.add_argument("--only", nargs="+", default=[],
                   help="restrict object set + YOLO prompts to these names")
    p.add_argument("--save-only", nargs="+", default=[],
                   help="keep full prompt set but only write images for these names")
    return p.parse_args()


def score_candidates(o, cand, bgr, rgb, model, device, segmentor, pool):
    """Return per-candidate dicts (box, yolo, rank, sem, appe, stage, selected)
    for EVERY routed YOLO box, plus the frame decision. cand = [(box,conf),...]
    sorted by conf desc. Mirrors recognize() gate logic exactly."""
    thr = float(o.get("score_threshold", 0.0))
    topk = int(o.get("top_k", 3))
    cands = []
    kept = 0
    for rank, (b, s) in enumerate(cand, start=1):
        entry = {"box": b, "yolo": float(s), "rank": rank, "sem": None,
                 "appe": None, "stage": "yolo", "selected": False, "_cls": None}
        if s >= thr and kept < topk:
            entry["_in_ism"] = True
            kept += 1
        else:
            entry["_in_ism"] = False     # cut at YOLO (below thr or beyond top_k)
        cands.append(entry)

    info = {"cands": cands, "decision": "no-object(no-proposal)", "accepted": False}
    ism = [c for c in cands if c["_in_ism"]]
    if not ism:
        return info

    norm_full = yi.normalize_rgb(rgb)
    best = None
    for c in ism:
        crop = yi.crop_resize_pad(norm_full, c["box"])
        if crop is None:
            c["sem"] = 0.0
            continue
        cls, _ = yi.dinov2_forward(model, crop, device, want_patch=False)
        c["sem"] = float(yi.semantic_score(cls, o["tcls"], o["match_topk"]))
        c["_cls"] = cls
        c["_crop"] = crop
        if best is None or c["sem"] > best["sem"]:
            best = c

    if best is None:
        return info
    best["selected"] = True
    # non-winning ISM candidates DID get a semantic score but lost the ranking ->
    # they were filtered at the semantic stage (cut@sem), not at YOLO.
    for c in ism:
        if c is not best:
            c["stage"] = "sem"

    if best["sem"] < o["similarity_threshold"]:
        best["stage"] = "sem<thr"
        info["decision"] = "no-object(below-sim)"
        return info

    # best passes semantic -> masked-appe gate
    _, qpatch = yi.dinov2_forward(model, best["_crop"], device, want_patch=True)
    if not o.get("use_mask", True):
        best["appe"] = float(yi.appearance_score(qpatch, o["tappe"], o["match_topk"]))
        best["stage"] = "pass"; info["decision"], info["accepted"] = "detected", True
        return info
    mask = yi.segment_box(segmentor, bgr, best["box"], device)
    q_fg = yi.masked_query_patches(qpatch, mask, best["box"], pool)[0] if mask is not None else qpatch
    best_t = int(torch.argmax(o["tcls"] @ best["_cls"]))
    best["appe"] = float(yi.masked_appe_score(q_fg, o["tappe"][best_t]))
    if best["appe"] >= o["appe_gate"]:
        best["stage"] = "pass"; info["decision"], info["accepted"] = "detected", True
    else:
        best["stage"] = "appe"; info["decision"] = "no-object(below-appe)"
    return info


def put_label(img, text, x, y, color, scale=0.42):
    """Draw a filled label whose rectangle is fully clamped inside the image."""
    h, w = img.shape[:2]
    (tw, th), _ = cv2.getTextSize(text, FONT, scale, 1)
    tw += 4; th += 5
    x = int(max(0, min(x, w - tw)))
    y = int(max(th, min(y, h)))           # y = bottom of the label rectangle
    cv2.rectangle(img, (x, y - th), (x + tw, y), color, -1)
    cv2.putText(img, text, (x + 2, y - 4), FONT, scale, BLACK, 1, cv2.LINE_AA)


def fmt(c):
    parts = [f"#{c['rank']}", f"y={c['yolo']:.2f}"]
    if c["sem"] is not None:
        parts.append(f"s={c['sem']:.2f}")
    if c["appe"] is not None:
        parts.append(f"a={c['appe']:.2f}")
    parts.append(f"[{STAGE_TAG[c['stage']]}]")
    return " ".join(parts)


def render(o, info, bgr, frame_idx):
    img = bgr.copy()
    h, w = img.shape[:2]
    # draw non-selected first, selected last (on top)
    order = sorted(info["cands"], key=lambda c: c["selected"])
    for c in order:
        color, thick = STAGE_STYLE[c["stage"]]
        x1, y1, x2, y2 = [int(v) for v in c["box"]]
        cv2.rectangle(img, (x1, y1), (x2, y2), color, thick)
        # label above the box top; clamp keeps it on-screen for edge boxes
        put_label(img, ("SEL " if c["selected"] else "") + fmt(c), x1, y1 - 2, color)

    title = (f"{o['name']}  prompt='{o['yolo_prompt']}'  frame={frame_idx}  | "
             f"{info['decision']}  | sim_thr={o['similarity_threshold']} "
             f"appe_gate={o['appe_gate']} top_k={o.get('top_k',3)}")
    legend = "legend: gray=cut@yolo  yellow=cut@sem  orange=cut@sem<thr  red=cut@appe  green=PASS"
    bar = np.zeros((40, w, 3), dtype=np.uint8)
    cv2.putText(bar, title, (4, 15), FONT, 0.42, WHITE, 1, cv2.LINE_AA)
    cv2.putText(bar, legend, (4, 33), FONT, 0.40, (180, 180, 180), 1, cv2.LINE_AA)
    return np.vstack([bar, img])


def main():
    args = parse_args()
    device = args.device or "cuda:0"
    device = device if torch.cuda.is_available() else "cpu"

    defaults, objs = o_n.load_config(args.config)
    if args.only:
        objs = [o for o in objs if o["name"] in set(args.only)]
        if not objs:
            raise SystemExit(f"[viz] none of --only {args.only} in config")
    save_set = set(args.save_only) if args.save_only else None
    stride = args.stride or int(defaults.get("stride", 10))
    bag_name = os.path.basename(os.path.normpath(args.bag))
    frames_dir = args.frames_dir or os.path.join(
        REPO_ROOT, "outputs", "yolo_test", bag_name, "frames")
    out_root = os.path.join(args.output_root, bag_name)

    print("[dinov2] loading")
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, rebuild=False)
    seg_w = o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt"))
    segmentor = yi.build_segmentor(seg_w, device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    unique_prompts, groups = o_n.build_prompt_groups(objs)
    min_score = min(float(o.get("score_threshold", 0.02)) for o in objs)

    write_objs = [o for o in objs if (save_set is None or o["name"] in save_set)]
    for o in write_objs:
        os.makedirs(os.path.join(out_root, o["name"]), exist_ok=True)
    print(f"[save] writing images for: {[o['name'] for o in write_objs]}")

    from ultralytics import YOLOWorld
    weights = defaults.get("weights", "yolov8m-worldv2.pt")
    try:
        yolo = YOLOWorld(weights)
    except Exception:
        yolo = YOLOWorld("yolov8s-worldv2.pt")
    yolo.set_classes(unique_prompts)
    print(f"[yolo] prompts={unique_prompts}")

    bag_path = args.bag if os.path.isabs(args.bag) else os.path.join(REPO_ROOT, "data", "ros2_bag", bag_name)
    n = 0
    accept = {o["name"]: 0 for o in write_objs}
    for frame_idx, _name, bgr in yi.resolve_frames(
            bag_path, frames_dir, max(1, stride), args.max_frames,
            "/camera/camera/color/image_raw"):
        n += 1
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res = yolo.predict(bgr, conf=min_score, imgsz=640, verbose=False, device=device)
        prompt_boxes = {i: [] for i in range(len(unique_prompts))}
        if len(res) and res[0].boxes is not None and len(res[0].boxes) > 0:
            b = res[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                cls_i = int(b.cls[j]) if b.cls is not None else 0
                if cls_i in prompt_boxes:
                    prompt_boxes[cls_i].append(([x1, y1, x2, y2], float(b.conf[j])))
        for pi, objs_here in groups.items():
            cand = sorted(prompt_boxes.get(pi, []), key=lambda t: t[1], reverse=True)
            for o in objs_here:
                if save_set is not None and o["name"] not in save_set:
                    continue
                info = score_candidates(o, cand, bgr, rgb, model, device, segmentor, pool)
                if info["accepted"]:
                    accept[o["name"]] += 1
                out = render(o, info, bgr, frame_idx)
                cv2.imwrite(os.path.join(out_root, o["name"], f"frame_{frame_idx:06d}.png"), out)
        if n % 20 == 0:
            print(f"  ...{n} frames")

    print(f"\n[done] {n} frames -> {out_root}")
    for o in write_objs:
        print(f"  {o['name']:22s} accepted {accept[o['name']]}/{n}")


if __name__ == "__main__":
    main()
