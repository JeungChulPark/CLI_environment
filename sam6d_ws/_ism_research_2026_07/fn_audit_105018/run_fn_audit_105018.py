#!/usr/bin/env python3
"""run_fn_audit_105018.py — per-frame / per-object FN root-cause dump for ONE bag.

READ-ONLY diagnostic. It does NOT modify any operational file, config, threshold
or selection rule. It re-runs the *current operational Phase 1C bag path* --
i.e. exactly what tools/build_ism_inputs_imu.py does:

    separate YOLO-World pass per unique prompt  (avoids cross-prompt NMS suppression)
      -> yolo_ism_object_n.recognize()  (semantic gate -> masked-appe gate -> HSV gate)
      -> cross-object NMS (rank = res["rank_appe"], IoU > 0.5 drops the loser)

on EVERY color frame of the bag, and records for each (frame, object) which stage
the object died at. Two things are added on top, both strictly observational:

  1. a SECOND YOLO pass per prompt at conf=0.001 -- used only to tell
     "YOLO produced literally nothing" apart from "YOLO produced a box that the
     operational conf threshold (0.02) dropped". Decisions use the 0.02 pass only.
  2. a shadow HSV score for candidates that died BEFORE the HSV gate ran
     (informational; ism_hsv is side-effect free, so this cannot change anything).

Per-candidate semantic scores are captured by wrapping yolo_ism.semantic_score
with a pass-through recorder (returns the original value unchanged).

Outputs (under outputs/sam_105018_fn_audit/):
    frames.jsonl   one JSON object per frame (all objects, all candidates)
"""
import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE)
REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))

import yolo_ism as yi                 # noqa: E402
import yolo_ism_object_n as o_n       # noqa: E402
import ism_hsv                        # noqa: E402

COLOR = "/camera/camera/color/image_raw"
BAG = "sam_105018"
GT_CSV = os.path.join(RSRCH, "gt_input", "user_visibility_gt.csv")
RAW_CONF = 0.001                      # diagnostic-only "did YOLO see anything at all" pass


# ---------------------------------------------------------------- semantic recorder
_SEM_LOG = []
_ORIG_SEM = yi.semantic_score


def _sem_recorder(cls, tcls, k):
    v = _ORIG_SEM(cls, tcls, k)
    _SEM_LOG.append(float(v))
    return v


yi.semantic_score = _sem_recorder     # pass-through: identical return value


def _iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / union if union > 0 else 0.0


def load_gt():
    """(frame_id -> set(object names)) for this bag; only user_reviewed == yes."""
    gt = {}
    with open(GT_CSV, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r["dataset_name"] != BAG or r["user_reviewed"] != "yes":
                continue
            vis = set(t.strip() for t in r["visible_objects"].split(";")
                      if t.strip() and t.strip().lower() != "none")
            gt[int(r["frame_id"])] = vis
    return gt


def decode_color(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(buf, cv2.COLOR_RGB2BGR) if msg.encoding.lower() == "rgb8" else buf.copy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(REPO, "outputs", "sam_105018_fn_audit"))
    ap.add_argument("--limit", type=int, default=0, help="pilot: process only N frames")
    ap.add_argument("--device", default="cuda:0")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    device = args.device if torch.cuda.is_available() else "cpu"

    gt = load_gt()
    print(f"[gt] {len(gt)} labelled frames for {BAG}")

    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    print("[dinov2] loading")
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, rebuild=False)
    segmentor = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    unique_prompts, groups = o_n.build_prompt_groups(objs)
    min_score = min(float(o.get("score_threshold", 0.02)) for o in objs)
    img_sz = int(defaults.get("imgsz", 640))
    print(f"[yolo] imgsz={img_sz} op_conf={min_score} raw_conf={RAW_CONF} "
          f"prompts={len(unique_prompts)}")

    from ultralytics import YOLOWorld
    weights = defaults.get("weights", "yolov8m-worldv2.pt")

    def _mk(p):
        try:
            y = YOLOWorld(weights)
        except Exception:
            y = YOLOWorld("yolov8s-worldv2.pt")
        y.set_classes([p])
        return y

    yolo_per_prompt = [_mk(p) for p in unique_prompts]

    obj_of_prompt = {pi: [o["name"] for o in os_] for pi, os_ in groups.items()}
    meta = {
        "bag": BAG, "objects": [o["name"] for o in objs],
        "unique_prompts": unique_prompts,
        "prompt_of_object": {o["name"]: o["yolo_prompt"] for o in objs},
        "objects_of_prompt": obj_of_prompt,
        "op_conf": min_score, "raw_conf": RAW_CONF, "imgsz": img_sz,
        "thresholds": {o["name"]: {
            "score_threshold": float(o.get("score_threshold", 0.02)),
            "top_k": int(o.get("top_k", 3)),
            "similarity_threshold": float(o["similarity_threshold"]),
            "appe_gate": float(o["appe_gate"]),
            "hsv_gate_enabled": bool(o.get("hsv_gate_enabled", False)),
            "hsv_gate_threshold": float(o.get("hsv_gate_threshold", 0.0)),
            "hsv_hue_correction_ocv": int(o.get("hsv_hue_correction_ocv", 0)),
            "appe_blocks": o["_blocks"], "nms_rank_blocks": o["_rank_blocks"],
            "training_free_gate_enabled": bool(o.get("training_free_gate_enabled", False)),
        } for o in objs},
    }
    json.dump(meta, open(os.path.join(args.out, "run_meta.json"), "w"), indent=2)

    frames_dir = os.path.join(args.out, "frames")
    os.makedirs(frames_dir, exist_ok=True)
    jl = open(os.path.join(args.out, "frames.jsonl"), "w")

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    bag_dir = os.path.join(REPO, "data", "ros2_bag", BAG)
    uri = os.path.join(bag_dir, "bag") if os.path.isdir(os.path.join(bag_dir, "bag")) else bag_dir
    ts = get_typestore(Stores.ROS2_HUMBLE)

    t_start = time.time()
    nproc = 0
    with AnyReader([Path(uri)], default_typestore=ts) as reader:
        ccon = [c for c in reader.connections if c.topic == COLOR]
        ci = -1
        for con, tstamp, raw in reader.messages(connections=ccon):
            ci += 1
            if args.limit and nproc >= args.limit:
                break
            bgr = decode_color(reader.deserialize(raw, con.msgtype))
            h, w = bgr.shape[:2]
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            norm_full = yi.normalize_rgb(rgb)

            # ---- YOLO: raw pass (diagnostic) then operational filter -------------
            prompt_raw, prompt_op = {}, {}
            for pk, yolo in enumerate(yolo_per_prompt):
                res = yolo.predict(bgr, conf=RAW_CONF, imgsz=img_sz, verbose=False, device=device)
                boxes = []
                if len(res) and res[0].boxes is not None and len(res[0].boxes) > 0:
                    b = res[0].boxes
                    for j in range(len(b)):
                        xy = b.xyxy[j].tolist()
                        x1 = max(0, min(int(xy[0]), w - 1))
                        y1 = max(0, min(int(xy[1]), h - 1))
                        x2 = max(x1 + 1, min(int(xy[2]), w))
                        y2 = max(y1 + 1, min(int(xy[3]), h))
                        boxes.append(([x1, y1, x2, y2], float(b.conf[j])))
                boxes.sort(key=lambda t: t[1], reverse=True)
                prompt_raw[pk] = boxes
                prompt_op[pk] = [t for t in boxes if t[1] >= min_score]

            # ---- per object: operational recognize() -----------------------------
            per_obj = {}
            accepted_res = []
            for pi, objs_here in groups.items():
                cand = prompt_op[pi]                      # already conf-desc sorted
                for o in objs_here:
                    thr = float(o.get("score_threshold", 0.0))
                    passed = [(b, s) for (b, s) in cand if s >= thr]
                    sel = passed[:int(o.get("top_k", 3))]
                    # which of the selected boxes actually yield a crop (alignment
                    # for the semantic recorder -- recognize() skips crop==None)
                    crop_ok = [yi.crop_resize_pad(norm_full, b) is not None for b, _ in sel]

                    _SEM_LOG.clear()
                    r = o_n.recognize(o, cand, bgr, rgb, model, device,
                                      segmentor, pool, norm_full=norm_full)
                    sems = list(_SEM_LOG)

                    cands = []
                    k = 0
                    for idx, ((b, s), ok) in enumerate(zip(sel, crop_ok), start=1):
                        sv = None
                        if ok and k < len(sems):
                            sv = round(sems[k], 5)
                            k += 1
                        cands.append({"rank": idx, "box": b, "yolo_conf": round(s, 5),
                                      "sem": sv, "crop_ok": bool(ok)})

                    # shadow HSV for candidates that died before the HSV gate ran
                    hsv_shadow = None
                    if "hsv_score" not in r and r.get("box") is not None:
                        try:
                            hsv_shadow = round(float(ism_hsv.shadow_score(
                                bgr, r["box"], r.get("mask"), o.get("_hsv_proto"))), 5)
                        except Exception:
                            hsv_shadow = None

                    per_obj[o["name"]] = {
                        "obj": o,
                        "res": r,
                        "cands": cands,
                        "raw_n": len(prompt_raw[pi]),
                        "raw_best": round(prompt_raw[pi][0][1], 5) if prompt_raw[pi] else None,
                        "raw_boxes": [list(b) for b, _ in prompt_raw[pi][:8]],
                        "op_n": len(cand),
                        "op_boxes": [list(b) for b, _ in cand[:8]],
                        "sel_n": len(sel),
                        "hsv_shadow": hsv_shadow,
                    }
                    if r["accepted"] and r["mask"] is not None:
                        accepted_res.append((o, r))

            # ---- cross-object NMS (identical to build_ism_inputs_imu, rank=appe) --
            accepted_res.sort(key=lambda t: t[1].get("rank_appe") or t[1]["masked_appe"],
                              reverse=True)
            kept, suppressed = [], {}
            for o, r in accepted_res:
                hit = next((ko for ko, rk in kept if _iou(r["box"], rk["box"]) > 0.5), None)
                if hit is not None:
                    suppressed[o["name"]] = hit["name"]
                    continue
                kept.append((o, r))
            kept_names = set(o["name"] for o, _ in kept)

            # ---- assemble the frame record ---------------------------------------
            visible = gt.get(ci)          # None = frame not labelled
            rows = []
            for name, d in per_obj.items():
                r = d["res"]
                dec = r["decision"]
                stage = {"no-object(no-proposal)": "NO_YOLO_BBOX",
                         "no-object(below-sim)": "SEMANTIC_REJECT",
                         "no-object(below-appe)": "APPEARANCE_REJECT",
                         "no-object(below-hsv)": "HSV_REJECT"}.get(dec)
                if r["accepted"]:
                    stage = None if name in kept_names else "SELECTION_OR_OUTPUT_MISS"
                elif stage is None:
                    stage = "UNKNOWN"
                detected = bool(r["accepted"] and name in kept_names)

                if visible is None:
                    status, cause = "NO_GT_LABEL", ("DETECTED" if detected else (stage or "UNKNOWN"))
                elif name in visible:
                    status, cause = ("TP", "DETECTED") if detected else ("FN", stage or "UNKNOWN")
                else:
                    status, cause = ("FP", "DETECTED") if detected else ("TN", "NOT_VISIBLE")

                rows.append({
                    "object": name,
                    "gt_visible": (None if visible is None else int(name in visible)),
                    "yolo_raw_n": d["raw_n"], "yolo_raw_best": d["raw_best"],
                    "yolo_op_n": d["op_n"], "yolo_sel_n": d["sel_n"],
                    "raw_boxes": d["raw_boxes"], "op_boxes": d["op_boxes"],
                    "cands": d["cands"],
                    "sel_box": r["box"], "sel_rank": r["rank"],
                    "sem": round(float(r["best_sem"]), 5),
                    "sem_thr": float(d["obj"]["similarity_threshold"]),
                    "sem_pass": int(r["best_sem"] >= d["obj"]["similarity_threshold"]) if r["box"] is not None else 0,
                    "appe": round(float(r["masked_appe"]), 5),
                    "appe_thr": float(d["obj"]["appe_gate"]),
                    "appe_pass": (int(r["masked_appe"] >= d["obj"]["appe_gate"])
                                  if r["box"] is not None else None),
                    "rank_appe": round(float(r.get("rank_appe") or 0.0), 5),
                    "mask_area": int(r["mask_area"]),
                    "hsv": r.get("hsv_score", d["hsv_shadow"]),
                    "hsv_computed_by_gate": int("hsv_score" in r),
                    "hsv_thr": float(d["obj"].get("hsv_gate_threshold", 0.0)),
                    "hsv_pass": r.get("hsv_pass", None),
                    "decision": dec,
                    "detected": int(detected),
                    "suppressed_by": suppressed.get(name),
                    "final_status": status,
                    "cause": cause,
                })

            # ---- overlay image ----------------------------------------------------
            img_name = f"frame_{ci:06d}.jpg"
            masks = {name: per_obj[name]["res"]["mask"] for name in per_obj}
            draw_overlay(bgr, rows, masks, kept_names, visible, ci,
                         os.path.join(frames_dir, img_name))

            rec = {"frame_index": ci, "timestamp_ns": int(tstamp),
                   "image": f"frames/{img_name}",
                   "gt_labelled": visible is not None,
                   "gt_visible": (None if visible is None else sorted(visible)),
                   "detected": sorted(n for n in per_obj if per_obj[n]["res"]["accepted"]
                                      and n in kept_names),
                   "objects": rows}
            jl.write(json.dumps(rec) + "\n")
            nproc += 1
            if nproc % 10 == 0 or nproc == 1:
                el = time.time() - t_start
                print(f"[{nproc}] frame {ci}  {el:.1f}s  ({el/nproc:.2f}s/frame)", flush=True)

    jl.close()
    print(f"[done] {nproc} frames in {time.time()-t_start:.1f}s -> {args.out}")


# ------------------------------------------------------------------ overlay drawing
PANEL_W = 690
BLUE = (255, 150, 60)
GREEN = (60, 200, 60)
RED = (60, 60, 235)
GRAY = (150, 150, 150)


def draw_overlay(bgr, rows, masks, kept_names, visible, ci, out_path):
    h, w = bgr.shape[:2]
    canvas = np.full((max(h, 420), w + PANEL_W, 3), 28, np.uint8)
    img = bgr.copy()

    # 1) every operational YOLO candidate box (blue, thin) -- union over prompts
    seen = set()
    for r in rows:
        for b in r["op_boxes"]:
            key = tuple(b)
            if key in seen:
                continue
            seen.add(key)
            cv2.rectangle(img, (b[0], b[1]), (b[2], b[3]), BLUE, 1)

    # 2) each object's selected box: green = final detection, red = rejected
    for r in rows:
        if r["sel_box"] is None:
            continue
        x1, y1, x2, y2 = r["sel_box"]
        det = bool(r["detected"])
        col = GREEN if det else RED
        m = masks.get(r["object"])
        if m is not None:
            cnts, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img, cnts, -1, col, 2 if det else 1)
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 2 if det else 1)
        tag = r["object"] if det else f"{r['object']}:{r['cause'][:4]}"
        cv2.putText(img, tag, (x1 + 2, max(11, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, col, 1, cv2.LINE_AA)

    canvas[:h, :w] = img

    # 3) text panel
    x0 = w + 10
    y = 18

    def put(txt, col=(230, 230, 230), scale=0.42, dy=15):
        nonlocal y
        cv2.putText(canvas, txt, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, scale, col, 1, cv2.LINE_AA)
        y += dy

    gtxt = "unlabelled" if visible is None else (", ".join(sorted(visible)) or "none")
    put(f"frame {ci}", (255, 255, 255), 0.55, 20)
    put(f"GT visible: {gtxt}", (180, 220, 255))
    put("obj              GT  raw/op  sem        appe       hsv        result", GRAY, 0.40)
    for r in sorted(rows, key=lambda r: (-(r["gt_visible"] or 0), -r["detected"], r["object"])):
        interesting = (r["gt_visible"] or r["detected"] or r["sel_box"] is not None)
        if not interesting:
            continue
        g = "-" if r["gt_visible"] is None else ("Y" if r["gt_visible"] else "n")

        def f(v, thr, p):
            if v is None:
                return "   -      "
            mark = "P" if p else ("F" if p is not None else "?")
            return f"{v:.3f}/{thr:.2f}{mark}"

        sem = f(r["sem"] if r["sel_box"] is not None or r["sem"] else None, r["sem_thr"], r["sem_pass"])
        appe = f(r["appe"] if r["sel_box"] is not None else None, r["appe_thr"], r["appe_pass"])
        hsv = f(r["hsv"], r["hsv_thr"], r["hsv_pass"] if r["hsv_pass"] is not None
                else (None if r["hsv"] is None else int(r["hsv"] >= r["hsv_thr"])))
        col = GREEN if r["detected"] else (RED if r["final_status"] == "FN" else (230, 230, 230))
        put(f"{r['object'][:16]:16s} {g}  {r['yolo_raw_n']:2d}/{r['yolo_op_n']:<2d}  "
            f"{sem} {appe} {hsv} {r['final_status']}:{r['cause']}", col, 0.40, 14)
    put("", dy=6)
    put("blue=YOLO cand   green=final   red=rejected", GRAY, 0.38)
    cv2.imwrite(out_path, canvas, [cv2.IMWRITE_JPEG_QUALITY, 85])


if __name__ == "__main__":
    main()
