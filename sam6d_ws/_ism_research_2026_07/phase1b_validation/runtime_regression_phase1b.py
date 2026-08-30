#!/usr/bin/env python3
"""runtime_regression_phase1b.py — GPU runtime regression for HSV shadow safety.

Runs the REAL operational pipeline (DINOv2 + MobileSAM + YOLO-World + recognize_frame) on
real bag frames, TWICE on identical inputs:
  A = HSV shadow OFF   (hsv_gate_shadow_mode = False)
  B = HSV shadow ON    (hsv_gate_shadow_mode = True)
and asserts the final decision of every (frame, object) is identical (AC-4). Also measures
the per-frame latency added by the shadow computation.

Usage: ~/miniconda3/envs/sam_yolo/bin/python .../runtime_regression_phase1b.py [--ds sam_110633] [--n 30]
"""
import argparse, csv, os, sys, time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE)
REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
import yolo_ism_object_n as o_n      # noqa: E402
import yolo_ism as yi               # noqa: E402

GT = os.path.join(RSRCH, "gt_input")
CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")


def bag_frames(ds, want, n):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    ts = get_typestore(Stores.ROS2_HUMBLE)
    want = sorted(want)[:n]
    hi = max(want)
    out = []
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
            bgr = cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else b.copy()
            out.append((i, bgr))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ds", default="sam_110633")
    ap.add_argument("--n", type=int, default=30)
    a = ap.parse_args()

    want = set()
    for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
        if r["dataset_name"] == a.ds and r["user_reviewed"] == "yes":
            want.add(int(r["frame_id"]))

    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    # prepare with shadow ON so the HSV proto is attached to every object
    for o in objs:
        o["hsv_gate_shadow_mode"] = True
    objs = o_n.prepare_objects(objs, model, device, False)
    seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    unique_prompts, groups = o_n.build_prompt_groups(objs)

    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes(unique_prompts)
    min_score = min(float(o.get("score_threshold", 0.02)) for o in objs)
    img_sz = int(defaults.get("imgsz", 960))

    frames = bag_frames(a.ds, want, a.n)
    print(f"{a.ds}: {len(frames)} frames, {len(objs)} objects")

    def run(shadow):
        for o in objs:
            o["hsv_gate_shadow_mode"] = shadow
        out = {}
        t = 0.0
        for fi, bgr in frames:
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            norm = yi.normalize_rgb(rgb)
            h, w = bgr.shape[:2]
            r = yolo.predict(bgr, conf=min_score, imgsz=img_sz, verbose=False, device=device)
            pb = {i: [] for i in range(len(unique_prompts))}
            if len(r) and r[0].boxes is not None and len(r[0].boxes) > 0:
                b = r[0].boxes
                for j in range(len(b)):
                    xy = b.xyxy[j].tolist()
                    x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                    x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                    ci = int(b.cls[j]) if b.cls is not None else 0
                    if ci in pb:
                        pb[ci].append(([x1, y1, x2, y2], float(b.conf[j])))
            torch.cuda.synchronize() if device.startswith("cuda") else None
            t0 = time.time()
            res = o_n.recognize_frame(groups, pb, bgr, rgb, norm, model, device, seg, pool)
            torch.cuda.synchronize() if device.startswith("cuda") else None
            t += time.time() - t0
            for name, rr in res.items():
                out[(fi, name)] = rr
        return out, t

    A, tA = run(False)      # shadow OFF
    B, tB = run(True)       # shadow ON

    # ---- compare final decisions field-by-field ----
    FIELDS = ["accepted", "decision", "num_proposals"]
    mism = 0; checked = 0; acc = 0; hsv_logged = 0
    box_mism = sem_mism = appe_mism = 0
    for k in A:
        a_, b_ = A[k], B[k]
        checked += 1
        for f in FIELDS:
            if a_.get(f) != b_.get(f):
                mism += 1
                print(f"  MISMATCH {k} {f}: {a_.get(f)} != {b_.get(f)}")
        if a_.get("box") != b_.get("box"):
            box_mism += 1
        if round(a_.get("best_sem", 0), 6) != round(b_.get("best_sem", 0), 6):
            sem_mism += 1
        if round(a_.get("masked_appe", 0), 6) != round(b_.get("masked_appe", 0), 6):
            appe_mism += 1
        if b_.get("accepted"):
            acc += 1
            if "hsv_score" in b_:
                hsv_logged += 1
        # OFF run must NOT have hsv fields
        assert "hsv_score" not in a_, f"shadow OFF leaked hsv field at {k}"

    n = len(frames)
    lines = [
        f"dataset={a.ds}  frames={n}  objects={len(objs)}  (frame,obj) pairs checked={checked}",
        f"decision mismatches (accepted/decision/num_proposals) = {mism}",
        f"box mismatches={box_mism}  best_sem mismatches={sem_mism}  masked_appe mismatches={appe_mism}",
        f"accepted candidates (B) = {acc}  |  of those with hsv_score logged = {hsv_logged}",
        f"recognize_frame total: OFF={tA:.3f}s  ON={tB:.3f}s  added={tB-tA:+.3f}s",
        f"added latency per frame (median-ish avg) = {1000*(tB-tA)/max(n,1):+.2f} ms/frame",
        f"VERDICT: {'PASS - shadow ON == OFF decisions' if mism==0 and box_mism==0 else 'FAIL'}",
    ]
    with open(os.path.join(HERE, "runtime_regression.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n" + "\n".join(lines))


if __name__ == "__main__":
    main()
