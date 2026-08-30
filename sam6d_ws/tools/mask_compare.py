#!/usr/bin/env python3
"""Compare 3 bbox-promptable segmentation models (SAM_b / FastSAM-s / MobileSAM)
on the operating-point milk decisions (conf 0.02 + sim>=0.40), and measure the
masked-query appe each enables.

Pipeline per SELECTED box:
  bbox --prompt--> {SAM_b, FastSAM-s, MobileSAM} --> 3 masks
  mask quality:  area-in-bbox, pairwise IoU, per-model runtime
  masked appe:   DINOv2 patch tokens of the crop, keep foreground patches per
                 model mask (AvgPool14 coverage>0.5), then SAM-6D faithful
                 query-anchored similarity (compute_straight) vs the BEST
                 template's masked patches. Also a no-mask appe (all patches)
                 with the same formula to isolate the masking effect.
  pseudo-GT:     reuse existing TP/FP labels (conf02 audit + sweep flip audit) —
                 NO new VLM round needed.

Does NOT modify yolo_ism.py (imports its helpers).
Outputs: outputs/mask_compare/{results.csv, summary.json, overlays/}
"""
import csv
import glob
import json
import os
import sys
import time
import collections

import cv2
import numpy as np
import torch
import torch.nn.functional as F

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)
import yolo_ism  # noqa: E402

S035 = os.path.join(REPO, "outputs", "yolo_ism_conf02_sim035")
OUT = os.path.join(REPO, "outputs", "mask_compare")
C02_CASES = os.path.join(REPO, "outputs", "yolo_ism_audit_conf02", "audit_cases_conf02.csv")
FLIP_VLM = os.path.join(REPO, "outputs", "yolo_ism_audit_sweep", "vlm_raw")
TEMPLATE_DIR = os.path.join(REPO, "sam6d_master", "SAM-6D", "Data", "custom",
                            "Milk_scaled_195mm", "templates")
CLS_CACHE = os.path.join(REPO, "outputs", "temp", "yolo_ism", "template_features",
                         "milk_dinov2_cls_features.pt")
APPE_CACHE = os.path.join(REPO, "outputs", "temp", "yolo_ism", "template_features",
                          "milk_dinov2_appe_features.pt")

BAGS = ["high_texture_around", "high_texture_far_close", "two_table_around",
        "two_table_around_goback", "two_table_diagonal1", "two_table_diagonal2",
        "two_table_goback", "only_milk", "milk_nomilk_bag"]
FRAMES_DIR = {"only_milk": "outputs/yolo_test/only_Milk/frames"}
SIM_OP = 0.40
MILK_BOX = {"correct_milk_box", "partial_milk_box"}
NONMILK_BOX = {"background_box", "other_object_box"}


def load_labels():
    lab = {}
    for r in csv.DictReader(open(C02_CASES)):
        lab[(r["bag"], int(r["frame_idx"]))] = (r["milk_present"], r["selected_box_correct"])
    for f in glob.glob(os.path.join(FLIP_VLM, "*.json")):
        bag = os.path.basename(f).rsplit("__", 1)[0]
        for r in json.load(open(f)):
            lab[(bag, int(r["frame_idx"]))] = (
                r.get("milk_present", "uncertain"), r.get("selected_box_correct", "na"))
    return lab


def gt_class(mp, box):
    if mp == "uncertain" or box in ("uncertain", "", "na", None):
        if mp == "no":
            return "FP"
        return "UNC"
    if box in NONMILK_BOX or mp == "no":
        return "FP"
    if box in MILK_BOX and mp == "yes":
        return "TP"
    return "UNC"


def targets():
    """bag -> {frame_idx: (name, box)} for sim>=0.40 milk decisions."""
    out = collections.defaultdict(dict)
    for bag in BAGS:
        for r in csv.DictReader(open(os.path.join(S035, bag, "yolo_ism_results.csv"))):
            if r["decision"] == "milk" and float(r["best_semantic_score"]) >= SIM_OP:
                box = [int(float(v)) for v in r["selected_bbox_xyxy"].split(";")]
                out[bag][int(r["frame_idx"])] = (r["timestamp"], box,
                                                 float(r["best_semantic_score"]))
    return out


def mask_iou(a, b):
    inter = np.logical_and(a, b).sum()
    uni = np.logical_or(a, b).sum()
    return float(inter / uni) if uni else 0.0


def compute_straight(q, r):
    """SAM-6D faithful query-anchored masked-patch similarity (single template)."""
    if q.shape[0] == 0 or r.shape[0] == 0:
        return 0.0
    sim = q @ r.T
    return float(sim.max(dim=1).values.mean().clamp(0, 1))


def main():
    os.makedirs(os.path.join(OUT, "overlays"), exist_ok=True)
    device = "cuda:0" if torch.cuda.is_available() else "cpu"

    # ---- models ----
    from ultralytics import SAM, FastSAM
    seg = {
        "sam_b": SAM(os.path.join(REPO, "sam_b.pt")),
        "mobile_sam": SAM(os.path.join(REPO, "mobile_sam.pt")),
        "fastsam_s": FastSAM(os.path.join(
            REPO, "sam6d_master/SAM-6D/Instance_Segmentation_Model/checkpoints/FastSAM/FastSAM-s.pt")),
    }
    dino = yolo_ism.build_dinov2(yolo_ism.DEFAULT_DINOV2_CKPT, device)
    tcls, _ = yolo_ism.build_template_cls(TEMPLATE_DIR, dino, device, CLS_CACHE, False)
    tappe, _ = yolo_ism.build_template_appe(TEMPLATE_DIR, dino, device, APPE_CACHE, False)
    pool = torch.nn.AvgPool2d(yolo_ism.PATCH, yolo_ism.PATCH)

    labels = load_labels()
    tb = targets()
    n_total = sum(len(v) for v in tb.values())
    print(f"[mask_compare] {n_total} selected boxes @ conf0.02 sim>={SIM_OP}")

    rows = []
    seg_time = collections.Counter()
    seg_n = 0
    overlay_saved = collections.Counter()
    for bag in BAGS:
        want = tb[bag]
        if not want:
            continue
        fd = os.path.join(REPO, FRAMES_DIR[bag]) if bag in FRAMES_DIR else ""
        for frame_idx, name, bgr in yolo_ism.resolve_frames(
                os.path.join("data", "ros2_bag", bag), fd, 10, 0,
                "/camera/camera/color/image_raw"):
            if frame_idx not in want:
                continue
            nm, box, sem = want[frame_idx]
            x1, y1, x2, y2 = box
            barea = max(1, (x2 - x1) * (y2 - y1))
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            norm = yolo_ism.normalize_rgb(rgb)
            crop = yolo_ism.crop_resize_pad(norm, box)
            cls, qpatch = yolo_ism.dinov2_forward(dino, crop, device, want_patch=True)
            best_t = int(torch.argmax(tcls @ cls))
            r_patches = tappe[best_t]
            appe_nomask = compute_straight(qpatch, r_patches)

            masks = {}
            for mname, model in seg.items():
                t0 = time.time()
                res = model(bgr, bboxes=[box], verbose=False, device=0)
                seg_time[mname] += time.time() - t0
                m = None
                if res and res[0].masks is not None and len(res[0].masks.data) > 0:
                    m = res[0].masks.data[0].cpu().numpy().astype(bool)
                    if m.shape != bgr.shape[:2]:
                        m = cv2.resize(m.astype(np.uint8), (bgr.shape[1], bgr.shape[0]),
                                       interpolation=cv2.INTER_NEAREST).astype(bool)
                masks[mname] = m
            seg_n += 1

            rec = {"bag": bag, "frame_idx": frame_idx, "name": nm, "best_sem": round(sem, 4)}
            mp, boxlab = labels.get((bag, frame_idx), ("uncertain", "na"))
            rec["gt"] = gt_class(mp, boxlab)
            rec["box_correct"] = boxlab
            rec["appe_nomask"] = round(appe_nomask, 4)
            for mname, m in masks.items():
                if m is None:
                    rec[f"area_{mname}"] = 0
                    rec[f"appe_{mname}"] = 0.0
                    continue
                rec[f"area_{mname}"] = int(m[y1:y2, x1:x2].sum())
                # masked query patches
                mt = torch.from_numpy(m[None].astype(np.float32))
                mcrop = yolo_ism.crop_resize_pad(mt, box, mode="nearest")
                cover = pool(mcrop.unsqueeze(0))[0].flatten()
                valid = cover > yolo_ism.VALID_PATCH_THRESH
                q_fg = qpatch[valid]
                rec[f"appe_{mname}"] = round(compute_straight(q_fg, r_patches), 4)
                rec[f"nfg_{mname}"] = int(valid.sum())
            # pairwise IoU
            mm = {k: v for k, v in masks.items() if v is not None}
            keys = list(mm)
            for i in range(len(keys)):
                for j in range(i + 1, len(keys)):
                    rec[f"iou_{keys[i]}_{keys[j]}"] = round(mask_iou(mm[keys[i]], mm[keys[j]]), 4)
            rows.append(rec)

            # overlays: all FP + up to 30 TP
            save = rec["gt"] == "FP" or (rec["gt"] == "TP" and overlay_saved["TP"] < 30)
            if save:
                ov = bgr.copy()
                cols = {"sam_b": (0, 0, 255), "mobile_sam": (0, 255, 0), "fastsam_s": (255, 0, 0)}
                for mname, m in masks.items():
                    if m is None:
                        continue
                    cnts, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(ov, cnts, -1, cols[mname], 2)
                cv2.rectangle(ov, (x1, y1), (x2, y2), (255, 255, 255), 1)
                cv2.putText(ov, f"{rec['gt']} sem{sem:.2f} R:sam G:mob B:fast",
                            (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.imwrite(os.path.join(OUT, "overlays", f"{rec['gt']}_{bag}__{nm}.jpg"), ov)
                overlay_saved[rec["gt"]] += 1

    # ---- write results ----
    keys = ["bag", "frame_idx", "name", "best_sem", "gt", "box_correct", "appe_nomask",
            "appe_sam_b", "appe_mobile_sam", "appe_fastsam_s",
            "area_sam_b", "area_mobile_sam", "area_fastsam_s",
            "nfg_sam_b", "nfg_mobile_sam", "nfg_fastsam_s",
            "iou_sam_b_mobile_sam", "iou_sam_b_fastsam_s", "iou_mobile_sam_fastsam_s"]
    with open(os.path.join(OUT, "results.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in keys})

    # ---- summary ----
    def stats(vals):
        vals = [v for v in vals if v is not None]
        if not vals:
            return None
        vals = sorted(vals)
        n = len(vals)
        return {"n": n, "mean": round(sum(vals)/n, 4),
                "median": round(vals[n//2], 4),
                "min": round(vals[0], 4), "max": round(vals[-1], 4)}

    models = ["sam_b", "mobile_sam", "fastsam_s"]
    summary = {"n_boxes": len(rows), "operating_point": "conf0.02 sim>=0.40",
               "seg_runtime_s_per_box": {m: round(seg_time[m]/max(1, seg_n), 4) for m in models},
               "pairwise_iou": {
                   "sam_b_vs_mobile": stats([r.get("iou_sam_b_mobile_sam") for r in rows]),
                   "sam_b_vs_fastsam": stats([r.get("iou_sam_b_fastsam_s") for r in rows]),
                   "mobile_vs_fastsam": stats([r.get("iou_mobile_sam_fastsam_s") for r in rows]),
               }}
    # appe TP vs FP separation per model (incl no-mask)
    sep = {}
    for tag in ["appe_nomask", "appe_sam_b", "appe_mobile_sam", "appe_fastsam_s"]:
        tp = [r[tag] for r in rows if r["gt"] == "TP"]
        fp = [r[tag] for r in rows if r["gt"] == "FP"]
        d = {"TP": stats(tp), "FP": stats(fp)}
        # best threshold to drop FP while keeping TP (maximize TP_kept - FP_kept proxy: youden)
        best = None
        cand = sorted(set([round(x, 3) for x in (tp + fp)]))
        for t in cand:
            tp_keep = sum(1 for v in tp if v >= t)
            fp_keep = sum(1 for v in fp if v >= t)
            tpr = tp_keep / len(tp) if tp else 0
            fpr = fp_keep / len(fp) if fp else 0
            j = tpr - fpr
            if best is None or j > best["youden"]:
                best = {"thr": t, "youden": round(j, 4), "TP_kept": tp_keep,
                        "TP_total": len(tp), "FP_kept": fp_keep, "FP_total": len(fp)}
        d["best_appe_gate"] = best
        sep[tag] = d
    summary["appe_TP_vs_FP_separation"] = sep
    json.dump(summary, open(os.path.join(OUT, "summary.json"), "w"), indent=2, ensure_ascii=False)

    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
