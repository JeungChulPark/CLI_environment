"""Mask-proposal study on YCB-V: which front end gives the best object masks, how fast?

  fastsam_full   FastSAM-x segment-everything masks (upstream SAM-6D settings: iou 0.9, conf 0.25,
                 max_det 200, imgsz 640; boxes under 5 % of the image side or masks under 3e-4 of the
                 image dropped), the masks FastSAM itself predicts
  fastsam_msam   the same FastSAM-x boxes, masks re-made from each box by MobileSAM (one call per frame)
  text_msam      YOLO-World text boxes (method-1 prompts, det_study/out/prompts_best_m.json), top-3 per
                 target object with score >= 0.02 (our ISM's top_k / score_threshold), masks by MobileSAM

Per GT instance of a target object (BOP19 list, visib_fract >= 0.1): best mask IoU against the GT
visible mask (mask_visib) over all proposals of the frame (class-agnostic; for text_msam also over
that object's own boxes only = class-aware). Reported: recall at IoU 0.5 / 0.75, mean best IoU,
proposals per image, and per-image time (proposal, mask, total; CUDA-synchronised, medians).

    python det_study/mask_study.py [--limit N] [--subset K] --out det_study/out/mask_study.json
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import paths as P  # noqa: E402
import data as D  # noqa: E402
import proposers as PR  # noqa: E402

SR = HERE.parents[3] / "sam6d_realtime"
sys.path.insert(0, str(SR))
import yolo_ism as yi  # noqa: E402

TEST = P.YCBV / "test"
DEV = "cuda:0"


def sync():
    torch.cuda.synchronize()


class FastSAM:
    def __init__(self):
        from ultralytics import YOLO
        self.m = YOLO(str(P.FASTSAM_X))
        self.args = dict(iou=0.9, conf=0.25, max_det=200, imgsz=640, verbose=False, device=DEV, half=False,
                         save=False)

    def __call__(self, bgr, want_masks):
        r = self.m.predict(bgr, **self.args)[0]
        if r.boxes is None or r.masks is None:
            return [], []
        h, w = bgr.shape[:2]
        md = r.masks.data
        area = md.float().sum(dim=(1, 2)) / float(md.shape[1] * md.shape[2])
        keep = [j for j, (b, ma) in enumerate(zip(r.boxes.xyxy.tolist(), area.tolist()))
                if (b[2] - b[0]) * (b[3] - b[1]) / (w * h) > 0.05 ** 2 and ma > 3e-4]
        boxes = [r.boxes.xyxy[j].tolist() for j in keep]
        masks = []
        if want_masks and keep:
            m = torch.nn.functional.interpolate(md[keep].unsqueeze(1).float(), size=(h, w), mode="bilinear",
                                                align_corners=False)[:, 0] > 0.5
            masks = list(m.cpu().numpy())
        return boxes, masks


def gt_masks(s, i):
    root = TEST / f"{s:06d}"
    sc = gt_masks.cache.setdefault(s, {k: json.load(open(root / f"{k}.json")) for k in ("scene_gt", "scene_gt_info")})
    out = []
    for k, (g, gi) in enumerate(zip(sc["scene_gt"][str(i)], sc["scene_gt_info"][str(i)])):
        if gi["visib_fract"] < 0.1:
            continue
        m = cv2.imread(str(root / "mask_visib" / f"{i:06d}_{k:06d}.png"), cv2.IMREAD_GRAYSCALE) > 0
        out.append((g["obj_id"], m))
    return out


gt_masks.cache = {}


def iou(a, b):
    inter = np.logical_and(a, b).sum()
    return float(inter) / float(max(np.logical_or(a, b).sum(), 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--subset", type=int, default=0, help="every k-th image of the 900 (25 = the 36-image check)")
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    items = D.load_targets()
    if a.subset:
        items = items[::a.subset]
    if a.limit:
        items = items[:a.limit]
    print(f"{len(items)} images", flush=True)

    fs = FastSAM()
    seg = yi.build_segmentor(str(SR / "mobile_sam.pt"), DEV)
    text = PR.build("text:yolov8m-worldv2.pt:" + str(HERE / "out" / "prompts_best_m.json"))

    def run_fastsam_full(bgr, oids):
        sync(); t0 = time.perf_counter()
        boxes, masks = fs(bgr, True)
        sync(); t1 = time.perf_counter()
        return [(None, m) for m in masks], 1e3 * (t1 - t0), 0.0

    def run_fastsam_msam(bgr, oids):
        sync(); t0 = time.perf_counter()
        boxes, _ = fs(bgr, False)
        sync(); t1 = time.perf_counter()
        masks = yi.segment_boxes(seg, bgr, boxes, DEV) if boxes else []
        sync(); t2 = time.perf_counter()
        return [(None, m) for m in masks if m is not None], 1e3 * (t1 - t0), 1e3 * (t2 - t1)

    def run_text_msam(bgr, oids):
        text.activate(oids)
        sync(); t0 = time.perf_counter()
        dets = text.predict(bgr, 0.02)
        top = {}
        for d in sorted(dets, key=lambda d: -d[2]):
            if len(top.setdefault(d[1], [])) < 3:
                top[d[1]].append(d[0])
        boxes = [(o, b) for o, bs in top.items() for b in bs]
        sync(); t1 = time.perf_counter()
        masks = yi.segment_boxes(seg, bgr, [b for _, b in boxes], DEV) if boxes else []
        sync(); t2 = time.perf_counter()
        return [(o, m) for (o, _), m in zip(boxes, masks) if m is not None], 1e3 * (t1 - t0), 1e3 * (t2 - t1)

    methods = {"fastsam_full": run_fastsam_full, "fastsam_msam": run_fastsam_msam, "text_msam": run_text_msam}
    for s, i, oids in items[:a.warmup]:
        bgr = D.load_image(s, i)[0]
        for f in methods.values():
            f(bgr, oids)

    res = {"images": len(items), "label": P.LABEL, "rule": "GT visib_fract >= 0.1, target objects, mask_visib IoU",
           "methods": {}}
    acc = {k: {"best": [], "best_cls": [], "n_prop": [], "t_prop": [], "t_mask": [], "per_obj": {}} for k in methods}
    for n, (s, i, oids) in enumerate(items):
        bgr = cv2.imread(str(TEST / f"{s:06d}" / "rgb" / f"{i:06d}.png"))
        gts = [(o, m) for o, m in gt_masks(s, i) if o in oids]
        for k, f in methods.items():
            props, tp, tm = f(bgr, oids)
            A = acc[k]
            A["n_prop"].append(len(props)); A["t_prop"].append(tp); A["t_mask"].append(tm)
            for o, gm in gts:
                ious = [iou(m, gm) for _, m in props]
                b = max(ious) if ious else 0.0
                A["best"].append(b)
                A["per_obj"].setdefault(o, []).append(b)
                if k == "text_msam":
                    own = [iou(m, gm) for po, m in props if po == o]
                    A["best_cls"].append(max(own) if own else 0.0)
        if n % 50 == 0:
            print(n, {k: round(float(np.mean(v["best"])), 3) for k, v in acc.items()}, flush=True)

    for k, A in acc.items():
        b = np.array(A["best"])
        t = np.array(A["t_prop"]) + np.array(A["t_mask"])
        r = {"gt_instances": int(len(b)),
             "recall_iou50_pct": round(100 * float((b >= 0.5).mean()), 1),
             "recall_iou75_pct": round(100 * float((b >= 0.75).mean()), 1),
             "mean_best_iou": round(float(b.mean()), 3),
             "proposals_per_image": round(float(np.mean(A["n_prop"])), 1),
             "time_ms_median": {"proposal": round(float(np.median(A["t_prop"])), 1),
                                "mask": round(float(np.median(A["t_mask"])), 1), "total": round(float(np.median(t)), 1)},
             "time_ms_mean_total": round(float(t.mean()), 1),
             "per_object_recall50": {P.YCB_NAMES[o - 1]: round(100 * float((np.array(v) >= 0.5).mean()), 1)
                                     for o, v in sorted(A["per_obj"].items())}}
        if A["best_cls"]:
            c = np.array(A["best_cls"])
            r["class_aware"] = {"recall_iou50_pct": round(100 * float((c >= 0.5).mean()), 1),
                                "recall_iou75_pct": round(100 * float((c >= 0.75).mean()), 1),
                                "mean_best_iou": round(float(c.mean()), 3)}
        res["methods"][k] = r
        print(k, {x: r[x] for x in ("recall_iou50_pct", "recall_iou75_pct", "mean_best_iou", "proposals_per_image",
                                    "time_ms_median")}, r.get("class_aware"), flush=True)
    json.dump(res, open(a.out, "w"), indent=1, ensure_ascii=False)
    print("->", a.out)


if __name__ == "__main__":
    main()
