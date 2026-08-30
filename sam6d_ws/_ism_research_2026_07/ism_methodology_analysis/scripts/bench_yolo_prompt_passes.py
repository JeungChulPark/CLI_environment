#!/usr/bin/env python3
"""bench_yolo_prompt_passes.py — 10 per-prompt YOLO passes vs 1 shared pass.

Production `tools/build_ism_inputs_imu.py` runs a SEPARATE YOLO-World pass per
unique prompt (10 passes/frame) because a shared `set_classes(all)` pass loses
boxes: ultralytics NMS keeps only the argmax class per box, so the white rabbit
labelled "brown bear doll" disappears from the rabbit prompt's candidate list.

This probe measures, WITHOUT touching any source file:
  A) 10 separate passes      (production, reference box set)
  B) 1 shared pass           (yolo_ism_object_n.main behaviour)
  C) 1 shared pass + multi_label NMS (in-process monkeypatch of
     ultralytics.utils.nms.non_max_suppression, this process only)

and reports latency plus per-prompt box recall of B and C against A.
"""
import argparse, glob, json, os, statistics, sys, time

import cv2
import torch

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))  # sam6d_ws
sys.path.insert(0, REPO)
import yolo_ism_object_n as o_n            # noqa: E402
from ultralytics import YOLOWorld          # noqa: E402
from ultralytics.utils import nms as ul_nms  # noqa: E402

_ORIG_NMS = ul_nms.non_max_suppression
_MULTI = {"on": False}


def _patched(prediction, *a, **kw):
    if _MULTI["on"]:
        kw["multi_label"] = True
    return _ORIG_NMS(prediction, *a, **kw)


ul_nms.non_max_suppression = _patched
# the predictor imported the symbol into its own module namespace
import ultralytics.models.yolo.detect.predict as _dp  # noqa: E402
_dp.nms.non_max_suppression = _patched


def sync():
    if torch.cuda.is_available():
        torch.cuda.synchronize()


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    u = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / u if u > 0 else 0.0


def boxes_of(res, npr):
    out = {i: [] for i in range(npr)}
    if len(res) and res[0].boxes is not None and len(res[0].boxes):
        b = res[0].boxes
        for j in range(len(b)):
            out[int(b.cls[j])].append(([int(v) for v in b.xyxy[j].tolist()], float(b.conf[j])))
    return out


def stat(v):
    v = sorted(v)
    return {"n": len(v), "median_ms": round(statistics.median(v), 2),
            "mean_ms": round(statistics.fmean(v), 2), "p90_ms": round(v[int(0.9*(len(v)-1))], 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame-globs", nargs="+", default=[
        os.path.join(REPO, "outputs/rgbd_imu_sdk_bag/SAM_loop1/input/frame_*/rgb.png"),
        os.path.join(REPO, "outputs/pem_inputs/sam_105314/frame_*/rgb.png")])
    ap.add_argument("--max-frames-per-glob", type=int, default=15)
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results", "bench_yolo_passes.json"))
    a = ap.parse_args()

    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    prompts, _ = o_n.build_prompt_groups(objs)
    img_sz = int(defaults.get("imgsz", 640))
    conf = min(float(o.get("score_threshold", 0.02)) for o in objs)
    w = defaults.get("weights", "yolov8m-worldv2.pt")

    per_prompt = []
    for p in prompts:
        y = YOLOWorld(w); y.set_classes([p]); per_prompt.append(y)
    shared = YOLOWorld(w); shared.set_classes(prompts)

    frames = []
    for g in a.frame_globs:
        frames += sorted(glob.glob(g))[: a.max_frames_per_glob]
    print(f"[bench] {len(frames)} frames, {len(prompts)} prompts, imgsz={img_sz}, conf={conf}")

    tA, tB, tC = [], [], []
    recall_B = {"hit": 0, "tot": 0}
    recall_C = {"hit": 0, "tot": 0}
    extra_C = 0
    for fi, fp in enumerate(frames):
        bgr = cv2.imread(fp)
        if bgr is None:
            continue
        warm = fi < a.warmup

        sync(); t0 = time.perf_counter()
        A = {i: [] for i in range(len(prompts))}
        for pk, y in enumerate(per_prompt):
            r = y.predict(bgr, conf=conf, imgsz=img_sz, verbose=False, device=device)
            if len(r) and r[0].boxes is not None and len(r[0].boxes):
                b = r[0].boxes
                for j in range(len(b)):
                    A[pk].append(([int(v) for v in b.xyxy[j].tolist()], float(b.conf[j])))
        sync(); dA = (time.perf_counter() - t0) * 1e3

        _MULTI["on"] = False
        sync(); t0 = time.perf_counter()
        B = boxes_of(shared.predict(bgr, conf=conf, imgsz=img_sz, verbose=False, device=device), len(prompts))
        sync(); dB = (time.perf_counter() - t0) * 1e3

        _MULTI["on"] = True
        sync(); t0 = time.perf_counter()
        C = boxes_of(shared.predict(bgr, conf=conf, imgsz=img_sz, verbose=False, device=device), len(prompts))
        sync(); dC = (time.perf_counter() - t0) * 1e3
        _MULTI["on"] = False

        if warm:
            continue
        tA.append(dA); tB.append(dB); tC.append(dC)
        for pk in range(len(prompts)):
            for box, _c in A[pk]:
                recall_B["tot"] += 1; recall_C["tot"] += 1
                if any(iou(box, b2) > 0.5 for b2, _ in B[pk]):
                    recall_B["hit"] += 1
                if any(iou(box, b2) > 0.5 for b2, _ in C[pk]):
                    recall_C["hit"] += 1
            for box, _c in C[pk]:
                if not any(iou(box, b2) > 0.5 for b2, _ in A[pk]):
                    extra_C += 1

    out = {"prompts": prompts, "imgsz": img_sz, "conf": conf, "frames_steady": len(tA),
           "A_10_separate_passes": stat(tA), "B_1_shared_pass": stat(tB),
           "C_1_shared_multilabel": stat(tC),
           "recall_vs_A": {"B": round(recall_B["hit"]/max(1, recall_B["tot"]), 4),
                           "C": round(recall_C["hit"]/max(1, recall_C["tot"]), 4),
                           "reference_boxes": recall_B["tot"]},
           "C_boxes_not_in_A": extra_C}
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(out, open(a.out, "w"), indent=2)
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
