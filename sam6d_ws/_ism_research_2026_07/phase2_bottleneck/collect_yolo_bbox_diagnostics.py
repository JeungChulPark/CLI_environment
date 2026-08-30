#!/usr/bin/env python3
"""collect_yolo_bbox_diagnostics.py — Workstream C: YOLO-World raw 박스 진단 덤프.

운영 검출기 = YOLO-World(yolov8m-worldv2), 운영 offline 경로는 set_classes(전 프롬프트)
단일 shared pass, imgsz 960, conf 0.02, ultralytics 기본 NMS(iou0.7, class-aware).
진단을 위해 conf=0.005 로 낮춰 raw 박스를 뽑고, 아래 6개 config 를 gt_input PNG(339프레임)
에서 재생해 raw 박스 좌표+conf 를 CSV 로 저장한다. 임계·NMS·prompt·해상도 분석은 오프라인.

configs:
  shared960     운영과 동일한 단일 multi-label pass (기본 NMS), imgsz960
  shared960_agn 위 + class-agnostic NMS
  perprompt960  프롬프트별 별도 pass (교차클래스 NMS 억제 제거 기준선)
  perprompt1280 프롬프트별, imgsz1280
  perprompt1920 프롬프트별, imgsz1920
  ext960        객체별 확장 prompt(사전 고정), 프롬프트별 pass

산출: raw/yolo_raw/<config>/<ds>.csv  (frame_id, object, x1,y1,x2,y2, conf, cls_source)
"""
import argparse, csv, glob, json, os, sys, time
import cv2
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
import yolo_ism_object_n as o_n            # noqa: E402

GT = os.path.join(RSRCH, "gt_input")
FRAMES = os.path.join(GT, "frames")
OUT = os.path.join(REPO, "outputs", "phase2_appearance_semantic_yolo_bbox")
RAW = os.path.join(OUT, "raw", "yolo_raw")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
CONF = 0.005; MAXDET = 300

# 확장 prompt (dump_proposals.EXT 와 동일, 사전 고정) — 현행(ps[0]) 제외한 대체/보강 1개
EXT = {
    "milk": "white paper milk carton", "choco_hazelnut_high": "maroon snack box",
    "Febreze_high": "blue spray bottle", "Mugcup_high": "pink ceramic mug",
    "saffron": "white plastic jug with handle", "Sauce_high": "brown sauce bottle",
    "Sikhye_high": "gold beverage can", "Bear": "brown teddy bear plush",
    "Rabbit": "white bunny plush toy", "Dinosaur": "green dinosaur plush toy",
}


def load_gt_frames():
    want = {}
    with open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r["user_reviewed"] == "yes":
                want.setdefault(r["dataset_name"], []).append(int(r["frame_id"]))
    return {k: sorted(v) for k, v in want.items()}


def run(model, img, imgsz, device, agnostic, names):
    r = model.predict(img, conf=CONF, imgsz=imgsz, max_det=MAXDET, verbose=False,
                      device=device, agnostic_nms=agnostic)
    out = []
    if len(r) and r[0].boxes is not None and len(r[0].boxes):
        b = r[0].boxes; H, W = img.shape[:2]
        for j in range(len(b)):
            xy = b.xyxy[j].tolist(); ci = int(b.cls[j]) if b.cls is not None else 0
            out.append((names[ci], max(0, int(xy[0])), max(0, int(xy[1])),
                        min(W, int(xy[2])), min(H, int(xy[3])), round(float(b.conf[j]), 4)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--configs", nargs="+", default=["shared960", "shared960_agn",
                    "perprompt960", "perprompt1280", "perprompt1920", "ext960"])
    ap.add_argument("--datasets", nargs="+", default=DATASETS)
    ap.add_argument("--max-frames", type=int, default=0)
    a = ap.parse_args()

    want = load_gt_frames()
    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    W = defaults.get("weights", "yolov8m-worldv2.pt")
    prompts = [o["yolo_prompt"] for o in objs]
    P2O = {o["yolo_prompt"]: o["name"] for o in objs}
    from ultralytics import YOLOWorld
    cache = {}

    def M(classes, tag):
        if tag not in cache:
            y = YOLOWorld(W); y.set_classes(classes); cache[tag] = (y, classes)
        return cache[tag]

    prov = {}
    for cfg in a.configs:
        os.makedirs(os.path.join(RAW, cfg), exist_ok=True)
        t0 = time.time(); total = 0
        for ds in a.datasets:
            fids = want[ds][:a.max_frames] if a.max_frames else want[ds]
            rows = []
            for fid in fids:
                fp = os.path.join(FRAMES, ds, f"frame_{fid:06d}.png")
                bgr = cv2.imread(fp)
                if bgr is None:
                    continue
                if cfg == "shared960":
                    y, cl = M(prompts, "shared"); dets = run(y, bgr, 960, device, False, cl)
                    for nm, x1, y1, x2, y2, c in dets:
                        rows.append([fid, P2O.get(nm, ""), x1, y1, x2, y2, c, nm])
                elif cfg == "shared960_agn":
                    y, cl = M(prompts, "shared"); dets = run(y, bgr, 960, device, True, cl)
                    for nm, x1, y1, x2, y2, c in dets:
                        rows.append([fid, P2O.get(nm, ""), x1, y1, x2, y2, c, nm])
                elif cfg in ("perprompt960", "perprompt1280", "perprompt1920"):
                    sz = {"perprompt960": 960, "perprompt1280": 1280, "perprompt1920": 1920}[cfg]
                    for p in prompts:
                        y, _ = M([p], f"one::{p}")
                        for nm, x1, y1, x2, y2, c in run(y, bgr, sz, device, False, [p]):
                            rows.append([fid, P2O[p], x1, y1, x2, y2, c, p])
                elif cfg == "ext960":
                    for onm, p in EXT.items():
                        y, _ = M([p], f"one::{p}")
                        for nm, x1, y1, x2, y2, c in run(y, bgr, 960, device, False, [p]):
                            rows.append([fid, onm, x1, y1, x2, y2, c, p])
            with open(os.path.join(RAW, cfg, f"{ds}.csv"), "w", newline="") as f:
                w = csv.writer(f); w.writerow(["frame_id", "object", "x1", "y1", "x2", "y2", "conf", "cls_source"])
                w.writerows(rows); total += len(rows)
        prov[cfg] = {"boxes": total, "seconds": round(time.time() - t0, 1)}
        print(f"  {cfg:14s} boxes {total:>7,}  ({prov[cfg]['seconds']}s)")

    os.makedirs(os.path.join(OUT, "metrics"), exist_ok=True)
    json.dump({"conf_floor": CONF, "max_det": MAXDET, "configs": prov,
               "gt_frames": {k: len(v) for k, v in want.items()},
               "note": "raw dumps at conf=0.005; operational conf 0.02 & sweeps applied offline"},
              open(os.path.join(OUT, "metrics", "yolo_diag_provenance.json"), "w"),
              indent=2, ensure_ascii=False)
    print(f"-> {RAW}")


if __name__ == "__main__":
    main()
