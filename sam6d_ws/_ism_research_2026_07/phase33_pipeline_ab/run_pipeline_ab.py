#!/usr/bin/env python3
"""run_pipeline_ab.py — 실제 운영 파이프라인으로 Phase 3 OFF/ON A/B (READ-ONLY on config).

두 구성 모두 yolo_ism_object_n 의 실제 함수(recognize_frame + apply_cross_object_nms)를
그대로 호출한다. 오프라인 후보풀 재계산이 아니다.
  OFF : 현행 배포값        appe_blocks[11] gate 0.55, cross-object NMS 없음
  ON  : appe_v2_enabled + cross_object_nms_enabled  ([2,9] gate 0.605, NMS IoU 0.9)
YOLO 는 운영 main() 과 동일한 '전체 프롬프트 1회 공유 패스'.

TP/FP 색은 사람 검증 보정 GT(935 + FP재판정 T 24 = 959셀) 기준.
산출: outputs/phase33_pipeline_ab/{viz,csv,metrics}/
"""
import argparse, csv, json, os, sys, time
from collections import defaultdict
import numpy as np
import cv2, torch

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, os.path.join(RSRCH, "phase21_fn_audit")); sys.path.insert(0, REPO)
import phase21_common as p21
import yolo_ism_object_n as o_n
import yolo_ism as yi

OUT = os.path.join(REPO, "outputs", "phase33_pipeline_ab")
VIZ = os.path.join(OUT, "viz")
FRAMES = os.path.join(RSRCH, "gt_input", "frames")
FPAUDIT = os.path.join(REPO, "outputs", "phase32_fp_audit", "fp_audit_task.csv")
FLAGS_ON = {"appe_v2_enabled": True, "cross_object_nms_enabled": True}


def corrected_gt():
    gt = {k: set(v) for k, v in p21.load_gt().items()}
    n = 0
    if os.path.isfile(FPAUDIT):
        for r in csv.DictReader(open(FPAUDIT)):
            if r["answer"].strip().upper() == "T":
                gt[(r["bag_name"], int(r["frame_id"]))].add(r["class_name"]); n += 1
    print(f"[gt] 사람 재판정 {n}셀 추가 -> 가시 셀 {sum(len(v) for v in gt.values())}")
    return gt


def draw(bgr, acc, vis):
    img = bgr.copy()
    items = sorted(acc.items(), key=lambda kv: (kv[0] not in vis, kv[0]))
    drawn = []
    for o, box in items:
        x1, y1, x2, y2 = box
        col = (90, 220, 90) if o in vis else (60, 60, 235)     # 초록=TP 빨강=FP
        ins = sum(5 for b in drawn if max(abs(b[0]-x1), abs(b[1]-y1),
                                          abs(b[2]-x2), abs(b[3]-y2)) <= 4)
        drawn.append(box)
        X1, Y1, X2, Y2 = x1+ins, y1+ins, x2-ins, y2-ins
        cv2.rectangle(img, (X1, Y1), (X2, Y2), col, 3)
        t = o.replace("_high", "")
        cv2.rectangle(img, (X1, max(0, Y1-19)), (X1+8*len(t), max(0, Y1-19)+19), col, -1)
        cv2.putText(img, t, (X1+2, max(0, Y1-19)+14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1)
    return img


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--limit", type=int, default=0)
    a = ap.parse_args()
    for d in ("viz", "csv", "metrics"):
        os.makedirs(os.path.join(OUT, d), exist_ok=True)
    defaults, objs_cfg = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0")
    device = device if torch.cuda.is_available() else "cpu"
    ckpt = defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT
    print(f"[dinov2] {ckpt} on {device}")
    model = yi.build_dinov2(ckpt, device)

    import copy
    sets = {}
    for tag, flags in (("OFF", {}), ("ON", FLAGS_ON)):
        cfg = copy.deepcopy(objs_cfg)
        for o in cfg: o.update(flags)
        print(f"[prep] {tag}  blocks={o_n._blocks_of(cfg[0])} gate={o_n._appe_gate_of(cfg[0])} "
              f"nms={bool(cfg[0].get('cross_object_nms_enabled', False))}")
        sets[tag] = o_n.prepare_objects(cfg, model, device, False)
    prompts, groups_off = o_n.build_prompt_groups(sets["OFF"])
    _, groups_on = o_n.build_prompt_groups(sets["ON"])
    GRP = {"OFF": groups_off, "ON": groups_on}

    seg_w = o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt"))
    segmentor = yi.build_segmentor(seg_w, device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes(prompts)
    min_score = min(float(o.get("score_threshold", 0.02)) for o in sets["OFF"])
    img_sz = int(defaults.get("imgsz", 640))
    print(f"[yolo] {len(prompts)} prompts, conf {min_score}, imgsz {img_sz}")

    gt = corrected_gt()
    frames = sorted(gt.keys())
    if a.limit: frames = frames[:a.limit]
    man = []; boxes = {}; agg = {t: [0, 0, 0] for t in ("OFF", "ON")}; t0 = time.time()
    for n, (ds, fr) in enumerate(frames, 1):
        bgr = cv2.imread(os.path.join(FRAMES, ds, f"frame_{fr:06d}.png"))
        if bgr is None: continue
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        norm_full = yi.normalize_rgb(rgb)
        pr = yolo.predict(bgr, conf=min_score, imgsz=img_sz, verbose=False, device=device)
        pb = {i: [] for i in range(len(prompts))}
        if len(pr) and pr[0].boxes is not None and len(pr[0].boxes):
            b = pr[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
                x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
                ci = int(b.cls[j]) if b.cls is not None else 0
                if ci in pb: pb[ci].append(([x1, y1, x2, y2], float(b.conf[j])))
        vis = gt[(ds, fr)]; acc = {}
        for tag in ("OFF", "ON"):
            res = o_n.recognize_frame(GRP[tag], pb, bgr, rgb, norm_full,
                                      model, device, segmentor, pool)
            res = o_n.apply_cross_object_nms([o for g in GRP[tag].values() for o in g], res)
            acc[tag] = {k: r["box"] for k, r in res.items() if r["accepted"] and r["box"]}
            for o in p21.OBJECTS:
                v, ok = o in vis, o in acc[tag]
                if v and ok: agg[tag][0] += 1
                elif ok: agg[tag][1] += 1
                elif v: agg[tag][2] += 1
        img = np.hstack([bgr, draw(bgr, acc["OFF"], vis), draw(bgr, acc["ON"], vis)])
        fn = f"{ds}_{fr:06d}.png"; cv2.imwrite(os.path.join(VIZ, fn), img)
        A, B = set(acc["OFF"]), set(acc["ON"])
        man.append({"name": f"{ds}_{fr:06d}", "bag": ds, "frame": fr,
                    "gt_visible": ";".join(sorted(vis)),
                    "OFF": ";".join(sorted(A)), "ON": ";".join(sorted(B)),
                    "gain_TP": ";".join(sorted((B-A) & vis)),
                    "new_FP": ";".join(sorted((B-A) - vis)),
                    "lost_TP": ";".join(sorted((A-B) & vis)),
                    "removed_FP": ";".join(sorted((A-B) - vis)),
                    "changed": int(A != B), "image": fn})
        boxes[f"{ds}_{fr:06d}"] = {t: {k: list(v) for k, v in acc[t].items()} for t in ("OFF", "ON")}
        if n % 40 == 0: print(f"  {n}/{len(frames)}  ({time.time()-t0:.0f}s)")
    with open(os.path.join(OUT, "csv", "pipeline_ab_manifest.csv"), "w", newline="") as f:
        wcsv = csv.DictWriter(f, fieldnames=list(man[0].keys())); wcsv.writeheader(); wcsv.writerows(man)
    json.dump(boxes, open(os.path.join(OUT, "csv", "pipeline_ab_boxes.json"), "w"))
    met = {}
    for t in ("OFF", "ON"):
        TP, FP, FN = agg[t]; P = TP/max(TP+FP, 1); R = TP/max(TP+FN, 1)
        met[t] = dict(TP=TP, FP=FP, FN=FN, precision=round(P, 4), recall=round(R, 4),
                      f1=round(2*P*R/max(P+R, 1e-9), 4))
    met["frames"] = len(man); met["changed"] = sum(m["changed"] for m in man)
    json.dump(met, open(os.path.join(OUT, "metrics", "pipeline_ab.json"), "w"), indent=2)
    for t in ("OFF", "ON"):
        m = met[t]; print(f"  {t:4s} TP {m['TP']} FP {m['FP']} FN {m['FN']} "
                          f"P {m['precision']} R {m['recall']} F1 {m['f1']}")
    print(f"  변화 프레임 {met['changed']}/{met['frames']} -> {VIZ}")


if __name__ == "__main__":
    main()
