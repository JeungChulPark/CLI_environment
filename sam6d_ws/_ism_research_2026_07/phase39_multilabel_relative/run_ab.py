#!/usr/bin/env python3
"""phase39 — 박스당 다중 라벨(1) + 상대 판정(2) + 인형 색 판정(3) A/B.

사람 검증 보정 GT(959 가시 셀, phase33 과 동일 기준)로 TP/FP/FN 을 잰다.
운영 코드는 읽기 전용으로 import 해서 그대로 호출한다.

변형
  BASE      현행 배포 그대로 (YOLO 공유 1패스, NMS 는 박스당 1등 라벨만)
  ML        (1) 공유 1패스에 multi_label=True — 한 박스가 여러 프롬프트로 살아남는다
  ML_REL    (1)+(2) 한 박스의 주인을 '모든 객체 중 semantic 최대'로 정한다(상대 판정)
  ML_REL_D  (1)+(2)+(3) 승자가 인형 3종이면 그 안에서 색(HSV)으로 다시 고른다
  BASE_D    (3) 만 단독 — 현행에 인형 색 재판정만 얹는다

(2)의 요지: 지금은 객체마다 독립으로 "0.35 넘으면 통과"를 보고, 같은 박스를 두 객체가
가져가면 cross-object NMS 가 뒤늦게 정리한다. 여기서는 박스마다 주인을 먼저 정하므로
흰 토끼 박스에서 곰이 이길 수 없다. 절대 게이트(sem/appe/HSV)는 그대로 둔다 — FP 통제가
그쪽 몫이기 때문이다.
"""
import argparse, copy, csv, json, os, sys, time
from collections import defaultdict

import cv2, numpy as np, torch

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, os.path.join(RSRCH, "phase21_fn_audit")); sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import phase21_common as p21
import yolo_ism_object_n as o_n
import yolo_ism as yi
import ism_hsv

FRAMES = os.path.join(RSRCH, "gt_input", "frames")
FPAUDIT = os.path.join(REPO, "outputs", "phase32_fp_audit", "fp_audit_task.csv")
OUT = os.path.join(REPO, "outputs", "phase39_multilabel_relative")
DOLLS = ("Bear", "Rabbit", "Dinosaur")
VARIANTS = ("BASE", "ML", "ML_REL", "ML_REL_D", "BASE_D")


def corrected_gt():
    gt = {k: set(v) for k, v in p21.load_gt().items()}
    n = 0
    if os.path.isfile(FPAUDIT):
        for r in csv.DictReader(open(FPAUDIT)):
            if r["answer"].strip().upper() == "T":
                gt[(r["bag_name"], int(r["frame_id"]))].add(r["class_name"]); n += 1
    print(f"[gt] 사람 재판정 {n}셀 추가 -> 가시 셀 {sum(len(v) for v in gt.values())}")
    return gt


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix2-ix1), max(0, iy2-iy1)
    inter = iw*ih
    u = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter/u if u > 0 else 0.0


def yolo_boxes(yolo, bgr, prompts, conf, imgsz, device, multi_label):
    import ultralytics.utils.nms as N
    if multi_label:
        orig = N.non_max_suppression
        N.non_max_suppression = lambda *a, **k: orig(*a, **{**k, "multi_label": True})
    try:
        pr = yolo.predict(bgr, conf=conf, imgsz=imgsz, verbose=False, device=device)
    finally:
        if multi_label:
            N.non_max_suppression = orig
    h, w = bgr.shape[:2]
    pb = {i: [] for i in range(len(prompts))}
    if len(pr) and pr[0].boxes is not None and len(pr[0].boxes):
        b = pr[0].boxes
        for j in range(len(b)):
            xy = b.xyxy[j].tolist()
            x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
            x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
            ci = int(b.cls[j]) if b.cls is not None else 0
            if ci in pb:
                pb[ci].append(([x1, y1, x2, y2], float(b.conf[j])))
    for k in pb:
        pb[k].sort(key=lambda t: -t[1])
    return pb


def relative_assign(objs, pb, bgr, rgb, norm_full, model, device, segmentor, pool,
                    doll_color=False, name2obj=None):
    """박스마다 주인을 semantic 최대로 정한 뒤, 그 객체의 절대 게이트만 적용한다."""
    # 1) 프레임 안의 서로 다른 박스 모으기 (프롬프트 무관, IoU 0.95 로 중복 제거)
    uniq = []
    for lst in pb.values():
        for box, conf in lst:
            for u in uniq:
                if iou(u["box"], box) > 0.95:
                    u["conf"] = max(u["conf"], conf); break
            else:
                uniq.append({"box": box, "conf": conf})
    uniq = [u for u in uniq if u["conf"] >= 0.02]
    if not uniq:
        return {}
    crops, keep = [], []
    for u in uniq:
        c = yi.crop_resize_pad(norm_full, u["box"])
        if c is not None:
            crops.append(c); keep.append(u)
    if not crops:
        return {}
    need = sorted({b for o in objs for b in o["_need_blocks"]})
    cls_all, patch_all = o_n.dinov2_blocks_forward(model, crops, device, need)
    masks = yi.segment_boxes(segmentor, bgr, [u["box"] for u in keep], device)

    best_for_obj = {}
    for i, u in enumerate(keep):
        sems = {o["name"]: float(yi.semantic_score(cls_all[i], o["tcls"], o["match_topk"]))
                for o in objs}
        winner = max(sems, key=sems.get)
        if doll_color and winner in DOLLS and masks[i] is not None:
            # (3) 인형끼리는 형태로 못 가른다 -> 색으로 다시 고른다
            hs = {d: float(ism_hsv.shadow_score(bgr, u["box"], masks[i],
                                                name2obj[d].get("_hsv_proto")))
                  for d in DOLLS if d in name2obj}
            if hs:
                winner = max(hs, key=hs.get)
        o = name2obj[winner]
        if sems[winner] < o["similarity_threshold"]:
            continue
        mask = masks[i]
        if o.get("use_mask", True) and mask is None:
            continue
        qb = {b: patch_all[b][i].cpu() for b in o["_need_blocks"]}
        bt = int(torch.argmax(o["tcls"] @ cls_all[i]))
        m_appe = float(o_n.masked_appe_blocks(qb, mask, u["box"], pool, o, bt)) if mask is not None else 0.0
        if m_appe < o_n._appe_gate_of(o):
            continue
        if o_n._hsv_on(o) and bool(o.get("hsv_gate_enabled", False)):
            hv = float(ism_hsv.shadow_score(bgr, u["box"], mask, o.get("_hsv_proto")))
            if hv < float(o.get("hsv_gate_threshold", 0.0)):
                continue
        prev = best_for_obj.get(winner)
        if prev is None or m_appe > prev[1]:
            best_for_obj[winner] = (u["box"], m_appe)
    return {k: v[0] for k, v in best_for_obj.items()}


def doll_recolor(objs_by_name, res, bgr, segmentor, device):
    """(3) 단독: 이미 수락된 인형 라벨을 그 박스의 색으로 다시 고른다."""
    out = dict(res)
    for name in list(out):
        if name not in DOLLS:
            continue
        box = out[name]
        mask = yi.segment_box(segmentor, bgr, box, device)
        hs = {d: float(ism_hsv.shadow_score(bgr, box, mask, objs_by_name[d].get("_hsv_proto")))
              for d in DOLLS if d in objs_by_name}
        if not hs:
            continue
        best = max(hs, key=hs.get)
        if best != name:
            del out[name]
            out[best] = box                      # 색이 가리키는 인형으로 라벨 교체
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit", type=int, default=0)
    a = ap.parse_args()
    os.makedirs(os.path.join(OUT, "csv"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "metrics"), exist_ok=True)

    defaults, objs_cfg = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0")
    device = device if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(copy.deepcopy(objs_cfg), model, device, False)
    name2obj = {o["name"]: o for o in objs}
    prompts, groups = o_n.build_prompt_groups(objs)
    segmentor = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes(prompts)
    min_score = min(float(o.get("score_threshold", 0.02)) for o in objs)
    img_sz = int(defaults.get("imgsz", 640))
    print(f"[yolo] prompts {len(prompts)} conf {min_score} imgsz {img_sz} "
          f"| appe_gate {o_n._appe_gate_of(objs[0])} blocks {o_n._blocks_of(objs[0])} "
          f"| hsv {objs[0].get('hsv_gate_enabled')} | xnms {objs[0].get('cross_object_nms_enabled')}")

    gt = corrected_gt()
    frames = sorted(gt.keys())
    if a.limit:
        frames = frames[:a.limit]
    agg = {v: [0, 0, 0] for v in VARIANTS}
    per_obj = {v: defaultdict(lambda: [0, 0, 0]) for v in VARIANTS}
    rows = []
    t0 = time.time()
    for n, (ds, fr) in enumerate(frames, 1):
        bgr = cv2.imread(os.path.join(FRAMES, ds, f"frame_{fr:06d}.png"))
        if bgr is None:
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        norm_full = yi.normalize_rgb(rgb)
        vis = gt[(ds, fr)]
        acc = {}

        pb_off = yolo_boxes(yolo, bgr, prompts, min_score, img_sz, device, False)
        pb_ml = yolo_boxes(yolo, bgr, prompts, min_score, img_sz, device, True)

        r = o_n.recognize_frame(groups, pb_off, bgr, rgb, norm_full, model, device, segmentor, pool)
        r = o_n.apply_cross_object_nms(objs, r)
        acc["BASE"] = {k: v["box"] for k, v in r.items() if v["accepted"] and v["box"]}

        r = o_n.recognize_frame(groups, pb_ml, bgr, rgb, norm_full, model, device, segmentor, pool)
        r = o_n.apply_cross_object_nms(objs, r)
        acc["ML"] = {k: v["box"] for k, v in r.items() if v["accepted"] and v["box"]}

        acc["ML_REL"] = relative_assign(objs, pb_ml, bgr, rgb, norm_full, model, device,
                                        segmentor, pool, False, name2obj)
        acc["ML_REL_D"] = relative_assign(objs, pb_ml, bgr, rgb, norm_full, model, device,
                                          segmentor, pool, True, name2obj)
        acc["BASE_D"] = doll_recolor(name2obj, acc["BASE"], bgr, segmentor, device)

        for v in VARIANTS:
            for o in p21.OBJECTS:
                hit, ok = o in vis, o in acc[v]
                i = 0 if (hit and ok) else 1 if ok else 2 if hit else None
                if i is not None:
                    agg[v][i] += 1
                    per_obj[v][o][i] += 1
        rows.append({"bag": ds, "frame": fr, "gt": ";".join(sorted(vis)),
                     **{v: ";".join(sorted(acc[v])) for v in VARIANTS}})
        if n % 40 == 0:
            print(f"  {n}/{len(frames)}  ({time.time()-t0:.0f}s)", flush=True)

    with open(os.path.join(OUT, "csv", "phase39_manifest.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    met = {}
    print(f"\n{'variant':10s}{'TP':>6s}{'FP':>6s}{'FN':>6s}{'precision':>11s}{'recall':>9s}{'F1':>8s}")
    for v in VARIANTS:
        TP, FP, FN = agg[v]
        P = TP/max(TP+FP, 1); R = TP/max(TP+FN, 1)
        F = 2*P*R/max(P+R, 1e-9)
        met[v] = dict(TP=TP, FP=FP, FN=FN, precision=round(P, 4), recall=round(R, 4), f1=round(F, 4))
        print(f"{v:10s}{TP:6d}{FP:6d}{FN:6d}{P:11.4f}{R:9.4f}{F:8.4f}")
    met["frames"] = len(rows)
    met["per_object"] = {v: {o: per_obj[v][o] for o in p21.OBJECTS} for v in VARIANTS}
    json.dump(met, open(os.path.join(OUT, "metrics", "phase39.json"), "w"), indent=1)
    print(f"\n객체별 TP/FP/FN")
    for o in p21.OBJECTS:
        line = "".join(f"{'/'.join(str(x) for x in per_obj[v][o]):>16s}" for v in VARIANTS)
        print(f"{o:22s}{line}")
    print("  " + "".join(f"{v:>16s}" for v in VARIANTS))
    print(f"\n-> {OUT}")


if __name__ == "__main__":
    main()
