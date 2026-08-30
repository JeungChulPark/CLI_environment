#!/usr/bin/env python3
"""compare_yoloe_vs_world.py — YOLO-World vs YOLOE(비주얼 프롬프트) 실제 bbox 비교 (READ-ONLY).

패널: [원본 | YOLO-World(텍스트) | YOLOE multi-K(뷰별 슬롯) | YOLOE montage(뷰 통합)]
  · YOLO-World : 객체별 텍스트 프롬프트 per-prompt 패스
  · multi-K    : 렌더 K뷰 각각의 VPE 를 별도 클래스 슬롯으로 등록 → 객체별 max
  · montage    : 렌더 12뷰를 한 장으로 합쳐 cls=0 으로 전달(= 뷰 통합 프로토타입)
모두 conf 0.02. 박스에 클래스명만 표기(불필요 글자 없음).

산출: outputs/phase28_yoloe/{viz,csv,metrics}/
"""
import argparse, csv, glob, json, os, sys, time
from collections import defaultdict
import numpy as np
import cv2, torch

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, os.path.join(RSRCH, "phase21_fn_audit")); sys.path.insert(0, REPO)
import phase21_common as p21
import yolo_ism_object_n as o_n
from ultralytics import YOLOE, YOLOWorld
from ultralytics.models.yolo.yoloe import YOLOEVPSegPredictor

OUT = os.path.join(REPO, "outputs", "phase28_yoloe")
VIZ = os.path.join(OUT, "viz")
FRAMES = os.path.join(RSRCH, "gt_input", "frames")
YW = "yolov8m-worldv2.pt"; YE = "yoloe-11s-seg.pt"; CONF = 0.02; IMGSZ = 960
COL = {"Bear":(60,180,250),"Rabbit":(245,245,245),"Dinosaur":(90,220,90),"milk":(240,200,120),
       "choco_hazelnut_high":(80,90,190),"Febreze_high":(250,170,80),"Mugcup_high":(200,130,250),
       "saffron":(170,230,250),"Sauce_high":(60,140,240),"Sikhye_high":(70,220,240)}


def render_bbox(td, p):
    ix = os.path.basename(p).split("_")[1].split(".")[0]
    mk = cv2.imread(os.path.join(td, f"mask_{ix}.png"), 0)
    if mk is None: return None
    ys, xs = np.where(mk > 0)
    return None if not len(xs) else [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]


def build_multiK(objs, K):
    m = YOLOE(YE)
    pred = YOLOEVPSegPredictor(overrides=dict(task="segment", mode="predict", model=YE,
                                              verbose=False, conf=CONF))
    pred.setup_model(m.model)
    slots, slotname, owner = [], [], []
    for o in objs:
        td = o["template_dir"]; rps = sorted(glob.glob(os.path.join(td, "rgb_*.png")))
        idxs = np.linspace(0, len(rps) - 1, min(K, len(rps))).astype(int)
        vs = []
        for i in idxs:
            bb = render_bbox(td, rps[i])
            if bb is None: continue
            pred.set_prompts(dict(bboxes=np.array([bb], np.float32), cls=np.array([0])))
            vs.append(pred.get_vpe(rps[i]))
        if not vs: continue
        V = torch.cat(vs, dim=1); slots.append(V)
        for k in range(V.shape[1]):
            slotname.append(f"{o['name']}#{k}"); owner.append(o["name"])
    m2 = YOLOE(YE); m2.set_classes(slotname, torch.cat(slots, dim=1))
    return m2, owner


def build_montage(objs, K=12, cell=320):
    refs = {}
    for o in objs:
        td = o["template_dir"]; rps = sorted(glob.glob(os.path.join(td, "rgb_*.png")))
        idxs = np.linspace(0, len(rps) - 1, min(K, len(rps))).astype(int)
        sel = [rps[i] for i in idxs]
        cols = int(np.ceil(np.sqrt(len(sel)))); rows = int(np.ceil(len(sel) / cols))
        canvas = np.zeros((rows * cell, cols * cell, 3), np.uint8); boxes = []
        for k, p in enumerate(sel):
            ix = os.path.basename(p).split("_")[1].split(".")[0]
            im = cv2.imread(p); mk = cv2.imread(os.path.join(td, f"mask_{ix}.png"), 0)
            if im is None or mk is None: continue
            h, w = im.shape[:2]; s = cell / max(h, w)
            im2 = cv2.resize(im, (int(w * s), int(h * s))); mk2 = cv2.resize(mk, (int(w * s), int(h * s))) > 0
            r, c = divmod(k, cols); oy, ox = r * cell, c * cell
            canvas[oy:oy + im2.shape[0], ox:ox + im2.shape[1]] = im2
            ys, xs = np.where(mk2)
            if len(xs): boxes.append([ox + int(xs.min()), oy + int(ys.min()),
                                      ox + int(xs.max()), oy + int(ys.max())])
        p = os.path.join(OUT, "refs", f"{o['name']}.png"); os.makedirs(os.path.dirname(p), exist_ok=True)
        cv2.imwrite(p, canvas); refs[o["name"]] = (p, np.array(boxes, np.float32))
    return refs


def draw(bgr, dets):
    img = bgr.copy()
    for nm, boxes in sorted(dets.items()):
        col = COL.get(nm, (200, 200, 200))
        for (x1, y1, x2, y2, cf) in boxes:
            cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
        if boxes:
            x1, y1 = boxes[0][0], boxes[0][1]
            lab = nm.replace("_high", "")
            cv2.rectangle(img, (x1, max(0, y1 - 18)), (x1 + 8 * len(lab), y1), col, -1)
            cv2.putText(img, lab, (x1 + 2, max(12, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    return img


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--K", type=int, default=42)
    ap.add_argument("--limit", type=int, default=0); a = ap.parse_args()
    os.makedirs(VIZ, exist_ok=True); os.makedirs(os.path.join(OUT, "csv"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "metrics"), exist_ok=True)
    gt = p21.load_gt(); _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    names = [o["name"] for o in objs]
    print("multi-K VPE 구축..."); mK, owner = build_multiK(objs, a.K)
    print("montage refs 구축..."); refs = build_montage(objs)
    mM = YOLOE(YE); yc = {}
    frames = sorted(gt.keys())
    if a.limit: frames = frames[:a.limit]
    man = []; hit = {t: defaultdict(int) for t in ("W", "K", "M")}; tot = defaultdict(int)
    t0 = time.time()
    for n, (ds, fr) in enumerate(frames, 1):
        f = os.path.join(FRAMES, ds, f"frame_{fr:06d}.png"); bgr = cv2.imread(f)
        if bgr is None: continue
        H, Wd = bgr.shape[:2]
        dW, dK, dM = defaultdict(list), defaultdict(list), defaultdict(list)
        # YOLO-World per-prompt
        for o in objs:
            pr = o["yolo_prompt"]
            if pr not in yc:
                y = YOLOWorld(YW); y.set_classes([pr]); yc[pr] = y
            r = yc[pr].predict(bgr, conf=CONF, imgsz=IMGSZ, verbose=False)
            b = r[0].boxes
            if b is not None and len(b):
                for j in range(len(b)):
                    xy = [int(v) for v in b.xyxy[j].tolist()]
                    dW[o["name"]].append((max(0,xy[0]),max(0,xy[1]),min(Wd,xy[2]),min(H,xy[3]),float(b.conf[j])))
        # YOLOE multi-K (1 pass)
        r = mK.predict(f, conf=CONF, verbose=False); b = r[0].boxes
        if b is not None and len(b):
            for j in range(len(b)):
                xy = [int(v) for v in b.xyxy[j].tolist()]
                dK[owner[int(b.cls[j])]].append((xy[0],xy[1],xy[2],xy[3],float(b.conf[j])))
        # YOLOE montage per-object
        for o in objs:
            rp, bb = refs[o["name"]]
            if not len(bb): continue
            r = mM.predict(f, refer_image=rp, visual_prompts=dict(bboxes=bb, cls=np.zeros(len(bb), int)),
                           predictor=YOLOEVPSegPredictor, conf=CONF, verbose=False)
            b = r[0].boxes
            if b is not None and len(b):
                for j in range(len(b)):
                    xy = [int(v) for v in b.xyxy[j].tolist()]
                    dM[o["name"]].append((xy[0],xy[1],xy[2],xy[3],float(b.conf[j])))
        vis = gt[(ds, fr)]
        for o in vis:
            tot[o] += 1
            if dW.get(o): hit["W"][o] += 1
            if dK.get(o): hit["K"][o] += 1
            if dM.get(o): hit["M"][o] += 1
        combo = np.hstack([bgr, draw(bgr, dW), draw(bgr, dK), draw(bgr, dM)])
        fn = f"{ds}_{fr:06d}.png"; cv2.imwrite(os.path.join(VIZ, fn), combo)
        man.append({"name": f"{ds}_{fr:06d}", "bag": ds, "frame": fr,
                    "gt_visible": ";".join(sorted(vis)),
                    "W_found": ";".join(sorted(k for k in dW if dW[k])),
                    "K_found": ";".join(sorted(k for k in dK if dK[k])),
                    "M_found": ";".join(sorted(k for k in dM if dM[k])), "image": fn})
        if n % 40 == 0: print(f"  {n}/{len(frames)}  ({time.time()-t0:.0f}s)")
    with open(os.path.join(OUT, "csv", "yoloe_compare_manifest.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(man[0].keys())); w.writeheader(); w.writerows(man)
    T = sum(tot.values())
    res = {"total_visible": T, "K": a.K,
           "recall": {t: round(sum(hit[t].values()) / max(T, 1), 4) for t in ("W", "K", "M")},
           "per_class": {n: {"visible": tot[n], "W": hit["W"][n], "K": hit["K"][n], "M": hit["M"][n]}
                         for n in names if tot[n]}}
    json.dump(res, open(os.path.join(OUT, "metrics", "yoloe_compare.json"), "w"), indent=2, ensure_ascii=False)
    print("\n=== 박스-존재 recall (visible cell %d) ===" % T)
    for t, lab in (("W", "YOLO-World"), ("K", f"YOLOE multi-K{a.K}"), ("M", "YOLOE montage12")):
        print(f"  {lab:20s} {res['recall'][t]:.3f}")
    print(f"\n{'class':22s} {'vis':>4s} {'YW':>4s} {'K':>4s} {'M':>4s}")
    for n in names:
        if tot[n]: print(f"{n:22s} {tot[n]:>4d} {hit['W'][n]:>4d} {hit['K'][n]:>4d} {hit['M'][n]:>4d}")
    print(f"-> {OUT}")


if __name__ == "__main__":
    main()
