#!/usr/bin/env python3
"""eval_prompt_ab.py — 프롬프트 A/B (현행 vs v2) 전체 파이프라인 평가 (READ-ONLY).

A: 현행 config 프롬프트   B: prompts_v2_preregistered.yaml (평가 전 고정)
둘 다 per-prompt 패스(imgsz960, conf0.02, top_k3) → DINOv2 semantic → MobileSAM →
masked-appe → HSV(Phase 1C) 로 동일하게 통과시키고 grid TP/FP/FN 을 비교한다.
후단 게이트·템플릿·임계는 전부 불변. 프롬프트만 바뀐다.

판정: FP<=86 유지하며 TP 증가 여부 + 클래스별 회귀 확인.
산출: outputs/phase26_prompt_ab/{csv,metrics}/
"""
import argparse, csv, json, os, sys
from collections import defaultdict
import numpy as np
import cv2, torch, yaml

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, os.path.join(RSRCH, "phase21_fn_audit")); sys.path.insert(0, REPO)
import phase21_common as p21
import yolo_ism as yi, yolo_ism_object_n as o_n, ism_hsv

OUT = os.path.join(REPO, "outputs", "phase26_prompt_ab")
FRAMES = os.path.join(RSRCH, "gt_input", "frames")
PRE = os.path.join(HERE, "prompts_v2_preregistered.yaml")
T = p21.T; TOPK = 3; CONF = 0.02; IMGSZ = 960


def build_models():
    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    dev = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, dev)
    objs = o_n.prepare_objects(objs, model, dev, False)
    seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), dev)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    for o in objs:
        o["_tcls"] = o["tcls"].to(dev)
        o["_flat11"] = torch.cat(o["tappe_blocks"][11], 0).to(dev)
        o["_seg"] = torch.cat([torch.full((t.shape[0],), i, dtype=torch.long)
                               for i, t in enumerate(o["tappe_blocks"][11])]).to(dev)
        o["_nview"] = len(o["tappe_blocks"][11])
        hp = os.path.join(os.path.dirname(o["cls_cache"]), f"{o['name']}_hsv.npz")
        o["_hsv"] = ism_hsv.load_cache(hp, o["template_dir"])[0]
    return defaults, objs, model, seg, pool, dev


def run_variant(tag, prompt_of, gt, defaults, objs, model, seg, pool, dev):
    """프롬프트 세트로 YOLO→ISM 전체 실행 → (ds,fr,obj)->best 후보 점수."""
    from ultralytics import YOLOWorld
    W = defaults.get("weights", "yolov8m-worldv2.pt")
    ycache = {}
    by = {o["name"]: o for o in objs}
    best = {}
    frames = sorted(gt.keys())
    for n, (ds, fr) in enumerate(frames, 1):
        fp = os.path.join(FRAMES, ds, f"frame_{fr:06d}.png")
        bgr = cv2.imread(fp)
        if bgr is None:
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB); norm = yi.normalize_rgb(rgb)
        H, W_ = bgr.shape[:2]
        # 객체별 per-prompt 패스 → top_k 후보
        items = []
        for o in objs:
            pr = prompt_of[o["name"]]
            if pr not in ycache:
                y = YOLOWorld(W); y.set_classes([pr]); ycache[pr] = y
            r = ycache[pr].predict(bgr, conf=CONF, imgsz=IMGSZ, verbose=False, device=dev)
            bx = []
            if len(r) and r[0].boxes is not None and len(r[0].boxes):
                b = r[0].boxes
                for j in range(len(b)):
                    xy = b.xyxy[j].tolist()
                    x1 = max(0, min(int(xy[0]), W_ - 1)); y1 = max(0, min(int(xy[1]), H - 1))
                    x2 = max(x1 + 1, min(int(xy[2]), W_)); y2 = max(y1 + 1, min(int(xy[3]), H))
                    bx.append(((x1, y1, x2, y2), float(b.conf[j])))
            bx.sort(key=lambda t: -t[1])
            for box, cf in bx[:TOPK]:
                items.append((o["name"], box, cf))
        if not items:
            continue
        crops, keep = [], []
        for i, (nm, box, cf) in enumerate(items):
            c = yi.crop_resize_pad(norm, list(box))
            if c is not None:
                crops.append(c); keep.append(i)
        if not crops:
            continue
        cls_all, patch_all = o_n.dinov2_blocks_forward(model, crops, dev, [11])
        masks = yi.segment_boxes(seg, bgr, [list(items[i][1]) for i in keep], dev)
        for ci, i in enumerate(keep):
            nm, box, cf = items[i]; o = by[nm]
            sims = (o["_tcls"] @ cls_all[ci].to(dev)).cpu()
            sem = float(torch.topk(sims, min(5, sims.numel())).values.mean())
            t1 = int(torch.argmax(sims)); x1, y1, x2, y2 = box; mask = masks[ci]
            qp = patch_all[11][ci]
            fg = yi.masked_query_patches(qp.cpu(), mask, list(box), pool)[0].to(dev) if mask is not None else qp
            if fg.shape[0] == 0:
                appe = 0.0
            else:
                sim = fg @ o["_flat11"].T
                per = torch.full((fg.shape[0], o["_nview"]), -1.0, device=dev, dtype=sim.dtype)
                per.scatter_reduce_(1, o["_seg"].unsqueeze(0).expand(fg.shape[0], -1), sim, reduce="amax")
                appe = float(per.mean(0).clamp(0, 1)[t1])
            mc = mask[y1:y2, x1:x2] if mask is not None else None
            proto = o["_hsv"]
            if proto is not None and mc is not None and mc.sum() >= 1:
                try:
                    hsv = float(ism_hsv.similarity(ism_hsv.query_hist(bgr[y1:y2, x1:x2],
                                                                     mc.astype(np.uint8)), proto))
                except Exception:
                    hsv = 1.0
            else:
                hsv = 1.0
            k = (ds, fr, nm)
            if k not in best or sem > best[k]["sem"]:
                best[k] = dict(sem=sem, appe=appe, hsv=hsv, conf=cf, box=box,
                               st=float(o["similarity_threshold"]), ag=float(o["appe_gate"]))
        if n % 40 == 0:
            print(f"    [{tag}] {n}/{len(frames)} frames")
    return best


def grid(gt, best):
    TP = FP = FN = 0; per = defaultdict(lambda: [0, 0, 0])
    for (ds, fr), vis in gt.items():
        for o in p21.OBJECTS:
            b = best.get((ds, fr, o))
            acc = bool(b) and b["sem"] >= b["st"] and b["appe"] >= b["ag"] and b["hsv"] >= T
            v = o in vis
            if v and acc: TP += 1; per[o][0] += 1
            elif acc: FP += 1; per[o][1] += 1
            elif v: FN += 1; per[o][2] += 1
    P = TP / max(TP + FP, 1); R = TP / max(TP + FN, 1)
    return dict(TP=TP, FP=FP, FN=FN, precision=round(P, 4), recall=round(R, 4),
                f1=round(2 * P * R / max(P + R, 1e-9), 4), per=dict(per))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--variants", nargs="+", default=["A", "B"])
    a = ap.parse_args()
    os.makedirs(os.path.join(OUT, "csv"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "metrics"), exist_ok=True)
    gt = p21.load_gt()
    defaults, objs, model, seg, pool, dev = build_models()
    cur = {o["name"]: o["yolo_prompt"] for o in objs}
    v2 = yaml.safe_load(open(PRE))["prompts_v2"]
    sets = {"A": cur, "B": {**cur, **v2}}
    res = {}
    for tag in a.variants:
        print(f"  === variant {tag} 실행 ===")
        best = run_variant(tag, sets[tag], gt, defaults, objs, model, seg, pool, dev)
        res[tag] = grid(gt, best)
        print(f"    {tag}: TP {res[tag]['TP']} FP {res[tag]['FP']} FN {res[tag]['FN']} F1 {res[tag]['f1']}")
    with open(os.path.join(OUT, "csv", "prompt_ab_summary.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["variant", "TP", "FP", "FN", "precision", "recall", "F1"])
        for t, m in res.items():
            w.writerow([t, m["TP"], m["FP"], m["FN"], m["precision"], m["recall"], m["f1"]])
    with open(os.path.join(OUT, "csv", "prompt_ab_by_class.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["class"] + [f"{t}_{k}" for t in res for k in ("TP", "FP", "FN")])
        for o in p21.OBJECTS:
            row = [o]
            for t in res:
                row += res[t]["per"].get(o, [0, 0, 0])
            w.writerow(row)
    json.dump({t: {k: v for k, v in m.items() if k != "per"} for t, m in res.items()},
              open(os.path.join(OUT, "metrics", "prompt_ab.json"), "w"), indent=2, ensure_ascii=False)
    print("\n=== 프롬프트 A/B 결과 ===")
    for t, m in res.items():
        print(f"  {t}: TP {m['TP']} FP {m['FP']} FN {m['FN']} P {m['precision']} R {m['recall']} F1 {m['f1']}")
    if "A" in res and "B" in res:
        dT = res["B"]["TP"] - res["A"]["TP"]; dF = res["B"]["FP"] - res["A"]["FP"]
        print(f"  Δ(B-A): TP {dT:+d}  FP {dF:+d}  F1 {res['B']['f1']-res['A']['f1']:+.4f}")
        print(f"  판정(FP<=86 & TP↑): {'PASS' if res['B']['FP']<=86 and dT>0 else 'FAIL'}")
        print("\n  === 클래스별 ΔTP / ΔFP ===")
        for o in p21.OBJECTS:
            pa = res["A"]["per"].get(o, [0, 0, 0]); pb = res["B"]["per"].get(o, [0, 0, 0])
            if pb[0] - pa[0] or pb[1] - pa[1]:
                print(f"    {o:22s} ΔTP {pb[0]-pa[0]:+3d}  ΔFP {pb[1]-pa[1]:+3d}")
    print(f"-> {OUT}")


if __name__ == "__main__":
    main()
