#!/usr/bin/env python3
"""run_fn_counterfactuals.py — CF-0/CF-4 + best-ROI oracle (GT-free 인과 분리).

CF-0  실제 재현 (622/86/313)
CF-4  gate bypass (semantic/appe/HSV/pairwise/all) → TP복구·FP재유입·F1
oracle best-ROI (GPU): FN cell 프레임의 전 per-prompt proposal(class-agnostic union, conf>=0.02)을
       target object 템플릿으로 ISM 스코어 → gate 통과 ROI 존재?
         존재+운영 미선택 → selection-causal 재분류
         없음 → gate/feature-limited (gate-causal 강)
CF-1/2/3·IoU recall: GT bbox 부재 → not_available (fabricate 금지).

산출: csv/counterfactual_results.csv, csv/fn_oracle_reclass.csv,
      metrics/counterfactual_metrics.json
"""
import argparse, csv, json, os, sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import phase21_common as pc
sys.path.insert(0, pc.REPO)
import cv2, torch
import yolo_ism as yi, yolo_ism_object_n as o_n, ism_hsv   # noqa

OUTC = os.path.join(pc.OUT, "csv"); OUTM = os.path.join(pc.OUT, "metrics")
RAW = os.path.join(pc.REPO, "outputs", "phase2_appearance_semantic_yolo_bbox", "raw", "yolo_raw")
FRAMES = pc.FRAMES; T = pc.T


def cf_grid(gt, cells, bypass):
    """bypass: set of {'S','A','H'}. accept = (passS or 'S'in bypass) and ... and has best."""
    TP = FP = FN = TN = 0
    for (ds, f), vis in gt.items():
        for o in pc.OBJECTS:
            c = cells.get((ds, f, o)); v = o in vis
            if not c or c["best"] is None:
                acc = False
            else:
                b = c["best"]
                acc = ((b["passS"] or "S" in bypass) and (b["passA"] or "A" in bypass)
                       and (b["passH"] or "H" in bypass))
            if v and acc: TP += 1
            elif acc: FP += 1
            elif v: FN += 1
            else: TN += 1
    P = TP / max(TP + FP, 1); R = TP / max(TP + FN, 1)
    return dict(TP=TP, FP=FP, FN=FN, TN=TN, precision=round(P, 4), recall=round(R, 4),
                f1=round(2 * P * R / max(P + R, 1e-9), 4))


def best_roi_oracle(gt, cells, PROTO):
    """FN cell(gate/selection 후보 있는 것)에 대해 프레임 전 per-prompt proposal을 스코어."""
    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, False)
    seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH); by = {o["name"]: o for o in objs}
    for o in objs:
        o["_tcls"] = o["tcls"].to(device)
        o["_flat11"] = torch.cat(o["tappe_blocks"][11], 0).to(device)
        o["_seg"] = torch.cat([torch.full((t.shape[0],), i, dtype=torch.long)
                               for i, t in enumerate(o["tappe_blocks"][11])]).to(device)
        o["_nview"] = len(o["tappe_blocks"][11])
        hp = os.path.join(os.path.dirname(o["cls_cache"]), f"{o['name']}_hsv.npz")
        o["_hsv"] = ism_hsv.load_cache(hp, o["template_dir"])[0]
    PROTO_D = ism_hsv.build_reference(by["Dinosaur"]["template_dir"], 9)[0]

    # FN cell 대상 (후보 있는 것: gate/selection). 프레임별 union box.
    target = {k: c for k, c in cells.items()
              if k[2] in gt.get((k[0], k[1]), set()) and not c["accept"] and c["best"] is not None}
    frames_needed = set((k[0], k[1]) for k in target)
    # 프레임별 전 per-prompt(960) box union (conf>=0.02, class-agnostic)
    fr_boxes = defaultdict(list)
    for ds in pc.DATASETS:
        p = os.path.join(RAW, "perprompt960", f"{ds}.csv")
        for r in csv.DictReader(open(p)):
            key = (ds, int(r["frame_id"]))
            if key in frames_needed and float(r["conf"]) >= 0.02:
                fr_boxes[key].append((int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"])))

    reclass = {}
    for (ds, fr) in sorted(frames_needed):
        fp = os.path.join(FRAMES, ds, f"frame_{fr:06d}.png")
        bgr = cv2.imread(fp)
        if bgr is None:
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB); norm = yi.normalize_rgb(rgb)
        # unique boxes
        ub = sorted(set(fr_boxes.get((ds, fr), [])))
        if not ub:
            continue
        crops, keep = [], []
        for bx in ub:
            c = yi.crop_resize_pad(norm, list(bx))
            if c is not None: crops.append(c); keep.append(bx)
        if not crops: continue
        cls_all, patch_all = o_n.dinov2_blocks_forward(model, crops, device, [11])
        masks = yi.segment_boxes(seg, bgr, [list(b) for b in keep], device)
        # 이 프레임의 각 target object에 대해 전 box 스코어
        objs_here = [k[2] for k in target if k[0] == ds and k[1] == fr]
        for obj in objs_here:
            o = by[obj]; proto = PROTO_D if obj == "Dinosaur" else o["_hsv"]
            best = None
            for ci, bx in enumerate(keep):
                sims = (o["_tcls"] @ cls_all[ci].to(device)).cpu()
                sem = float(torch.topk(sims, min(5, sims.numel())).values.mean())
                t1 = int(torch.argmax(sims)); x1, y1, x2, y2 = bx; mask = masks[ci]
                qp = patch_all[11][ci]
                fg = yi.masked_query_patches(qp.cpu(), mask, list(bx), pool)[0].to(device) if mask is not None else qp
                if fg.shape[0] == 0:
                    appe = 0.0
                else:
                    sim = fg @ o["_flat11"].T
                    per = torch.full((fg.shape[0], o["_nview"]), -1.0, device=device, dtype=sim.dtype)
                    per.scatter_reduce_(1, o["_seg"].unsqueeze(0).expand(fg.shape[0], -1), sim, reduce="amax")
                    appe = float(per.mean(0).clamp(0, 1)[t1])
                mc = mask[y1:y2, x1:x2] if mask is not None else None
                if proto is not None and mc is not None and mc.sum() >= 1:
                    try: hv = float(ism_hsv.similarity(ism_hsv.query_hist(bgr[y1:y2, x1:x2], mc.astype(np.uint8)), proto))
                    except Exception: hv = 1.0
                else: hv = 1.0
                passall = (sem >= float(o["similarity_threshold"]) and appe >= float(o["appe_gate"]) and hv >= T)
                cand = dict(bx=bx, sem=sem, appe=appe, hsv=hv, passall=passall)
                if best is None or sem > best["sem"]:
                    best = cand
                if passall and (best is None or not best["passall"]):
                    best = cand
            any_pass = any(True for _ in [best] if best and best["passall"])
            reclass[(ds, fr, obj)] = dict(oracle_best_sem=round(best["sem"], 4) if best else 0.0,
                                          oracle_best_appe=round(best["appe"], 4) if best else 0.0,
                                          oracle_best_hsv=round(best["hsv"], 4) if best else 0.0,
                                          oracle_pass=int(bool(best and best["passall"])),
                                          n_boxes=len(keep))
    return reclass


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--with-gpu", action="store_true"); a = ap.parse_args()
    gt = pc.load_gt(); PROTO = pc.build_protos(); cells = pc.build_cells(gt, PROTO)

    CFS = {
        "CF0_actual": set(),
        "CF4_semantic_bypass": {"S"}, "CF4_appearance_bypass": {"A"}, "CF4_hsv_bypass": {"H"},
        "CF4_sem+appe_bypass": {"S", "A"}, "CF4_appe+hsv_bypass": {"A", "H"},
        "CF4_all_gates_bypass": {"S", "A", "H"},
    }
    base = cf_grid(gt, cells, set())
    results = {}
    for name, byp in CFS.items():
        m = cf_grid(gt, cells, byp)
        m["FN_recovered"] = base["FN"] - m["FN"]; m["FP_readmitted"] = m["FP"] - base["FP"]
        results[name] = m

    os.makedirs(OUTC, exist_ok=True); os.makedirs(OUTM, exist_ok=True)

    oracle = {}
    if a.with_gpu:
        oracle = best_roi_oracle(gt, cells, PROTO)
        # 재분류: gate-FN 중 oracle_pass=1 → selection-causal(사용가능 ROI 존재), else gate/feature-limited
        sel_reclass = sum(1 for v in oracle.values() if v["oracle_pass"] == 1)
        with open(os.path.join(OUTC, "fn_oracle_reclass.csv"), "w", newline="") as f:
            w = csv.writer(f); w.writerow(["dataset", "frame_id", "object", "oracle_best_sem",
                                           "oracle_best_appe", "oracle_best_hsv", "oracle_pass",
                                           "n_boxes", "reclass"])
            for (ds, fr, o), v in sorted(oracle.items()):
                rc = "selection_causal(usable ROI exists)" if v["oracle_pass"] else "gate_feature_limited"
                w.writerow([ds, fr, o, v["oracle_best_sem"], v["oracle_best_appe"],
                            v["oracle_best_hsv"], v["oracle_pass"], v["n_boxes"], rc])
        results["_oracle"] = {"gate_fn_evaluated": len(oracle),
                              "usable_roi_exists(selection_causal)": sel_reclass,
                              "gate_feature_limited": len(oracle) - sel_reclass}

    with open(os.path.join(OUTC, "counterfactual_results.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["configuration", "TP", "FP", "FN", "precision", "recall",
                                       "F1", "FN_recovered", "FP_readmitted", "note"])
        for name, m in results.items():
            if name.startswith("_"): continue
            w.writerow([name, m["TP"], m["FP"], m["FN"], m["precision"], m["recall"], m["f1"],
                        m.get("FN_recovered", 0), m.get("FP_readmitted", 0), ""])
        # deferred rows (GT bbox 필요)
        for nm in ["CF1_GT_bbox_oracle", "CF2_GT_bbox+GT_mask", "CF3_YOLO_bestIoU_oracle",
                   "Recall@IoU0.5", "Recall@IoU0.7"]:
            w.writerow([nm, "not_available", "", "", "", "", "", "", "requires human GT bbox (deferred)"])

    out = {"cf0_baseline": base, "counterfactuals": {k: v for k, v in results.items() if not k.startswith("_")},
           "oracle": results.get("_oracle", {"status": "skipped (no --with-gpu)"}),
           "deferred": {"CF1/CF2/CF3/IoU_recall": "requires human GT bbox — Option1 defers; annotation CSV emitted"},
           "notes": {"cf4_all_gates_bypass_FN": "= proposal-miss floor (후보 없는 cell)",
                     "gate_causal_effect": "각 gate의 TP복구 vs FP재유입"}}
    json.dump(out, open(os.path.join(OUTM, "counterfactual_metrics.json"), "w"), indent=2, ensure_ascii=False)

    print("=== CF-0/CF-4 반사실 ===")
    for name, m in results.items():
        if name.startswith("_"): continue
        print(f"  {name:26s} TP {m['TP']} FP {m['FP']} FN {m['FN']} F1 {m['f1']} "
              f"(FN복구 {m.get('FN_recovered',0)}, FP재유입 {m.get('FP_readmitted',0)})")
    if "_oracle" in results:
        print("=== best-ROI oracle (gate-FN 재분류) ===", results["_oracle"])
    print(f"-> {OUTM}/counterfactual_metrics.json")


if __name__ == "__main__":
    main()
