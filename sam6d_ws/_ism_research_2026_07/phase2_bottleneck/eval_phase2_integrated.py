#!/usr/bin/env python3
"""eval_phase2_integrated.py — P0~P5 통합 평가 + 이득 중복 분석.

P0  Phase 1C 운영 기준선 (cur=per-prompt 후보, cur hsv.npz 로 정확 재현: 622/86/.7572)
P1  P0 + 최선 Appearance (block9 게이트, LODO threshold)
P2  P0 + 최선 Semantic (B에서 채택 없음 → P0와 동일; 기록만)
P3  P0 + 최선 YOLO 후보 (GPU: shared960 / perprompt1280 후보를 동일 게이트로 최종 평가)
P4  P0 + block9 + semantic(=block9)
P5  P0 + block9 + YOLO 후보

산출: csv/integrated_pipeline_metrics.csv, csv/phase2_per_candidate_debug.csv,
      metrics/integrated_metrics.json
"""
import csv, json, os, sys
from collections import defaultdict
import numpy as np
import cv2, torch
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import phase2_common as pc
sys.path.insert(0, pc.REPO)
import yolo_ism as yi, yolo_ism_object_n as o_n, ism_hsv   # noqa

T = 0.1214; TOPK = 3
OUTC = os.path.join(pc.OUT, "csv"); OUTM = os.path.join(pc.OUT, "metrics")
CUR = pc.CUR; FRAMES = os.path.join(pc.RSRCH, "gt_input", "frames")
RAW = os.path.join(pc.OUT, "raw", "yolo_raw")


# ---------------- P0 정확 재현 (eval_phase1c HSV 경로: cur hsv.npz 슬라이스) ----------------
def build_p0(gt):
    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    tdir = {o["name"]: o["template_dir"] for o in objs}
    PROTO = {}
    for o in objs:
        PROTO[o["name"]] = None if o["name"] == "Dinosaur" else ism_hsv.load_cache(
            os.path.join(os.path.dirname(o["cls_cache"]), f"{o['name']}_hsv.npz"), o["template_dir"])[0]
    PROTO["Dinosaur"] = ism_hsv.build_reference(tdir["Dinosaur"], 9)[0]
    MASKED = slice(0, 224); HS = slice(96, 224)
    rec = {}
    for ds in pc.DATASETS:
        z = np.load(os.path.join(CUR, f"{ds}_hsv.npz"))
        hq = {str(u): h[MASKED][HS].astype(np.float64) for u, h in zip(z["uid"], z["hist"])}
        rows = defaultdict(list)
        for r in csv.DictReader(open(os.path.join(CUR, f"{ds}_pairs.csv"))):
            if (ds, int(r["frame_id"])) not in gt: continue
            rows[(ds, int(r["frame_id"]), r["object"])].append(r)
        for k, cs in rows.items():
            v = [c for c in cs if int(c["routed"]) == 1 and float(c["yolo_conf"]) >= 0.02]
            v.sort(key=lambda c: -float(c["yolo_conf"])); v = v[:TOPK]
            if not v: continue
            b = max(v, key=lambda c: float(c["sem_top5"]))
            rec[k] = dict(sem=float(b["sem_top5"]), appe=float(b["appe11_clstop1"]),
                          appe9=None, q=hq.get(b["uid"]), obj=k[2], uid=b["uid"],
                          sim_thr=float(b["sim_thr"]), appe_gate=float(b["appe_gate"]))
    return rec, PROTO


def hsv_of(rec_row, proto):
    return ism_hsv.similarity(rec_row["q"], proto) if rec_row["q"] is not None else 1.0


def metrics_from_accept(gt, accept):
    TP = FP = FN = TN = 0; per = defaultdict(lambda: [0, 0, 0, 0])
    for (ds, f), vis in gt.items():
        for o in pc.OBJECTS:
            a = accept(ds, f, o); v = o in vis
            i = 0 if (v and a) else 1 if a else 2 if v else 3
            per[o][i] += 1
            if i == 0: TP += 1
            elif i == 1: FP += 1
            elif i == 2: FN += 1
            else: TN += 1
    P = TP / max(TP + FP, 1); R = TP / max(TP + FN, 1)
    f1 = 2 * P * R / max(P + R, 1e-9)
    return {"TP": TP, "FP": FP, "FN": FN, "TN": TN, "precision": round(P, 4),
            "recall": round(R, 4), "f1": round(f1, 4), "per_class": {k: list(v) for k, v in per.items()}}


# ---------------- block9 LODO 게이트 (cur 후보, phase2_candidates.csv) ----------------
def block9_gate(gt, rows):
    sel = pc.build_selected(rows, gt)
    pop = [(k, b) for k, b in sel.items() if b is not None]
    # pooled LODO threshold 선택
    def score(b): return b["appe9_clstop1"]
    folds = {}
    for test_ds, train in pc.lodo_folds():
        s, y = [], []
        for (ds, fr, o), b in pop:
            if ds not in train: continue
            if b["sem_top5"] < b["sim_thr"]: continue
            s.append(score(b)); y.append(1 if o in gt[(ds, fr)] else 0)
        thr, _ = pc.best_f1_threshold(np.array(s), np.array(y)); folds[test_ds] = thr
    def accept(ds, fr, o):
        b = sel.get((ds, fr, o))
        if not b: return False
        if b["sem_top5"] < b["sim_thr"]: return False
        if score(b) < folds[ds]: return False
        return b["hsv_score"] >= T
    return accept, folds, sel


# ---------------- P3: alt 후보(YOLO raw config) GPU 스코어링 → 게이트 ----------------
def score_alt_config(cfg, gt):
    """yolo_raw/<cfg> 박스를 (frame,object) top3 conf 선택 → DINOv2/HSV → 게이트, 최종 지표."""
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

    # (ds,frame,object) -> top3 conf boxes
    sel = defaultdict(list)
    for ds in pc.DATASETS:
        p = os.path.join(RAW, cfg, f"{ds}.csv")
        for r in csv.DictReader(open(p)):
            if r["object"] == "" or float(r["conf"]) < 0.02: continue
            if (ds, int(r["frame_id"])) not in gt: continue
            sel[(ds, int(r["frame_id"]), r["object"])].append(r)
    chosen = {}
    for k, cs in sel.items():
        cs.sort(key=lambda r: -float(r["conf"])); chosen[k] = cs[:TOPK]

    # frame 단위로 묶어 forward
    byframe = defaultdict(list)
    for (ds, fr, o), cs in chosen.items():
        for r in cs:
            byframe[(ds, fr)].append((o, (int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"])), float(r["conf"])))
    best = {}   # (ds,fr,o) -> dict(sem,appe11,hsv)
    for (ds, fr), items in byframe.items():
        fp = os.path.join(FRAMES, ds, f"frame_{fr:06d}.png")
        bgr = cv2.imread(fp)
        if bgr is None: continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB); norm = yi.normalize_rgb(rgb)
        boxes = [it[1] for it in items]
        crops, keep = [], []
        for i, bx in enumerate(boxes):
            c = yi.crop_resize_pad(norm, list(bx))
            if c is not None: crops.append(c); keep.append(i)
        if not crops: continue
        cls_all, patch_all = o_n.dinov2_blocks_forward(model, crops, device, [11])
        masks = yi.segment_boxes(seg, bgr, [list(boxes[i]) for i in keep], device)
        for ci, i in enumerate(keep):
            obj, bx, conf = items[i]; o = by[obj]
            sims = (o["_tcls"] @ cls_all[ci].to(device)).cpu()
            k = min(5, sims.numel()); sem = float(torch.topk(sims, k).values.mean())
            t_top1 = int(torch.argmax(sims))
            x1, y1, x2, y2 = bx; mask = masks[ci]
            qp = patch_all[11][ci]
            fg = yi.masked_query_patches(qp.cpu(), mask, list(bx), pool)[0].to(device) if mask is not None else qp
            if fg.shape[0] == 0:
                appe = 0.0
            else:
                sim = fg @ o["_flat11"].T
                per = torch.full((fg.shape[0], o["_nview"]), -1.0, device=device, dtype=sim.dtype)
                per.scatter_reduce_(1, o["_seg"].unsqueeze(0).expand(fg.shape[0], -1), sim, reduce="amax")
                appe = float(per.mean(0).clamp(0, 1)[t_top1])
            mc = mask[y1:y2, x1:x2] if mask is not None else None
            if o["_hsv"] is not None and mc is not None and mc.sum() >= 1:
                try: hsv = float(ism_hsv.similarity(ism_hsv.query_hist(bgr[y1:y2, x1:x2], mc.astype(np.uint8)), o["_hsv"]))
                except Exception: hsv = 1.0
            else: hsv = 1.0
            key = (ds, fr, obj)
            if key not in best or sem > best[key]["sem"]:
                best[key] = {"sem": sem, "appe": appe, "hsv": hsv,
                             "sim_thr": float(o["similarity_threshold"]), "appe_gate": float(o["appe_gate"])}
    def accept(ds, fr, o):
        b = best.get((ds, fr, o))
        if not b: return False
        if b["sem"] < b["sim_thr"] or b["appe"] < b["appe_gate"]: return False
        return b["hsv"] >= T
    return accept, best


def main():
    import argparse
    ap = argparse.ArgumentParser(); ap.add_argument("--with-gpu", action="store_true",
        help="P3 alt-후보 GPU 스코어링 수행(shared960, perprompt1280)")
    a = ap.parse_args()
    gt = pc.load_gt(); rows = pc.load_candidates()

    # P0 정확 재현
    rec, PROTO = build_p0(gt)
    def acc_P0(ds, fr, o):
        r = rec.get((ds, fr, o))
        if not r: return False
        if not (r["sem"] >= r["sim_thr"] and r["appe"] >= r["appe_gate"]): return False
        return hsv_of(r, PROTO[o]) >= T
    P0 = metrics_from_accept(gt, acc_P0)

    # P1 block9
    acc_P1, folds9, sel = block9_gate(gt, rows)
    P1 = metrics_from_accept(gt, acc_P1)
    P2 = dict(P0); P2["note"] = "semantic 채택 후보 없음 → P0와 동일"
    P4 = dict(P1); P4["note"] = "P1+semantic(=P1, semantic 변경 없음)"

    results = {"P0_phase1c_baseline": P0, "P1_block9_appe": P1,
               "P2_best_semantic(keep)": P2, "P4_block9+semantic": P4}

    # FP/FN 중복 분석: P0 vs P1 어떤 셀이 바뀌나
    def cellset(acc):
        fp = set(); fn = set()
        for (ds, f), vis in gt.items():
            for o in pc.OBJECTS:
                a = acc(ds, f, o); v = o in vis
                if a and not v: fp.add((ds, f, o))
                if v and not a: fn.add((ds, f, o))
        return fp, fn
    fp0, fn0 = cellset(acc_P0); fp1, fn1 = cellset(acc_P1)
    overlap = {"P0_FP": len(fp0), "P1_FP": len(fp1), "FP_removed_by_block9": len(fp0 - fp1),
               "FP_new_block9": len(fp1 - fp0), "P0_FN": len(fn0), "P1_FN": len(fn1),
               "FN_new_block9": len(fn1 - fn0)}

    # P3/P5 (GPU)
    if a.with_gpu:
        for cfg in ("shared960", "perprompt1280"):
            acc, best = score_alt_config(cfg, gt)
            m = metrics_from_accept(gt, acc)
            results[f"P3_altcand_{cfg}"] = {k: v for k, v in m.items() if k != "per_class"}
            # P5 = block9 gate ON alt candidates
            def acc5(ds, fr, o, best=best):
                b = best.get((ds, fr, o))
                if not b: return False
                # block9 미보유(alt는 block11만) → block9 조합은 cur에서만 유효. 여기선 표기만.
                return acc(ds, fr, o)
        results["_note_P3"] = "alt 후보는 block11 only 스코어링; P5(block9+alt)는 cur 한정 유효"

    os.makedirs(OUTC, exist_ok=True); os.makedirs(OUTM, exist_ok=True)
    with open(os.path.join(OUTC, "integrated_pipeline_metrics.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["config", "TP", "FP", "FN", "TN", "precision", "recall", "F1", "note"])
        for k, m in results.items():
            if not isinstance(m, dict) or "TP" not in m: continue
            w.writerow([k, m["TP"], m["FP"], m["FN"], m.get("TN", ""), m["precision"], m["recall"],
                        m["f1"], m.get("note", "")])

    json.dump({"results": {k: ({kk: vv for kk, vv in v.items() if kk != "per_class"}
                               if isinstance(v, dict) else v) for k, v in results.items()},
               "overlap_P0_vs_P1": overlap, "block9_folds": {k: round(v, 4) for k, v in folds9.items()},
               "notes": {"P0": "cur(per-prompt) + cur hsv.npz 정확 재현",
                         "candidate_recall_cur": "0.9594 (per-prompt); shared-pass main=0.826",
                         "bottleneck": "cur 후보recall .959 → 최종 recall .665 = 게이트가 참후보 상당수 제거"}},
              open(os.path.join(OUTM, "integrated_metrics.json"), "w"), indent=2, ensure_ascii=False)

    print("=== 통합 P 구성 ===")
    for k, m in results.items():
        if isinstance(m, dict) and "TP" in m:
            print(f"  {k:26s} TP {m['TP']} FP {m['FP']} FN {m['FN']} F1 {m['f1']}  {m.get('note','')}")
    print("=== 이득 중복(P0 vs P1 block9) ===", overlap)
    print(f"-> {OUTM}/integrated_metrics.json")


if __name__ == "__main__":
    main()
