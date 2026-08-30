#!/usr/bin/env python3
"""eval_and_render.py — appe 집계 방식(mean/min/max/AND) A/B (READ-ONLY, 오프라인 후보풀).

목적: '블록 집계를 어떻게 하면 choco(질감으로 갈림)와 인형(색으로 갈림)을 동시에
      살릴 수 있는가'를 사람이 눈으로 판단하도록 실제 프레임에 결과를 그린다.

방법(전 객체 공통·훈련없음):
  mean = (appe2+appe9)/2   <- 현재 승격값
  min  = min(appe2,appe9)
  max  = max(appe2,appe9)
  AND  = appe2>=g2 AND appe9>=g9   (2문턱)
각 방식의 gate 는 959 보정 GT 위에서 F1 최대가 되는 '단일' operating point 로 고정.
semantic(0.35)/HSV(0.1214)는 전 방식 동일. 선택 후보 = 파이프라인 is_selected.

주의: 후보풀은 per-prompt YOLO(오프라인)라 절대수치는 배포(shared-pass)와 다르다.
      여기서 비교하는 것은 '집계 방식'이라는 한 변수뿐이다.
산출: outputs/phase36_appe_agg/{viz,csv,metrics}/
"""
import csv, json, os, sys
from collections import defaultdict
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, os.path.join(RSRCH, "phase21_fn_audit"))
import phase21_common as p21

OUT = os.path.join(REPO, "outputs", "phase36_appe_agg"); VIZ = os.path.join(OUT, "viz")
FRAMES = os.path.join(RSRCH, "gt_input", "frames")
POOL = os.path.join(REPO, "outputs", "phase22_training_free_gate", "csv", "candidate_pool.csv")
FPAUDIT = os.path.join(REPO, "outputs", "phase32_fp_audit", "fp_audit_task.csv")


def corrected_gt():
    gt = {k: set(v) for k, v in p21.load_gt().items()}
    if os.path.isfile(FPAUDIT):
        for r in csv.DictReader(open(FPAUDIT)):
            if r["answer"].strip().upper() == "T":
                gt[(r["bag_name"], int(r["frame_id"]))].add(r["class_name"])
    return gt


def load_cells():
    cells = defaultdict(list)
    for r in csv.DictReader(open(POOL)):
        x1, y1, x2, y2 = [int(v) for v in r["uid"].rsplit("|", 1)[1].split("_")]
        cells[(r["dataset"], int(r["frame_id"]), r["object"])].append(dict(
            sem=float(r["sem_top5"]), a2=float(r["appe2"]), a9=float(r["appe9"]),
            hsv=float(r["hsv"]), st=float(r["sim_thr"]), ht=float(r["hsv_thr"]),
            sel=r["is_selected"] == "1", box=(x1, y1, x2, y2)))
    return cells


def sel(cs):
    for c in cs:
        if c["sel"]: return c
    return max(cs, key=lambda c: c["sem"])


AGG = {"mean": lambda c: (c["a2"] + c["a9"]) / 2,
       "min":  lambda c: min(c["a2"], c["a9"]),
       "max":  lambda c: max(c["a2"], c["a9"])}


def base_pass(c):
    return c["sem"] >= c["st"] and c["hsv"] >= c["ht"]


def metrics(gt, cells, decide):
    TP = FP = FN = 0; per = defaultdict(lambda: [0, 0, 0])
    for (ds, fr), vis in gt.items():
        acc = set()
        for o in p21.OBJECTS:
            cs = cells.get((ds, fr, o))
            if not cs: continue
            c = sel(cs)
            if base_pass(c) and decide(c):
                acc.add(o)
        for o in p21.OBJECTS:
            v, a = o in vis, o in acc
            if v and a: TP += 1; per[o][0] += 1
            elif a: FP += 1; per[o][1] += 1
            elif v: FN += 1; per[o][2] += 1
    P = TP / max(TP + FP, 1); R = TP / max(TP + FN, 1)
    return dict(TP=TP, FP=FP, FN=FN, precision=round(P, 4), recall=round(R, 4),
                f1=round(2 * P * R / max(P + R, 1e-9), 4)), per


def main():
    for d in ("viz", "csv", "metrics"):
        os.makedirs(os.path.join(OUT, d), exist_ok=True)
    gt = corrected_gt(); cells = load_cells()

    chosen = {}; met = {}
    # mean/min/max: 1D 스윕으로 F1 최대 단일 문턱
    for name, fn in AGG.items():
        best = None
        for g in np.arange(0.40, 0.80, 0.005):
            m, _ = metrics(gt, cells, lambda c, fn=fn, g=g: fn(c) >= g)
            if best is None or m["f1"] > best[0]["f1"]:
                best = (m, round(float(g), 3))
        chosen[name] = ("gate", best[1]); met[name] = best[0]
    # AND: 2D 그리드
    bestAND = None
    for g2 in np.arange(0.55, 0.72, 0.01):
        for g9 in np.arange(0.50, 0.66, 0.01):
            m, _ = metrics(gt, cells, lambda c, g2=g2, g9=g9: c["a2"] >= g2 and c["a9"] >= g9)
            if bestAND is None or m["f1"] > bestAND[0]["f1"]:
                bestAND = (m, (round(float(g2), 3), round(float(g9), 3)))
    chosen["AND"] = ("gates", bestAND[1]); met["AND"] = bestAND[0]

    # 현재 승격 mean@0.605 도 참조로
    m605, _ = metrics(gt, cells, lambda c: (c["a2"] + c["a9"]) / 2 >= 0.605)
    met["mean@0.605(현재)"] = m605

    DEC = {"mean": lambda c: (c["a2"] + c["a9"]) / 2 >= chosen["mean"][1],
           "min":  lambda c: min(c["a2"], c["a9"]) >= chosen["min"][1],
           "max":  lambda c: max(c["a2"], c["a9"]) >= chosen["max"][1],
           "AND":  lambda c: c["a2"] >= chosen["AND"][1][0] and c["a9"] >= chosen["AND"][1][1]}
    ORDER = ["mean", "min", "max", "AND"]

    man = []
    for (ds, fr) in sorted(gt.keys()):
        vis = gt[(ds, fr)]
        bgr = cv2.imread(os.path.join(FRAMES, ds, f"frame_{fr:06d}.png"))
        if bgr is None: continue
        acc = {}
        for name in ORDER:
            d = {}
            for o in p21.OBJECTS:
                cs = cells.get((ds, fr, o))
                if not cs: continue
                c = sel(cs)
                if base_pass(c) and DEC[name](c): d[o] = c["box"]
            acc[name] = d
        panels = [bgr]
        for name in ORDER:
            img = bgr.copy(); drawn = []
            for o, (x1, y1, x2, y2) in sorted(acc[name].items(),
                                              key=lambda kv: (kv[0] not in vis, kv[0])):
                col = (90, 220, 90) if o in vis else (60, 60, 235)
                ins = sum(5 for b in drawn if max(abs(b[0]-x1), abs(b[1]-y1),
                                                  abs(b[2]-x2), abs(b[3]-y2)) <= 4)
                drawn.append((x1, y1, x2, y2)); X1, Y1, X2, Y2 = x1+ins, y1+ins, x2-ins, y2-ins
                cv2.rectangle(img, (X1, Y1), (X2, Y2), col, 3)
                t = o.replace("_high", ""); ty = max(0, Y1-19)
                cv2.rectangle(img, (X1, ty), (X1+8*len(t), ty+19), col, -1)
                cv2.putText(img, t, (X1+2, ty+14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1)
            panels.append(img)
        fn = f"{ds}_{fr:06d}.png"; cv2.imwrite(os.path.join(VIZ, fn), np.hstack(panels))
        row = {"name": f"{ds}_{fr:06d}", "bag": ds, "frame": fr,
               "gt_visible": ";".join(sorted(vis)), "image": fn}
        base = set(acc["mean"])
        for name in ORDER:
            row[name] = ";".join(sorted(acc[name]))
        for name in ("min", "max", "AND"):
            s = set(acc[name])
            row[name+"_gainTP"] = ";".join(sorted((s-base) & vis))
            row[name+"_newFP"] = ";".join(sorted((s-base) - vis))
            row[name+"_lostTP"] = ";".join(sorted((base-s) & vis))
            row[name+"_rmFP"] = ";".join(sorted((base-s) - vis))
        row["changed"] = int(any(set(acc[n]) != base for n in ("min", "max", "AND")))
        man.append(row)
    with open(os.path.join(OUT, "csv", "agg_manifest.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(man[0].keys())); w.writeheader(); w.writerows(man)
    out = {"chosen": chosen, "metrics": met, "order": ORDER}
    json.dump(out, open(os.path.join(OUT, "metrics", "agg.json"), "w"), indent=2, ensure_ascii=False)
    print(f"{'method':18s} {'gate':>16s}  TP   FP   FN    F1")
    for name in ORDER:
        c = chosen[name]; g = c[1]; m = met[name]
        print(f"{name:18s} {str(g):>16s}  {m['TP']:>3d} {m['FP']:>3d} {m['FN']:>3d}  {m['f1']}")
    m = met["mean@0.605(현재)"]
    print(f"{'mean@0.605(현재)':18s} {'0.605':>16s}  {m['TP']:>3d} {m['FP']:>3d} {m['FN']:>3d}  {m['f1']}")
    print(f"변화 프레임 {sum(r['changed'] for r in man)}/{len(man)} -> {VIZ}")


if __name__ == "__main__":
    main()
