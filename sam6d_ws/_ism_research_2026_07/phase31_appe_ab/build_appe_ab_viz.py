#!/usr/bin/env python3
"""build_appe_ab_viz.py — appearance 특징 A/B 실제 검출 비교 렌더 (READ-ONLY).

패널: [원본 | A 현행 block11 | B1 block2+9 (LODO) | B2 block2+9 (최대F1)]
  A  : appe = block11,        gate 0.550
  B1 : appe = (b2+b9)/2,      gate 0.620   (LODO 선택, FP 보수적)
  B2 : appe = (b2+b9)/2,      gate 0.605   (최대 F1, TP 우선)
semantic(0.35) / HSV(0.1214) 는 모든 구성 동일.

박스 = 최종 ACCEPT (클래스명만 표기). 사람 검증 GT 로 TP/FP 를 색으로 구분:
  초록 = GT에 보이는 객체(TP)   빨강 = GT에 없는 객체(FP)
산출: outputs/phase31_appe_ab/{viz,csv,metrics}/
"""
import csv, json, os, sys
from collections import defaultdict
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, os.path.join(RSRCH, "phase21_fn_audit"))
import phase21_common as p21

OUT = os.path.join(REPO, "outputs", "phase31_appe_ab")
VIZ = os.path.join(OUT, "viz")
FRAMES = os.path.join(RSRCH, "gt_input", "frames")
POOL = os.path.join(REPO, "outputs", "phase22_training_free_gate", "csv", "candidate_pool.csv")
CFG = [("A", "block11 g0.55", lambda c: c["a11"], 0.550),
       ("B1", "b2+9 g0.620", lambda c: (c["a2"] + c["a9"]) / 2, 0.620),
       ("B2", "b2+9 g0.605", lambda c: (c["a2"] + c["a9"]) / 2, 0.605)]


def main():
    os.makedirs(VIZ, exist_ok=True)
    os.makedirs(os.path.join(OUT, "csv"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "metrics"), exist_ok=True)
    gt = p21.load_gt()
    cells = defaultdict(list)
    for r in csv.DictReader(open(POOL)):
        x1, y1, x2, y2 = [int(v) for v in r["uid"].rsplit("|", 1)[1].split("_")]
        cells[(r["dataset"], int(r["frame_id"]), r["object"])].append(dict(
            sem=float(r["sem_top5"]), a11=float(r["appe11"]), a2=float(r["appe2"]),
            a9=float(r["appe9"]), hsv=float(r["hsv"]), st=float(r["sim_thr"]),
            ht=float(r["hsv_thr"]), sel=r["is_selected"] == "1", box=(x1, y1, x2, y2)))

    def sel(cs):
        for c in cs:
            if c["sel"]: return c
        return max(cs, key=lambda c: c["sem"])

    man = []; agg = {t: [0, 0, 0] for t, _, _, _ in CFG}
    frames = sorted(gt.keys())
    for n, (ds, fr) in enumerate(frames, 1):
        bgr = cv2.imread(os.path.join(FRAMES, ds, f"frame_{fr:06d}.png"))
        if bgr is None: continue
        vis = gt[(ds, fr)]
        acc = {}
        for tag, _, fn, g in CFG:
            d = {}
            for o in p21.OBJECTS:
                cs = cells.get((ds, fr, o))
                if not cs: continue
                c = sel(cs)
                if c["sem"] >= c["st"] and fn(c) >= g and c["hsv"] >= c["ht"]:
                    d[o] = c["box"]
            acc[tag] = d
            for o in p21.OBJECTS:
                v = o in vis; a = o in d
                if v and a: agg[tag][0] += 1
                elif a: agg[tag][1] += 1
                elif v: agg[tag][2] += 1
        panels = [bgr]
        for tag, lab, _, _ in CFG:
            img = bgr.copy()
            # TP(초록) 먼저, FP(빨강) 나중에 -> 겹쳐도 FP가 가려지지 않음
            items = sorted(acc[tag].items(), key=lambda kv: (kv[0] not in vis, kv[0]))
            drawn = []
            for o, (x1, y1, x2, y2) in items:
                ok = o in vis
                col = (90, 220, 90) if ok else (60, 60, 235)     # 초록=TP 빨강=FP
                # 이미 같은(또는 거의 같은) 박스가 그려졌으면 안쪽으로 들여서 겹침을 보이게
                ins = 0
                for (bx, by, bX, bY) in drawn:
                    if abs(bx-x1) <= 4 and abs(by-y1) <= 4 and abs(bX-x2) <= 4 and abs(bY-y2) <= 4:
                        ins += 5
                X1, Y1, X2, Y2 = x1+ins, y1+ins, x2-ins, y2-ins
                drawn.append((x1, y1, x2, y2))
                cv2.rectangle(img, (X1, Y1), (X2, Y2), col, 3)
                t = o.replace("_high", "")
                ty = max(0, Y1 - 19 - (0 if ins == 0 else 0))
                cv2.rectangle(img, (X1, ty), (X1 + 8 * len(t), ty + 19), col, -1)
                cv2.putText(img, t, (X1 + 2, ty + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1)
            panels.append(img)
        cv2.imwrite(os.path.join(VIZ, f"{ds}_{fr:06d}.png"), np.hstack(panels))
        A, B1, B2 = acc["A"], acc["B1"], acc["B2"]
        man.append({"name": f"{ds}_{fr:06d}", "bag": ds, "frame": fr,
                    "gt_visible": ";".join(sorted(vis)),
                    "A": ";".join(sorted(A)), "B1": ";".join(sorted(B1)), "B2": ";".join(sorted(B2)),
                    "B2_gain_TP": ";".join(sorted((set(B2) - set(A)) & vis)),
                    "B2_new_FP": ";".join(sorted((set(B2) - set(A)) - vis)),
                    "B2_lost_TP": ";".join(sorted((set(A) - set(B2)) & vis)),
                    "B2_dup_box": ";".join(sorted(
                        o for o in set(B2) - set(A) - vis
                        if any(k != o and B2[k] == B2[o] for k in B2))),
                    "changed": int(set(A) != set(B2)), "image": f"{ds}_{fr:06d}.png"})
        if n % 60 == 0: print(f"  {n}/{len(frames)}")
    with open(os.path.join(OUT, "csv", "appe_ab_manifest.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(man[0].keys())); w.writeheader(); w.writerows(man)
    met = {}
    for tag, lab, _, g in CFG:
        t, fp, fn = agg[tag]; P = t / max(t + fp, 1); R = t / max(t + fn, 1)
        met[tag] = dict(label=lab, gate=g, TP=t, FP=fp, FN=fn, precision=round(P, 4),
                        recall=round(R, 4), f1=round(2 * P * R / max(P + R, 1e-9), 4))
    json.dump(met, open(os.path.join(OUT, "metrics", "appe_ab.json"), "w"), indent=2, ensure_ascii=False)
    for tag in met: print(f"  {tag} {met[tag]['label']:16s} TP {met[tag]['TP']} FP {met[tag]['FP']} F1 {met[tag]['f1']}")
    print(f"  변화 프레임 {sum(m['changed'] for m in man)}/{len(man)} -> {VIZ}")


if __name__ == "__main__":
    main()
