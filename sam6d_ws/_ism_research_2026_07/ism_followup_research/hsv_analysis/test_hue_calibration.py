#!/usr/bin/env python3
"""test_hue_calibration.py — 렌더의 계통적 Hue 편향을 보정하면 얼마나 회복되는지 (READ-ONLY).

figures/hsv_render_vs_real.png 에서 렌더 hue peak 가 실사보다 일관되게 오른쪽으로
밀려 있는 것이 보인다(Bear 13 vs 7, saffron 23 vs 12, Sikhye 23 vs 17).
이것이 객체별 우연인지 렌더러의 계통 편향인지 확인하고, 보정 가능한지 검증한다.

  C1 render_shift_perobj   객체별 최적 hue 이동 (학습 fold 에서만 추정)
  C2 render_shift_global   전 객체 공통 hue 이동 (학습 fold 에서만 추정)
  C3 render_old            현행 (기준)

이동량을 평가 데이터로 정하면 과적합이므로 LODO 학습 fold 에서만 추정한다.
공통 이동(C2)이 객체별(C1)에 근접하면 "렌더러의 계통 편향" 이라는 증거가 된다.

산출: results/hue_calibration_lodo.csv
"""
import csv, os
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
RES = os.path.join(ROOT, "results")

DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
EXCLUDE = {"unclear"}
# 관찰 덤프 히스토그램 레이아웃: [0:32]H32 [32:64]S32 [64:96]V32 [96:224]H16xS8
H32 = slice(0, 32)
MASKED = slice(0, 224)
SHIFTS = list(range(-8, 9))          # H32 bin 단위 (1 bin = 5.625° in OpenCV hue)


def auroc(pos, neg):
    p, n = np.asarray(pos, float), np.asarray(neg, float)
    if len(p) == 0 or len(n) == 0:
        return None
    return float(((p[:, None] > n[None, :]).sum() + 0.5 * (p[:, None] == n[None, :]).sum())
                 / (len(p) * len(n)))


def sim(q, P):
    """1 - Bhattacharyya, prototype 집합에 대한 최대값."""
    bc = np.sqrt(np.maximum(q[None, :] * P, 0)).sum(1)
    return float((1.0 - np.sqrt(np.maximum(0.0, 1.0 - bc))).max())


def roll(P, k):
    """Hue 축은 원형이므로 순환 이동."""
    return np.roll(P, k, axis=1)


labels = {r["uid"]: r["true_class"]
          for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}
boxes = []
for ds in DATASETS:
    z = np.load(os.path.join(OBS, "hsv_features", f"{ds}_hsv.npz"))
    for u, h in zip(z["uid"], z["hist"]):
        c = labels.get(str(u))
        if c and c not in EXCLUDE:
            v = h[MASKED][H32].astype(np.float64)
            boxes.append((str(u), ds, c, v / max(v.sum(), 1e-12)))

zt = np.load(os.path.join(OBS, "hsv_features", "template_render_hsv.npz"))
REND = {}
for k in zt.files:
    P = zt[k][:, H32].astype(np.float64)
    REND[k] = P / np.maximum(P.sum(1, keepdims=True), 1e-12)

rows, per = [], defaultdict(lambda: defaultdict(list))
global_pick = []
for held in DATASETS:
    tr = [b for b in boxes if b[1] != held]
    te = [b for b in boxes if b[1] == held]

    # 공통 이동량: 학습 fold 전 객체 AUROC 평균이 최대인 shift
    gscore = {}
    for k in SHIFTS:
        acc = []
        for obj in REND:
            p = [b for b in tr if b[2] == obj]; n = [b for b in tr if b[2] != obj]
            if len(p) < 2:
                continue
            P = roll(REND[obj], k)
            acc.append(auroc([sim(b[3], P) for b in p], [sim(b[3], P) for b in n]))
        gscore[k] = float(np.mean(acc)) if acc else 0.0
    gk = max(gscore, key=gscore.get)
    global_pick.append(gk)

    for obj in sorted(REND):
        pos = [b for b in te if b[2] == obj]; neg = [b for b in te if b[2] != obj]
        trp = [b for b in tr if b[2] == obj]; trn = [b for b in tr if b[2] != obj]
        if len(pos) < 2 or len(neg) < 2 or len(trp) < 2:
            continue
        # 객체별 최적 이동량 (학습 fold 에서만)
        ok = max(SHIFTS, key=lambda k: auroc([sim(b[3], roll(REND[obj], k)) for b in trp],
                                             [sim(b[3], roll(REND[obj], k)) for b in trn]))
        r = {"held_out": held, "object": obj, "n_pos": len(pos), "n_neg": len(neg),
             "shift_perobj_bins": ok, "shift_perobj_deg": round(ok * 180 / 32, 1),
             "shift_global_bins": gk, "shift_global_deg": round(gk * 180 / 32, 1)}
        for name, P in (("render_old", REND[obj]),
                        ("render_shift_perobj", roll(REND[obj], ok)),
                        ("render_shift_global", roll(REND[obj], gk))):
            a = auroc([sim(b[3], P) for b in pos], [sim(b[3], P) for b in neg])
            r[f"auroc_{name}"] = round(a, 4)
            per[obj][name].append(a)
        rows.append(r)

with open(os.path.join(RES, "hue_calibration_lodo.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

print(f"{'객체':22s} {'현행':>9} {'객체별보정':>10} {'공통보정':>9} {'객체별이동(도)':>13}")
for o in sorted(per):
    sh = [r["shift_perobj_deg"] for r in rows if r["object"] == o]
    print(f"{o:22s} {np.mean(per[o]['render_old']):>9.4f} "
          f"{np.mean(per[o]['render_shift_perobj']):>10.4f} "
          f"{np.mean(per[o]['render_shift_global']):>9.4f} "
          f"{str(sorted(set(sh))):>13s}")
for k in ("render_old", "render_shift_perobj", "render_shift_global"):
    print(f"평균 {k:22s} = {np.mean([np.mean(per[o][k]) for o in per]):.4f}")
print(f"\nfold별 공통 이동량(bin) = {global_pick}  ({[round(g*180/32,1) for g in global_pick]}도)")
print(f"-> {os.path.join(RES,'hue_calibration_lodo.csv')}")
