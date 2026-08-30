#!/usr/bin/env python3
"""eval_global_calib.py — 색 보정을 '전 객체 공통값' 으로 걸 수 있는지 검증한다 (READ-ONLY).

eval_hsv_methods.py 에서 객체별 최적 Hue 보정각이 -20°~+12° 로 흩어졌다.
직전 보고서는 Hue 단독(H32) 조건에서 공통 -5.6° 를 얻었으므로 두 결과가 충돌한다.
여기서는 **같은 조건(H16xS8)** 에서 공통값과 객체별값을 나란히 놓고 결론을 정리한다.

또한 prototype_colors 통계에서 렌더가 전 객체의 채도를 일관되게 낮추는 것이 확인됐으므로
(Bear 76.8 vs PLY 107.5 등) 채도 이득(gain)도 공통 보정 후보에 넣는다.

  render42                     현행
  render42_hue_global          공통 Hue 보정 (학습 fold 에서 1개 값 선택)
  render42_hue_perobj          객체별 Hue 보정 (참고 상한)
  render42_hs_global           공통 Hue + 공통 채도 gain
  render42_hs_global_lowsat    위와 같되 저채도 객체는 Hue 보정 제외
  ply_view42_hue_global        방식 B + 공통 Hue 보정

주의: 6개 데이터가 모두 같은 날 같은 사무실이므로 **조명이 다른 조건에서의 유효성은
      이 실험으로 확인할 수 없다.** 여기서 말하는 "일반화" 는 데이터셋 간 일반화뿐이다.

산출: results/hsv_global_calibration.csv
"""
import csv, os
from collections import defaultdict

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
RES = os.path.join(ROOT, "results")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
HS = slice(96, 224); MASKED = slice(0, 224)
SHIFTS = list(range(-20, 21, 2))
GAINS = [1.0, 1.15, 1.3, 1.5, 1.75, 2.0]
LOWSAT_TH = 20.0            # 평균 채도(0~255) 가 이보다 낮으면 Hue 가 의미를 잃는 객체로 본다


def hist_hs(rgb, shift=0, gain=1.0):
    img = np.asarray(rgb, np.uint8).reshape(-1, 1, 3)[:, :, ::-1]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.int16)
    if shift:
        hsv[..., 0] = (hsv[..., 0] + int(round(shift / 2))) % 180
    if gain != 1.0:
        hsv[..., 1] = np.clip(hsv[..., 1] * gain, 0, 255)
    hsv = hsv.astype(np.uint8)
    h = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256]).flatten()
    return (h / max(h.sum(), 1e-12)).astype(np.float64)


def sim_max(q, P):
    bc = np.sqrt(np.maximum(q[None, :] * P, 0)).sum(1)
    return float((1.0 - np.sqrt(np.maximum(0.0, 1.0 - bc))).max())


def auroc(pos, neg):
    if len(pos) == 0 or len(neg) == 0:
        return None
    p, n = np.asarray(pos, float), np.asarray(neg, float)
    return float(((p[:, None] > n[None, :]).sum() + 0.5 * (p[:, None] == n[None, :]).sum())
                 / (len(p) * len(n)))


Z = np.load(os.path.join(HERE, "prototype_colors.npz"))
OBJS = sorted({k.split("__")[0] for k in Z.files})
labels = {r["uid"]: r["true_class"]
          for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}
boxes = []
for ds in DATASETS:
    z = np.load(os.path.join(OBS, "hsv_features", f"{ds}_hsv.npz"))
    for u, h in zip(z["uid"], z["hist"]):
        c = labels.get(str(u))
        if c and c != "unclear":
            boxes.append((str(u), ds, c, h[MASKED][HS].astype(np.float64)))

# 객체 평균 채도 (렌더 기준)
SAT = {}
for obj in OBJS:
    k = f"{obj}__render42"
    if k in Z.files:
        a = Z[k].reshape(-1, 3)
        SAT[obj] = float(cv2.cvtColor(a.reshape(-1, 1, 3)[:, :, ::-1], cv2.COLOR_BGR2HSV)[..., 1].mean())
print("렌더 평균 채도:", {k: round(v, 1) for k, v in sorted(SAT.items())})
LOW = {o for o, s in SAT.items() if s < LOWSAT_TH}
print(f"저채도 객체(채도<{LOWSAT_TH}): {sorted(LOW) or '없음'}")

CACHE = {}
def P(obj, src, shift=0, gain=1.0):
    k = (obj, src, shift, gain)
    if k not in CACHE:
        key = f"{obj}__{src}"
        CACHE[k] = None if key not in Z.files else np.stack(
            [hist_hs(c, shift, gain) for c in Z[key]])
    return CACHE[k]


rows = []
gpick, gpick_hs = [], []
for held in DATASETS:
    tr = [b for b in boxes if b[1] != held]
    te = [b for b in boxes if b[1] == held]
    objs = [o for o in OBJS
            if sum(1 for b in te if b[2] == o) >= 2 and sum(1 for b in tr if b[2] == o) >= 2]
    if not objs:
        continue

    def mean_auroc(split, src, sh, gn, per_obj_shift=None, skip_low=False):
        acc = []
        for o in objs:
            s = per_obj_shift[o] if per_obj_shift else sh
            if skip_low and o in LOW:
                s = 0
            p = P(o, src, s, gn)
            if p is None:
                continue
            pos = [b for b in split if b[2] == o]; neg = [b for b in split if b[2] != o]
            if len(pos) < 2 or len(neg) < 2:
                continue
            acc.append(auroc([sim_max(b[3], p) for b in pos], [sim_max(b[3], p) for b in neg]))
        return float(np.mean(acc)) if acc else None

    # 공통 Hue 보정: 학습 fold 평균 AUROC 최대
    g = max(SHIFTS, key=lambda s: mean_auroc(tr, "render42", s, 1.0) or -1)
    gpick.append(g)
    # 공통 Hue + 채도 gain
    best, bg = -1, (0, 1.0)
    for s in SHIFTS:
        for gn in GAINS:
            v = mean_auroc(tr, "render42", s, gn) or -1
            if v > best:
                best, bg = v, (s, gn)
    gpick_hs.append(bg)
    # 객체별 (참고 상한)
    po = {}
    for o in objs:
        po[o] = max(SHIFTS, key=lambda s: (auroc(
            [sim_max(b[3], P(o, "render42", s)) for b in tr if b[2] == o],
            [sim_max(b[3], P(o, "render42", s)) for b in tr if b[2] != o]) or -1))
    gp = max(SHIFTS, key=lambda s: mean_auroc(tr, "ply_view42", s, 1.0) or -1)

    for name, kw in (
        ("render42", dict(src="render42", sh=0, gn=1.0)),
        ("render42_hue_global", dict(src="render42", sh=g, gn=1.0)),
        ("render42_hue_perobj", dict(src="render42", sh=0, gn=1.0, per_obj_shift=po)),
        ("render42_hs_global", dict(src="render42", sh=bg[0], gn=bg[1])),
        ("render42_hs_global_lowsat", dict(src="render42", sh=bg[0], gn=bg[1], skip_low=True)),
        ("ply_view42_hue_global", dict(src="ply_view42", sh=gp, gn=1.0)),
    ):
        for o in objs:
            s = kw.get("per_obj_shift", {}).get(o, kw["sh"]) if kw.get("per_obj_shift") else kw["sh"]
            if kw.get("skip_low") and o in LOW:
                s = 0
            p = P(o, kw["src"], s, kw["gn"])
            if p is None:
                continue
            pos = [b for b in te if b[2] == o]; neg = [b for b in te if b[2] != o]
            if len(pos) < 2 or len(neg) < 2:
                continue
            rows.append({"held_out": held, "method": name, "object": o,
                         "shift_deg": s, "sat_gain": kw["gn"],
                         "n_pos": len(pos), "n_neg": len(neg),
                         "auroc": round(auroc([sim_max(b[3], p) for b in pos],
                                              [sim_max(b[3], p) for b in neg]), 4)})

with open(os.path.join(RES, "hsv_global_calibration.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

M = ["render42", "render42_hue_global", "render42_hue_perobj", "render42_hs_global",
     "render42_hs_global_lowsat", "ply_view42_hue_global"]
per = defaultdict(lambda: defaultdict(list))
for r in rows:
    per[r["object"]][r["method"]].append(r["auroc"])
print(f"\n{'객체':22s} " + " ".join(f"{m[:20]:>21s}" for m in M))
for o in OBJS:
    if not per[o]:
        continue
    print(f"{o:22s} " + " ".join(
        f"{np.mean(per[o][m]):>21.4f}" if per[o][m] else f"{'-':>21s}" for m in M))
print(f"{'평균':22s} " + " ".join(
    f"{np.mean([np.mean(per[o][m]) for o in per if per[o][m]]):>21.4f}" for m in M))
print(f"{'최저객체':22s} " + " ".join(
    f"{min(np.mean(per[o][m]) for o in per if per[o][m]):>21.4f}" for m in M))
print(f"\n공통 Hue 보정각 (fold별): {gpick}  최빈 {max(set(gpick), key=gpick.count)}도")
print(f"공통 Hue+채도 (fold별):   {gpick_hs}")
print(f"-> {os.path.join(RES,'hsv_global_calibration.csv')}")
