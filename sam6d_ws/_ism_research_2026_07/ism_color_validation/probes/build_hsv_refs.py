#!/usr/bin/env python3
"""build_hsv_refs.py — HSV reference H0~H5 와 저채도 처리 변형을 만들어 비교한다 (READ-ONLY).

원칙
  · 실사 crop HSV 는 **평가 기준으로만** 쓰고 운영 reference 로 쓰지 않는다.
  · 후보 BBox / MobileSAM mask / semantic / appearance 는 전부 **고정**한다.
    (운영 덤프 ism_accuracy_observation 을 그대로 사용. 가드 제거 재덤프는 후보 집합이
     달라지므로 이번 비교에서 제외 — 색 효과만 분리하기 위함)

reference
  H0  HSV 미사용
  H1  기존 렌더 템플릿 42장
  H2  Hue 보정 렌더 템플릿 42장 (공통 -6°, 채도 x1.3)
  H3  PLY 전체 vertex color 히스토그램 1개
  H4  42 시점별 visible PLY vertex 히스토그램
  H5  H2 + H4 결합 (prototype 합집합에 max 유사도)

특징 공간 (저채도 대응 비교)
  hs      H16xS8            — 색상+채도 (기본)
  h       H32               — 색상만
  sv      S32+V32           — 채도+명도 (색상 미사용)
  hsv     H32+S32+V32       — 전부

저채도 처리 (Hue 가 불안정한 객체 대응, **객체별 전용 규칙이 아니라 공통식 안의 처리**)
  none      예외 없이 그대로
  skip      평균 채도 < TH 인 객체는 Hue 보정을 적용하지 않음
  satw      채도에 비례해 Hue 성분 가중 (저채도일수록 H 기여 감소)
  svfall    저채도 객체는 H 대신 S/V 특징으로 대체

산출: results/hsv_reference_comparison.csv
"""
import csv, json, os
from collections import defaultdict

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                      # ism_color_validation
RSRCH = os.path.dirname(ROOT)                     # _ism_research_2026_07
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
FUS = os.path.join(RSRCH, "ism_fusion_research")
GT = os.path.join(RSRCH, "gt_input")
RES = os.path.join(ROOT, "results")
os.makedirs(RES, exist_ok=True)

DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
HUE_SHIFT, SAT_GAIN, LOWSAT_TH = -6, 1.3, 20.0

# 관찰 덤프 히스토그램 레이아웃 (masked 블록 224차원)
SL = {"h": slice(0, 32), "s": slice(32, 64), "v": slice(64, 96), "hs": slice(96, 224)}
MASKED = slice(0, 224)


# ---------------------------------------------------------------- 특징 추출
def feat_from_hist(h224, space):
    """저장된 masked 히스토그램에서 특징 벡터를 뽑는다."""
    if space == "hs":
        x = h224[SL["hs"]]
    elif space == "h":
        x = h224[SL["h"]]
    elif space == "sv":
        x = np.concatenate([h224[SL["s"]], h224[SL["v"]]])
    else:                                   # hsv
        x = np.concatenate([h224[SL["h"]], h224[SL["s"]], h224[SL["v"]]])
    s = x.sum()
    return (x / s if s > 0 else x).astype(np.float64)


def feat_from_rgb(rgb, space, shift=0, gain=1.0):
    """prototype 원본 색에서 같은 규격의 특징을 만든다 (임의 Hue/채도 보정 적용 가능)."""
    img = np.asarray(rgb, np.uint8).reshape(-1, 1, 3)[:, :, ::-1]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.int16)
    if shift:
        hsv[..., 0] = (hsv[..., 0] + int(round(shift / 2))) % 180
    if gain != 1.0:
        hsv[..., 1] = np.clip(hsv[..., 1] * gain, 0, 255)
    hsv = hsv.astype(np.uint8)
    if space == "hs":
        x = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256]).flatten()
    elif space == "h":
        x = cv2.calcHist([hsv], [0], None, [32], [0, 180]).flatten()
    elif space == "sv":
        x = np.concatenate([cv2.calcHist([hsv], [1], None, [32], [0, 256]).flatten(),
                            cv2.calcHist([hsv], [2], None, [32], [0, 256]).flatten()])
    else:
        x = np.concatenate([cv2.calcHist([hsv], [0], None, [32], [0, 180]).flatten(),
                            cv2.calcHist([hsv], [1], None, [32], [0, 256]).flatten(),
                            cv2.calcHist([hsv], [2], None, [32], [0, 256]).flatten()])
    # sv/hsv 는 두 히스토그램을 이어붙이므로 각각 정규화 후 합이 1이 되게 맞춘다
    if space in ("sv", "hsv"):
        n = 2 if space == "sv" else 3
        parts = np.split(x, n)
        x = np.concatenate([p / max(p.sum(), 1e-12) for p in parts]) / n
    else:
        x = x / max(x.sum(), 1e-12)
    return x.astype(np.float64)


def sim_max(q, P):
    """1 - Bhattacharyya, prototype 집합 중 최대."""
    bc = np.sqrt(np.maximum(q[None, :] * P, 0)).sum(1)
    return float((1.0 - np.sqrt(np.maximum(0.0, 1.0 - bc))).max())


def auroc(pos, neg):
    if len(pos) == 0 or len(neg) == 0:
        return None
    p, n = np.asarray(pos, float), np.asarray(neg, float)
    return float(((p[:, None] > n[None, :]).sum() + 0.5 * (p[:, None] == n[None, :]).sum())
                 / (len(p) * len(n)))


def pr_auc(sc, y):
    o = np.argsort(-np.asarray(sc, float)); yy = np.asarray(y)[o]
    tp = np.cumsum(yy); fp = np.cumsum(1 - yy)
    prec = tp / np.maximum(tp + fp, 1); rec = tp / max(yy.sum(), 1)
    return float(np.sum(np.diff(np.concatenate([[0.0], rec])) * prec))


# ---------------------------------------------------------------- 적재
PC = np.load(os.path.join(FUS, "ply_hsv", "prototype_colors.npz"))
SAT = {}
for o in OBJECTS:
    a = PC[f"{o}__render42"].reshape(-1, 1, 3)[:, :, ::-1]
    SAT[o] = float(cv2.cvtColor(a, cv2.COLOR_BGR2HSV)[..., 1].mean())
print("객체 평균 채도(렌더):", {k: round(v, 1) for k, v in sorted(SAT.items())})
LOW = {o for o in OBJECTS if SAT[o] < LOWSAT_TH}
print(f"저채도 객체 (< {LOWSAT_TH}): {sorted(LOW) or '없음'}")

# 평가 라벨: provisional 박스 라벨 (사람 GT 와 구분해 표기)
prov = {r["uid"]: r["true_class"]
        for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}
boxes = []
for ds in DATASETS:
    z = np.load(os.path.join(OBS, "hsv_features", f"{ds}_hsv.npz"))
    for u, h in zip(z["uid"], z["hist"]):
        c = prov.get(str(u))
        if c and c != "unclear":
            boxes.append((str(u), ds, c, h[MASKED].astype(np.float64)))
print(f"평가 박스 {len(boxes)} (provisional 라벨 — 사람 GT 아님)")


# ---------------------------------------------------------------- prototype 빌더
def build_proto(ref, space, lowsat_mode):
    """(object -> prototype 행렬) 반환. lowsat_mode 는 공통식 안의 저채도 처리."""
    P = {}
    for o in OBJECTS:
        sp = space
        shift, gain = 0, 1.0
        if ref in ("H2", "H5"):
            shift, gain = HUE_SHIFT, SAT_GAIN
            if lowsat_mode == "skip" and o in LOW:
                shift = 0
            if lowsat_mode == "svfall" and o in LOW:
                sp, shift = "sv", 0
        if ref == "H1":
            cols = [PC[f"{o}__render42"]]
        elif ref == "H2":
            cols = [PC[f"{o}__render42"]]
        elif ref == "H3":
            cols = [PC[f"{o}__ply_all1"]]
        elif ref == "H4":
            cols = [PC[f"{o}__ply_view42"]]
        elif ref == "H5":
            cols = [PC[f"{o}__render42"], PC[f"{o}__ply_view42"]]
        else:
            P[o] = None; continue
        M = np.vstack([np.stack([feat_from_rgb(c, sp, shift, gain) for c in blk])
                       for blk in cols])
        P[o] = (M, sp)
    return P


def score_box(h224, obj, P, lowsat_mode):
    ent = P.get(obj)
    if ent is None:
        return 0.0
    M, sp = ent
    q = feat_from_hist(h224, sp)
    s = sim_max(q, M)
    if lowsat_mode == "satw":
        # 채도가 낮을수록 색 점수를 중립(0.5)쪽으로 끌어당긴다 — 공통식, 객체별 규칙 아님
        w = min(1.0, SAT[obj] / 60.0)
        s = w * s + (1 - w) * 0.5
    return s


REFS = ["H1", "H2", "H3", "H4", "H5"]
SPACES = ["hs", "h", "sv", "hsv"]
LOWMODES = ["none", "skip", "satw", "svfall"]

rows = []
for ref in REFS:
    for space in SPACES:
        for lm in LOWMODES:
            # 저채도 처리는 Hue 보정이 있는 ref 에서만 의미가 다르다.
            if ref in ("H1", "H3", "H4") and lm in ("skip", "svfall"):
                continue
            P = build_proto(ref, space, lm)
            per, perp = {}, {}
            for held in DATASETS:
                te = [b for b in boxes if b[1] == held]
                for o in OBJECTS:
                    pos = [b for b in te if b[2] == o]; neg = [b for b in te if b[2] != o]
                    if len(pos) < 2 or len(neg) < 2:
                        continue
                    sp_ = [score_box(b[3], o, P, lm) for b in pos]
                    sn_ = [score_box(b[3], o, P, lm) for b in neg]
                    per.setdefault(o, []).append(auroc(sp_, sn_))
                    perp.setdefault(o, []).append(
                        pr_auc(sp_ + sn_, [1] * len(sp_) + [0] * len(sn_)))
            if not per:
                continue
            m = {o: float(np.mean(v)) for o, v in per.items()}
            mp = {o: float(np.mean(v)) for o, v in perp.items()}
            rows.append({"reference": ref, "space": space, "lowsat": lm,
                         "mean_auroc": round(float(np.mean(list(m.values()))), 4),
                         "min_object_auroc": round(float(min(m.values())), 4),
                         "min_object": min(m, key=m.get),
                         "mean_prauc": round(float(np.mean(list(mp.values()))), 4),
                         **{f"auroc_{o}": round(m.get(o, float('nan')), 4) for o in OBJECTS}})
            print(f"{ref} {space:4s} {lm:7s} 평균AUROC {rows[-1]['mean_auroc']:.4f} "
                  f"최저 {rows[-1]['min_object_auroc']:.4f}({rows[-1]['min_object']}) "
                  f"PR {rows[-1]['mean_prauc']:.4f}")

rows.sort(key=lambda r: -r["mean_auroc"])
with open(os.path.join(RES, "hsv_reference_comparison.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

print(f"\n=== 상위 10 (평균 AUROC)")
print(f"{'ref':4s} {'space':6s} {'lowsat':8s} {'평균':>8} {'최저객체':>9} {'최저':>8} {'PR':>8}")
for r in rows[:10]:
    print(f"{r['reference']:4s} {r['space']:6s} {r['lowsat']:8s} {r['mean_auroc']:>8.4f} "
          f"{r['min_object']:>9s} {r['min_object_auroc']:>8.4f} {r['mean_prauc']:>8.4f}")
json.dump({"lowsat_objects": sorted(LOW), "saturation": SAT,
           "best": rows[0]}, open(os.path.join(RES, "hsv_reference_best.json"), "w"),
          indent=2, ensure_ascii=False)
print(f"\n-> {os.path.join(RES,'hsv_reference_comparison.csv')}")
