#!/usr/bin/env python3
"""eval_hsv_methods.py — HSV prototype 생성 방식 6종을 같은 척도로 비교한다 (READ-ONLY).

비교 대상
  render42          기존 렌더 템플릿 42장 (현행)
  ply_all1          방식 A — PLY 전 vertex, 객체당 히스토그램 1개
  ply_view42        방식 B — 42 시점별 visible vertex, 객체당 42개
  render42_hue      렌더 + Hue 보정 (보정각은 calibration fold 에서만 추정)
  ply_view42_hue    방식 B + Hue 보정
  render42_ply_max  렌더와 방식 B 의 prototype 합집합에 max 유사도
  render42_ply_avg  렌더 점수와 방식 B 점수의 평균

평가
  질의(query) = 6개 SAM 데이터의 실사 crop masked H16xS8 (이전 관찰 덤프에 저장된 값).
  **실사 crop 은 평가 기준으로만 쓰고 prototype 으로는 쓰지 않는다**(지시).
  LODO 6-fold. Hue 보정각은 학습 fold 에서만 고른다.

측정
  객체별 AUROC / PR-AUC, 실사와의 거리, 시점 안정성, 채도와의 관계
산출
  results/hsv_method_lodo.csv, results/hsv_method_summary.csv, results/hsv_view_stability.csv
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
HS = slice(96, 224)          # 관찰 덤프 masked 구간 내 H16xS8
MASKED = slice(0, 224)
EXCLUDE = {"unclear"}
SHIFTS = list(range(-20, 21, 2))          # 도 단위, 1 bin=11.25° 가 아니라 원본 색에서 정확히 적용


def hist_hs(rgb, hue_shift=0):
    """RGB[N,3] -> H16xS8 정규화 히스토그램. hue_shift 는 **도(degree)** 단위, 원본 색에 적용."""
    img = np.asarray(rgb, np.uint8).reshape(-1, 1, 3)[:, :, ::-1]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if hue_shift:
        # OpenCV Hue 는 0~179 (실제 각도의 1/2). 도 단위 shift → OpenCV 단위 shift/2
        hsv[..., 0] = (hsv[..., 0].astype(np.int16) + int(round(hue_shift / 2))) % 180
    h = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256]).flatten()
    return (h / max(h.sum(), 1e-12)).astype(np.float64)


def protos(colors, hue_shift=0):
    return np.stack([hist_hs(c, hue_shift) for c in colors])


def sim_max(q, P):
    bc = np.sqrt(np.maximum(q[None, :] * P, 0)).sum(1)
    return float((1.0 - np.sqrt(np.maximum(0.0, 1.0 - bc))).max())


def bhatt(a, b):
    a = a / max(a.sum(), 1e-12); b = b / max(b.sum(), 1e-12)
    return float(np.sqrt(max(0.0, 1.0 - np.sqrt(a * b).sum())))


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
Z = np.load(os.path.join(HERE, "prototype_colors.npz"))
OBJS = sorted({k.split("__")[0] for k in Z.files})
labels = {r["uid"]: r["true_class"]
          for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}
boxes = []
for ds in DATASETS:
    z = np.load(os.path.join(OBS, "hsv_features", f"{ds}_hsv.npz"))
    for u, h in zip(z["uid"], z["hist"]):
        c = labels.get(str(u))
        if c and c not in EXCLUDE:
            boxes.append((str(u), ds, c, h[MASKED][HS].astype(np.float64)))
print(f"평가 박스 {len(boxes)} | 객체 {len(OBJS)}")

# prototype 캐시 (shift 별)
CACHE = {}
def get(obj, src, shift=0):
    k = (obj, src, shift)
    if k not in CACHE:
        key = f"{obj}__{src}"
        if key not in Z.files:
            CACHE[k] = None
        else:
            CACHE[k] = protos(Z[key], shift)
    return CACHE[k]


# ---------------------------------------------------------------- 시점 안정성 & 실사 거리
real_cent = {}
for obj in OBJS:
    v = [b[3] for b in boxes if b[2] == obj]
    if v:
        m = np.mean(v, 0); real_cent[obj] = m / max(m.sum(), 1e-12)

stab = []
for obj in OBJS:
    row = {"object": obj, "n_real_crop": sum(1 for b in boxes if b[2] == obj)}
    for src in ("render42", "ply_view42", "ply_all1"):
        P = get(obj, src)
        if P is None:
            continue
        if len(P) > 1:
            d = [bhatt(P[i], P[j]) for i in range(len(P)) for j in range(i + 1, len(P))]
            row[f"viewvar_{src}"] = round(float(np.mean(d)), 4)      # 시점 간 평균 거리 = 불안정도
        else:
            row[f"viewvar_{src}"] = ""
        c = np.mean(P, 0); c = c / max(c.sum(), 1e-12)
        row[f"dist_real_{src}"] = (round(bhatt(c, real_cent[obj]), 4) if obj in real_cent else "")
    stab.append(row)
with open(os.path.join(RES, "hsv_view_stability.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(stab[0].keys()), extrasaction="ignore")
    w.writeheader(); w.writerows(stab)

# ---------------------------------------------------------------- LODO 평가
METHODS = ["render42", "ply_all1", "ply_view42", "render42_hue", "ply_view42_hue",
           "render42_ply_max", "render42_ply_avg"]
rows = []
picked_shift = defaultdict(list)
for held in DATASETS:
    tr = [b for b in boxes if b[1] != held]
    te = [b for b in boxes if b[1] == held]
    for obj in OBJS:
        pos = [b for b in te if b[2] == obj]; neg = [b for b in te if b[2] != obj]
        trp = [b for b in tr if b[2] == obj]; trn = [b for b in tr if b[2] != obj]
        if len(pos) < 2 or len(neg) < 2 or len(trp) < 2:
            continue

        # Hue 보정각: 학습 fold 에서 AUROC 최대인 값 (전 객체 공통값을 쓰지 않고
        # 객체별로 고르되, 아래에서 공통값 안정성도 함께 본다)
        def best_shift(src):
            best, bs = -1, 0
            for s in SHIFTS:
                P = get(obj, src, s)
                if P is None:
                    return 0
                a = auroc([sim_max(b[3], P) for b in trp], [sim_max(b[3], P) for b in trn])
                if a is not None and a > best:
                    best, bs = a, s
            return bs

        for m in METHODS:
            if m == "render42_hue":
                s = best_shift("render42"); P = get(obj, "render42", s)
            elif m == "ply_view42_hue":
                s = best_shift("ply_view42"); P = get(obj, "ply_view42", s)
            elif m == "render42_ply_max":
                a, b_ = get(obj, "render42"), get(obj, "ply_view42")
                P = np.vstack([a, b_]) if a is not None and b_ is not None else None; s = 0
            elif m == "render42_ply_avg":
                a, b_ = get(obj, "render42"), get(obj, "ply_view42"); s = 0
                if a is None or b_ is None:
                    continue
                sp = [0.5 * sim_max(x[3], a) + 0.5 * sim_max(x[3], b_) for x in pos]
                sn = [0.5 * sim_max(x[3], a) + 0.5 * sim_max(x[3], b_) for x in neg]
                rows.append({"held_out": held, "object": obj, "method": m, "hue_shift_deg": 0,
                             "n_pos": len(pos), "n_neg": len(neg),
                             "auroc": round(auroc(sp, sn), 4),
                             "pr_auc": round(pr_auc(sp + sn, [1]*len(sp)+[0]*len(sn)), 4)})
                continue
            else:
                P = get(obj, m); s = 0
            if P is None:
                continue
            if m.endswith("_hue"):
                picked_shift[(obj, m)].append(s)
            sp = [sim_max(b[3], P) for b in pos]; sn = [sim_max(b[3], P) for b in neg]
            rows.append({"held_out": held, "object": obj, "method": m, "hue_shift_deg": s,
                         "n_pos": len(pos), "n_neg": len(neg),
                         "auroc": round(auroc(sp, sn), 4),
                         "pr_auc": round(pr_auc(sp + sn, [1]*len(sp)+[0]*len(sn)), 4)})

with open(os.path.join(RES, "hsv_method_lodo.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

per = defaultdict(lambda: defaultdict(list))
perp = defaultdict(lambda: defaultdict(list))
for r in rows:
    per[r["object"]][r["method"]].append(r["auroc"])
    perp[r["object"]][r["method"]].append(r["pr_auc"])

summ = []
for obj in OBJS:
    row = {"object": obj}
    for m in METHODS:
        row[f"auroc_{m}"] = round(float(np.mean(per[obj][m])), 4) if per[obj][m] else ""
        row[f"prauc_{m}"] = round(float(np.mean(perp[obj][m])), 4) if perp[obj][m] else ""
    summ.append(row)
with open(os.path.join(RES, "hsv_method_summary.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(summ[0].keys())); w.writeheader(); w.writerows(summ)

print(f"\n=== LODO held-out AUROC\n{'객체':22s} " + " ".join(f"{m[:16]:>17s}" for m in METHODS))
for r in summ:
    print(f"{r['object']:22s} " + " ".join(
        f"{r['auroc_'+m]:>17}" if r["auroc_" + m] != "" else f"{'-':>17s}" for m in METHODS))
print(f"{'평균':22s} " + " ".join(
    f"{np.mean([r['auroc_'+m] for r in summ if r['auroc_'+m]!='']):>17.4f}" for m in METHODS))
print(f"\n=== LODO held-out PR-AUC 평균")
for m in METHODS:
    v = [r["prauc_" + m] for r in summ if r["prauc_" + m] != ""]
    print(f"  {m:20s} {np.mean(v):.4f}")

print("\n=== Hue 보정각 선택 (fold별)")
for (obj, m), v in sorted(picked_shift.items()):
    if m == "render42_hue":
        print(f"  {obj:22s} {m:16s} {v}  최빈 {max(set(v), key=v.count)}도")
print("\n=== 시점 안정성(값이 클수록 시점 간 색 변동이 큼) / 실사와의 거리")
print(f"{'객체':22s} {'변동_렌더':>10} {'변동_PLY42':>11} {'거리_렌더':>10} "
      f"{'거리_PLY42':>11} {'거리_PLY전체':>12}")
for r in stab:
    print(f"{r['object']:22s} {str(r.get('viewvar_render42','')):>10} "
          f"{str(r.get('viewvar_ply_view42','')):>11} {str(r.get('dist_real_render42','')):>10} "
          f"{str(r.get('dist_real_ply_view42','')):>11} {str(r.get('dist_real_ply_all1','')):>12}")
