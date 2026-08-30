#!/usr/bin/env python3
"""eval_guard_sweep.py — 객체별 score_threshold 가드를 없애면 어떻게 되는가 (READ-ONLY).

redump_no_guard.py 로 conf=0.02 (전 객체 동일) 후보를 새로 확보했으므로
이제 YOLO 임계를 오프라인에서 임의로 다시 적용해 비교할 수 있다.

비교 축
  YOLO 임계 : 현행 객체별 가드(Bear .35 / Rabbit .30 / Dinosaur .30 / 나머지 .02)
              vs 전 객체 동일 0.02 / 0.10 / 0.20 / 0.30
  semantic  : sem_top5  vs  fusion α=0.30
  HSV 게이트: 없음  vs  색보정 렌더 HSV 게이트

threshold(semantic·HSV)는 held-out 을 제외한 5개 데이터셋 GT 에서만 F1 최대로 보정한다.
top_k = 3 (운영값) 을 오프라인에서 적용.

산출: results/gt_guard_sweep.csv, results/gt_guard_sweep_per_object.csv
"""
import csv, os
from collections import defaultdict

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
GT = os.path.join(RSRCH, "gt_input")
ND = os.path.join(HERE, "noguard")
RES = os.path.join(ROOT, "results")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
HS = slice(96, 224); MASKED = slice(0, 224)
HUE_SHIFT, SAT_GAIN, LOWSAT_TH = -6, 1.3, 20.0
TOP_K = 3

# ---------------------------------------------------------------- GT
gt = {}
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] == "yes":
        toks = [t.strip() for t in r["visible_objects"].split(";") if t.strip()]
        gt[(r["dataset_name"], int(r["frame_id"]))] = set(t for t in toks if t in OBJECTS)

# ---------------------------------------------------------------- 재덤프 적재
hsvq = {}
for ds in DATASETS:
    z = np.load(os.path.join(ND, f"{ds}_hsv.npz"))
    for u, h in zip(z["uid"], z["hist"]):
        hsvq[str(u)] = h[MASKED][HS].astype(np.float64)

GUARD = {}
pairs = defaultdict(list)          # (ds, frame, object) -> 후보들 (임계 적용 전)
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(ND, f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) != 1:
            continue
        GUARD.setdefault(r["object"], float(r["orig_score_threshold"]))
        pairs[(ds, int(r["frame_id"]), r["object"])].append({
            "uid": r["uid"], "conf": float(r["yolo_conf"]), "rank": int(r["yolo_rank"]),
            "top5": float(r["sem_top5"]), "mean": float(r["sem_mean"]),
            "appe": float(r["appe11_clstop1"]), "appe_gate": float(r["appe_gate"])})
print(f"재덤프 후보 그룹 {len(pairs):,} / 원래 가드 {GUARD}")

# ---------------------------------------------------------------- HSV prototype
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


PC = np.load(os.path.join(ROOT, "ply_hsv", "prototype_colors.npz"))
PROTO = {}
for o in OBJECTS:
    cols = PC[f"{o}__render42"]
    a = cols.reshape(-1, 1, 3)[:, :, ::-1]
    sat = float(cv2.cvtColor(a, cv2.COLOR_BGR2HSV)[..., 1].mean())
    PROTO[o] = np.stack([hist_hs(c, 0 if sat < LOWSAT_TH else HUE_SHIFT, SAT_GAIN) for c in cols])

HSVCACHE = {}
for k, v in pairs.items():
    for c in v:
        ck = (c["uid"], k[2])
        if ck not in HSVCACHE:
            q = hsvq.get(c["uid"])
            if q is None:
                HSVCACHE[ck] = 1.0
            else:
                bc = np.sqrt(np.maximum(q[None, :] * PROTO[k[2]], 0)).sum(1)
                HSVCACHE[ck] = float((1.0 - np.sqrt(np.maximum(0.0, 1.0 - bc))).max())
        c["hsv"] = HSVCACHE[ck]

# ---------------------------------------------------------------- 판정
def thr_of(obj, mode):
    return GUARD.get(obj, 0.02) if mode == "guard" else float(mode)


def decide(keys, ymode, sem_mode, sem_thr, hsv_thr, zs):
    m5, s5, mm, sm = zs
    res = {}
    for k in keys:
        t = thr_of(k[2], ymode)
        v = sorted([c for c in pairs[k] if c["conf"] >= t],
                   key=lambda c: -c["conf"])[:TOP_K]
        if not v:
            continue
        for c in v:
            c["_s"] = (0.3 * ((c["top5"] - m5) / s5) + 0.7 * ((c["mean"] - mm) / sm)
                       if sem_mode == "fusion" else c["top5"])
        b = max(v, key=lambda c: c["_s"])
        ok = (b["_s"] >= sem_thr) and (b["appe"] >= b["appe_gate"])
        if hsv_thr is not None:
            ok = ok and (b["hsv"] >= hsv_thr)
        res[k] = ok
    return res


def score(res, frames):
    TP = FP = FN = 0
    per = defaultdict(lambda: [0, 0, 0])
    for (ds, f) in frames:
        vis = gt[(ds, f)]
        for o in OBJECTS:
            acc = res.get((ds, f, o), False)
            v = o in vis
            if v and acc:
                TP += 1; per[o][0] += 1
            elif acc:
                FP += 1; per[o][1] += 1
            elif v:
                FN += 1; per[o][2] += 1
    return TP, FP, FN, per


YMODES = ["guard", "0.02", "0.10", "0.20", "0.30"]
YLABEL = {"guard": "현행 객체별 가드", "0.02": "전객체 0.02 (가드 제거)",
          "0.10": "전객체 0.10", "0.20": "전객체 0.20", "0.30": "전객체 0.30"}
CONFIGS = [(y, s, h) for y in YMODES for s in ("top5", "fusion") for h in (False, True)]

agg = defaultdict(lambda: defaultdict(int))
per_obj = defaultdict(lambda: defaultdict(lambda: [0, 0, 0]))
rows = []
for held in DATASETS:
    tr_f = [kf for kf in gt if kf[0] != held]
    te_f = [kf for kf in gt if kf[0] == held]
    tr_k = [k for k in pairs if k[0] != held]
    te_k = [k for k in pairs if k[0] == held]
    t5 = np.array([c["top5"] for k in tr_k for c in pairs[k]])
    mn = np.array([c["mean"] for k in tr_k for c in pairs[k]])
    zs = (t5.mean(), t5.std() + 1e-9, mn.mean(), mn.std() + 1e-9)

    for (ym, sm_, use_h) in CONFIGS:
        base = decide(tr_k, ym, sm_, -1e9, None, zs)
        allb = []
        for k in tr_k:
            t = thr_of(k[2], ym)
            v = sorted([c for c in pairs[k] if c["conf"] >= t], key=lambda c: -c["conf"])[:TOP_K]
            if v:
                allb.append(max(v, key=lambda c: c["_s"]))
        if not allb:
            continue
        sgrid = np.quantile([b["_s"] for b in allb], np.linspace(0.02, 0.98, 50))
        hgrid = (np.quantile([b["hsv"] for b in allb], np.linspace(0.0, 0.6, 13))
                 if use_h else [None])
        best, bs, bh = -1, sgrid[0], None
        for t in sgrid:
            for hh in hgrid:
                TP, FP, FN, _ = score(decide(tr_k, ym, sm_, t, hh, zs), tr_f)
                f1 = 2 * TP / max(2 * TP + FP + FN, 1)
                if f1 > best:
                    best, bs, bh = f1, t, hh
        TP, FP, FN, per = score(decide(te_k, ym, sm_, bs, bh, zs), te_f)
        name = f"{ym}|{sm_}|{'HSV' if use_h else 'noHSV'}"
        for kk, vv in (("TP", TP), ("FP", FP), ("FN", FN)):
            agg[name][kk] += vv
        for o in OBJECTS:
            for i in range(3):
                per_obj[name][o][i] += per[o][i]
        rows.append({"held_out": held, "yolo_thr": ym, "semantic": sm_,
                     "hsv_gate": int(use_h), "sem_thr": round(float(bs), 5),
                     "hsv_thr": ("" if bh is None else round(float(bh), 5)),
                     "TP": TP, "FP": FP, "FN": FN})

with open(os.path.join(RES, "gt_guard_sweep.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

print(f"\n=== LODO 합산 (사람 GT 935 가시)   top_k={TOP_K}")
print(f"{'YOLO 임계':22s} {'semantic':8s} {'HSV':6s} {'TP':>5} {'FP':>5} {'FN':>5} "
      f"{'Prec':>7} {'Rec':>7} {'F1':>7}")
for (ym, sm_, uh) in CONFIGS:
    n = f"{ym}|{sm_}|{'HSV' if uh else 'noHSV'}"
    x = agg[n]
    if not x:
        continue
    P = x["TP"] / max(x["TP"] + x["FP"], 1); R = x["TP"] / max(x["TP"] + x["FN"], 1)
    print(f"{YLABEL[ym]:22s} {sm_:8s} {'O' if uh else '-':6s} {x['TP']:>5} {x['FP']:>5} "
          f"{x['FN']:>5} {P:>7.4f} {R:>7.4f} {2*P*R/max(P+R,1e-9):>7.4f}")

print(f"\n=== 인형 3종 객체별 (TP/FP/FN)  — 가드가 걸려 있던 객체들")
sel = [f"{y}|fusion|HSV" for y in YMODES]
print(f"{'객체':22s} " + " ".join(f"{YLABEL[y][:14]:>16s}" for y in YMODES))
for o in ("Bear", "Rabbit", "Dinosaur"):
    print(f"{o:22s} " + " ".join(
        f"{per_obj[n][o][0]:>4}/{per_obj[n][o][1]:>3}/{per_obj[n][o][2]:>3} " for n in sel))
po = []
for o in OBJECTS:
    row = {"object": o}
    for (ym, sm_, uh) in CONFIGS:
        n = f"{ym}|{sm_}|{'HSV' if uh else 'noHSV'}"
        row[f"{n}_TP"], row[f"{n}_FP"], row[f"{n}_FN"] = per_obj[n][o]
    po.append(row)
with open(os.path.join(RES, "gt_guard_sweep_per_object.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(po[0].keys())); w.writeheader(); w.writerows(po)
print(f"\n-> {RES}/gt_guard_sweep.csv, gt_guard_sweep_per_object.csv")
