#!/usr/bin/env python3
"""eval_pipeline.py — HSV 게이트 뒤에 붙일 검증기 배치를 비교한다 (READ-ONLY).

원칙(지시 §8): 새 특징을 단순 평균하지 않는다. **false acceptance 를 제거하는 검증기**로만 쓴다.
HSV hard gate 는 고정. 그 뒤 단계만 바꾼다.

  P0  baseline                sem → appe → HSV                       (TP 590 / FP 89)
  P1  + shape gate            … → HSV → sil_iou/solidity 게이트
  P2  + local patch 전수      … → HSV → appe_I(block2+block11) 게이트
  P3  + local patch 조건부    … → HSV → **class ambiguity 높을 때만** appe_I 게이트
  P4  + logo(edge) 전수       … → HSV → edge_density 게이트
  P5  P3 + 동색 hard negative 객체만  (choco/Febreze 처럼 잔여 FP 가 몰린 객체에서만)

class ambiguity = 같은 proposal group 안에서 2위 클래스와의 appe 격차가 작음
  → 지시된 "명확한 경우 기존 경로 유지, 애매할 때만 국소 patch 검사"

임계는 전부 LODO calibration fold 에서만 TP 손실 제약 하 FP 최소화로 정한다.

산출: results/pipeline_comparison.csv, recommended_method_cases.csv
"""
import csv, json, os
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
COL = os.path.join(RSRCH, "ism_color_validation")
GT = os.path.join(RSRCH, "gt_input")
FEAT = os.path.join(ROOT, "features")
RES = os.path.join(ROOT, "results")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
SAMECOLOR = {"choco_hazelnut_high", "Febreze_high"}      # 잔여 FP 가 몰린 객체 (66%)
TP_LOSS = float(os.environ.get("TP_LOSS", "0.02"))

F = {}
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(FEAT, f"{ds}_feat.csv"))):
        F[(r["uid"], r["object"])] = r
O = np.load(os.path.join(COL, "results", "_outcomes.npy"), allow_pickle=True)[0]
key = lambda s: (s.split("|")[0], int(s.split("|")[1]), s.split("|")[2])
ACC = {s: v for s, v in O["B"].items() if v[0] in ("TP", "FP")}     # HSV 통과분
print(f"HSV 통과 {len(ACC)} (TP {sum(1 for v in ACC.values() if v[0]=='TP')} / "
      f"FP {sum(1 for v in ACC.values() if v[0]=='FP')})")

# class ambiguity: 같은 프레임에서 2위 클래스와의 appe 격차
appe = defaultdict(dict)
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) == 1:
            k = (ds, int(r["frame_id"]))
            v = float(r["appe11_clstop1"])
            appe[k][r["object"]] = max(appe[k].get(r["object"], 0), v)


def ambiguity(s):
    ds, fr, ob = key(s)
    d = appe.get((ds, fr), {})
    if ob not in d or len(d) < 2:
        return 0.0
    o = sorted(d.values(), reverse=True)
    return 1.0 - (o[0] - o[1])            # 격차가 작을수록 ambiguity 큼


def fv(s, feat):
    ds, fr, ob = key(s)
    r = F.get((ACC[s][1], ob))
    if not r:
        return np.nan
    v = r.get(feat, "")
    return float(v) if v not in ("", None) else np.nan


def run(mode, feat, amb_th=None, objs=None):
    """LODO: calibration 에서 임계 결정 → held-out 채점. (dTP, dFP, fold별 임계)"""
    dTP = dFP = 0
    folds = []
    for held in DATASETS:
        tr = [s for s in ACC if key(s)[0] != held]
        te = [s for s in ACC if key(s)[0] == held]

        def applies(s):
            if objs is not None and key(s)[2] not in objs:
                return False
            if amb_th is not None and ambiguity(s) < amb_th:
                return False
            return True

        vals = [(fv(s, feat), ACC[s][0]) for s in tr if applies(s) and not np.isnan(fv(s, feat))]
        if len(vals) < 15:
            folds.append(None); continue
        ntp = sum(1 for _, o in vals if o == "TP")
        arr = np.array([x for x, _ in vals])
        best, bt = -1, None
        for t in np.quantile(arr, np.linspace(0.0, 0.7, 36)):
            fp = sum(1 for x, o in vals if o == "FP" and x < t)
            tp = sum(1 for x, o in vals if o == "TP" and x < t)
            if tp <= TP_LOSS * max(ntp, 1) and fp > best:
                best, bt = fp, t
        folds.append(None if bt is None else round(float(bt), 4))
        if bt is None:
            continue
        for s in te:
            if not applies(s):
                continue
            x = fv(s, feat)
            if np.isnan(x) or x >= bt:
                continue
            if ACC[s][0] == "FP":
                dFP -= 1
            else:
                dTP -= 1
    return dTP, dFP, folds


PIPES = [
    ("P0", "baseline (sem→appe→HSV)", None, None, None),
    ("P1", "+ shape gate (sil_iou_best)", "sil_iou_best", None, None),
    ("P1b", "+ shape gate (solidity)", "solidity", None, None),
    ("P2", "+ local patch 전수 (appe_I b2+b11)", "appe_I_b2b11", None, None),
    ("P2b", "+ local patch 전수 (appe_D max42)", "appe_D_max42", None, None),
    ("P3", "+ local patch 조건부 (ambiguity 상위)", "appe_I_b2b11", 0.90, None),
    ("P3b", "+ local patch 조건부 (ambiguity 중간)", "appe_I_b2b11", 0.85, None),
    ("P4", "+ logo/edge 전수 (edge_density)", "edge_density", None, None),
    ("P5", "+ local patch, 동색 hard-neg 객체만", "appe_I_b2b11", None, SAMECOLOR),
]
base_tp = sum(1 for v in ACC.values() if v[0] == "TP")
base_fp = sum(1 for v in ACC.values() if v[0] == "FP")
rows = []
for pid, lb, feat, amb, objs in PIPES:
    if feat is None:
        dTP, dFP, folds = 0, 0, []
    else:
        dTP, dFP, folds = run(pid, feat, amb, objs)
    TP, FP = base_tp + dTP, base_fp + dFP
    FN = 345 - dTP                              # TP 감소분이 FN 으로
    P = TP / max(TP + FP, 1); R = TP / max(TP + FN, 1)
    rows.append({"pipeline": pid, "label": lb, "feature": feat or "",
                 "ambiguity_th": amb or "", "objects": ";".join(sorted(objs)) if objs else "전체",
                 "TP": TP, "FP": FP, "FN": FN, "dTP": dTP, "dFP": dFP,
                 "precision": round(P, 4), "recall": round(R, 4),
                 "f1": round(2 * P * R / max(P + R, 1e-9), 4),
                 "fp_per_tp_cost": (round(abs(dFP) / abs(dTP), 2) if dTP else ""),
                 "fold_thresholds": ";".join(str(x) for x in folds)})

with open(os.path.join(RES, "pipeline_comparison.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

print(f"\n=== 검증기 배치 비교 (HSV 고정, TP 손실 제약 {TP_LOSS:.0%}, LODO)")
print(f"{'':4s} {'배치':38s} {'TP':>5} {'FP':>4} {'FN':>5} {'ΔTP':>5} {'ΔFP':>5} "
      f"{'Prec':>7} {'F1':>7} {'FP/TP':>6}")
for r in rows:
    print(f"{r['pipeline']:4s} {r['label']:38s} {r['TP']:>5} {r['FP']:>4} {r['FN']:>5} "
          f"{r['dTP']:>+5} {r['dFP']:>+5} {r['precision']:>7.4f} {r['f1']:>7.4f} "
          f"{str(r['fp_per_tp_cost']):>6}")

# 권장안 상세 사례
best = max((r for r in rows if r["pipeline"] != "P0"),
           key=lambda r: (r["dFP"] * -1) / max(abs(r["dTP"]), 1))
print(f"\n권장(FP/TP 효율 최대): {best['pipeline']} {best['label']}")

BEST = "appe_I_b2b11"
cases = []
for held in DATASETS:
    tr = [s for s in ACC if key(s)[0] != held]
    vals = [(fv(s, BEST), ACC[s][0]) for s in tr if not np.isnan(fv(s, BEST))]
    ntp = sum(1 for _, o in vals if o == "TP")
    arr = np.array([x for x, _ in vals]); bt, bf = None, -1
    for t in np.quantile(arr, np.linspace(0.0, 0.7, 36)):
        fp = sum(1 for x, o in vals if o == "FP" and x < t)
        tp = sum(1 for x, o in vals if o == "TP" and x < t)
        if tp <= TP_LOSS * max(ntp, 1) and fp > bf:
            bf, bt = fp, t
    if bt is None:
        continue
    for s in [x for x in ACC if key(x)[0] == held]:
        x = fv(s, BEST)
        if np.isnan(x) or x >= bt:
            continue
        ds, fr, ob = key(s)
        cases.append({"dataset": ds, "frame_id": fr, "object": ob, "uid": ACC[s][1],
                      "baseline_outcome": ACC[s][0],
                      "new_outcome": "TN" if ACC[s][0] == "FP" else "FN",
                      "direction": "fixed" if ACC[s][0] == "FP" else "broken",
                      "feature": BEST, "value": round(float(x), 5),
                      "threshold": round(float(bt), 4),
                      "ambiguity": round(ambiguity(s), 4)})
with open(os.path.join(RES, "recommended_method_cases.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(cases[0].keys())); w.writeheader(); w.writerows(cases)
from collections import Counter
print(f"\n권장안({BEST}) 판정 변화 {len(cases)}건: "
      f"{Counter(c['direction'] for c in cases).most_common()}")
print("  객체별 수정:", Counter(c["object"] for c in cases if c["direction"] == "fixed").most_common())
print("  객체별 파손:", Counter(c["object"] for c in cases if c["direction"] == "broken").most_common())
print(f"-> {RES}")
