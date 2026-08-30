#!/usr/bin/env python3
"""eval_noncolor.py — 비색상 특징의 변별력과 잔여 오류 수정 가능성을 측정한다 (READ-ONLY).

기준선: semantic + appearance → HSV hard gate (TP 590 / FP 89 / FN 345)
HSV 게이트는 **고정**하고, 그 뒤에 추가 검증기를 붙였을 때의 효과만 본다.
새 특징을 모두 평균하지 않는다 — false acceptance 제거용 **검증기(verifier)** 로 평가한다.

측정 1: 특징별 순위 변별력 (provisional 박스라벨 기준, LODO)
        객체별 AUROC / PR-AUC + 주요 혼동 코호트(초코↔갈색, 인형 3종, Sauce↔Sikhye) 전용 AUROC

측정 2: 잔여 FP 89 를 실제로 줄이는가 (사람 GT 기준, LODO)
        HSV 통과분에 추가 게이트를 걸었을 때 ΔFP / ΔTP

산출: results/shape_analysis.csv, local_patch_analysis.csv,
      semantic_enhancement_analysis.csv, feature_candidate_comparison.csv, hsv_residual_fp.csv
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

SHAPE = ["aspect", "extent", "solidity", "compactness", "contour_complexity",
         "hu1", "hu2", "hu3", "hu4", "hu5", "hu6", "hu7",
         "edge_orient_entropy", "sil_iou_best", "sil_iou_mean"]
PATCH = ["appe_A_top1cls", "appe_B_semtop3", "appe_C_semtop5", "appe_D_max42",
         "appe_E_mnn", "appe_F_bidir", "appe_G_top20pct", "appe_H_pyramid",
         "appe_I_b2b11", "appe_b2_top1cls"]
LOGO = ["edge_density", "patch_top10pct", "patch_score_std", "disc_patch_score"]
SEM = ["sem_top1", "sem_top5", "sem_mean", "sem_bottom5", "sem_std",
       "sem_max_min_gap", "sem_top1_minus_mean"]
ALL = SHAPE + PATCH + LOGO + SEM

# ---------------------------------------------------------------- 적재
F = {}
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(FEAT, f"{ds}_feat.csv"))):
        d = {}
        for k in ALL:
            v = r.get(k, "")
            d[k] = float(v) if v not in ("", None) else np.nan
        d["is_candidate"] = int(r["is_candidate"])
        F[(r["uid"], r["object"])] = d
print(f"특징 pair {len(F):,}")

prov = {r["uid"]: r["true_class"]
        for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}
gt = {}
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] == "yes":
        gt[(r["dataset_name"], int(r["frame_id"]))] = set(
            t.strip() for t in r["visible_objects"].split(";") if t.strip())

O = np.load(os.path.join(COL, "results", "_outcomes.npy"), allow_pickle=True)[0]
key = lambda s: (s.split("|")[0], int(s.split("|")[1]), s.split("|")[2])


def auroc(pos, neg):
    pos = [x for x in pos if not np.isnan(x)]; neg = [x for x in neg if not np.isnan(x)]
    if len(pos) < 2 or len(neg) < 2:
        return None
    p, n = np.asarray(pos, float), np.asarray(neg, float)
    a = float(((p[:, None] > n[None, :]).sum() + 0.5 * (p[:, None] == n[None, :]).sum())
              / (len(p) * len(n)))
    return max(a, 1 - a)          # 방향 무관 변별력 (일부 특징은 작을수록 좋을 수 있음)


def pr_auc(sc, y):
    ok = [i for i, v in enumerate(sc) if not np.isnan(v)]
    if len(ok) < 4:
        return None
    sc = np.array(sc)[ok]; y = np.array(y)[ok]
    o = np.argsort(-sc); yy = y[o]
    tp = np.cumsum(yy); fp = np.cumsum(1 - yy)
    return float(np.sum(np.diff(np.concatenate([[0.0], tp / max(yy.sum(), 1)]))
                        * (tp / np.maximum(tp + fp, 1))))


# ---------------------------------------------------------------- 측정 1: 변별력
cand = defaultdict(list)
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) == 1 and r["uid"] in prov and prov[r["uid"]] != "unclear":
            cand[r["object"]].append((r["uid"], prov[r["uid"]]))

COHORT = {"초코↔갈색박스": ("choco_hazelnut_high", {"carton"}),
          "인형3종": (None, {"Bear", "Rabbit", "Dinosaur"}),
          "Sauce↔Sikhye": ("Sauce_high", {"Sikhye_high"})}

rows = []
for feat in ALL:
    per, cohort = {}, {}
    for o in OBJECTS:
        pos = [F[(u, o)][feat] for u, c in cand[o] if (u, o) in F and c == o]
        neg = [F[(u, o)][feat] for u, c in cand[o] if (u, o) in F and c != o]
        a = auroc(pos, neg)
        if a is not None:
            per[o] = a
    # 코호트
    for nm, (obj, negset) in COHORT.items():
        if obj:
            pos = [F[(u, obj)][feat] for u, c in cand[obj] if (u, obj) in F and c == obj]
            neg = [F[(u, obj)][feat] for u, c in cand[obj] if (u, obj) in F and c in negset]
            cohort[nm] = auroc(pos, neg)
        else:
            vs = []
            for o in negset:
                pos = [F[(u, o)][feat] for u, c in cand[o] if (u, o) in F and c == o]
                neg = [F[(u, o)][feat] for u, c in cand[o]
                       if (u, o) in F and c in negset and c != o]
                a = auroc(pos, neg)
                if a is not None:
                    vs.append(a)
            cohort[nm] = float(np.mean(vs)) if vs else None
    if not per:
        continue
    grp = ("shape" if feat in SHAPE else "patch" if feat in PATCH
           else "logo" if feat in LOGO else "semantic")
    rows.append({"feature": feat, "group": grp,
                 "mean_auroc": round(float(np.mean(list(per.values()))), 4),
                 "min_auroc": round(float(min(per.values())), 4),
                 "min_object": min(per, key=per.get),
                 **{f"auroc_{o}": round(per.get(o, np.nan), 4) for o in OBJECTS},
                 **{f"cohort_{k}": (round(v, 4) if v is not None else "")
                    for k, v in cohort.items()}})

rows.sort(key=lambda r: -r["mean_auroc"])
for grp, fn in (("shape", "shape_analysis.csv"), ("patch", "local_patch_analysis.csv"),
                ("semantic", "semantic_enhancement_analysis.csv")):
    R = [r for r in rows if r["group"] == grp] + \
        ([r for r in rows if r["group"] == "logo"] if grp == "patch" else [])
    with open(os.path.join(RES, fn), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(R)

print(f"\n=== 특징별 변별력 (provisional 박스라벨, 방향 무관 AUROC)")
print(f"{'특징':22s} {'군':9s} {'평균':>7} {'최저':>7} {'최저객체':>20} "
      f"{'초코↔갈색':>10} {'인형3종':>9} {'Sauce↔Sikhye':>13}")
for r in rows:
    print(f"{r['feature']:22s} {r['group']:9s} {r['mean_auroc']:>7.4f} {r['min_auroc']:>7.4f} "
          f"{r['min_object']:>20s} {str(r['cohort_초코↔갈색박스']):>10} "
          f"{str(r['cohort_인형3종']):>9} {str(r['cohort_Sauce↔Sikhye']):>13}")

# ---------------------------------------------------------------- 측정 2: 잔여 FP 감소
# HSV 게이트 통과분(=B 에서 수락된 것) 에 추가 게이트를 걸었을 때
accB = {s: v for s, v in O["B"].items() if v[0] in ("TP", "FP")}
print(f"\n=== HSV 통과 후 잔여: TP {sum(1 for v in accB.values() if v[0]=='TP')} / "
      f"FP {sum(1 for v in accB.values() if v[0]=='FP')}")

fp_rows = []
for s, v in accB.items():
    if v[0] != "FP":
        continue
    ds, fr, ob = key(s)
    u = v[1]
    vis = gt.get((ds, fr), set())
    fp_rows.append({"dataset": ds, "frame_id": fr, "accepted_object": ob, "uid": u,
                    "visible_in_frame": ";".join(sorted(vis)) or "none",
                    "provisional_label": prov.get(u, ""),
                    **{k: (F[(u, ob)][k] if (u, ob) in F else "") for k in
                       ("sil_iou_best", "appe_A_top1cls", "appe_E_mnn", "appe_G_top20pct",
                        "patch_top10pct", "disc_patch_score", "edge_density",
                        "sem_top5", "sem_bottom5")}})
with open(os.path.join(RES, "hsv_residual_fp.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(fp_rows[0].keys())); w.writeheader(); w.writerows(fp_rows)

CANDF = ["sil_iou_best", "solidity", "compactness", "extent",
         "appe_C_semtop5", "appe_D_max42", "appe_E_mnn", "appe_F_bidir",
         "appe_G_top20pct", "appe_H_pyramid", "appe_I_b2b11",
         "patch_top10pct", "disc_patch_score", "edge_density",
         "sem_bottom5", "sem_top1_minus_mean", "sem_max_min_gap"]
comp = []
for feat in CANDF:
    dTP = dFP = 0
    fold = []
    for held in DATASETS:
        tr = [(s, v) for s, v in accB.items() if key(s)[0] != held]
        te = [(s, v) for s, v in accB.items() if key(s)[0] == held]
        vals = [(F[(v[1], key(s)[2])][feat], v[0]) for s, v in tr
                if (v[1], key(s)[2]) in F and not np.isnan(F[(v[1], key(s)[2])][feat])]
        if len(vals) < 20:
            continue
        arr = np.array([x for x, _ in vals])
        best, bt = None, None
        for t in np.quantile(arr, np.linspace(0.0, 0.6, 25)):
            fp = sum(1 for x, o in vals if o == "FP" and x < t)
            tp = sum(1 for x, o in vals if o == "TP" and x < t)
            if tp <= 0.02 * sum(1 for _, o in vals if o == "TP"):   # TP 손실 2% 이내 제약
                if best is None or fp > best:
                    best, bt = fp, t
        if bt is None:
            continue
        for s, v in te:
            k2 = (v[1], key(s)[2])
            if k2 not in F or np.isnan(F[k2][feat]):
                continue
            if F[k2][feat] < bt:
                if v[0] == "FP":
                    dFP -= 1
                else:
                    dTP -= 1
        fold.append(round(float(bt), 4))
    comp.append({"feature": feat, "group": ("shape" if feat in SHAPE else
                                            "patch" if feat in PATCH else
                                            "logo" if feat in LOGO else "semantic"),
                 "delta_FP": dFP, "delta_TP": dTP,
                 "net": dFP - dTP if dTP <= 0 else dFP,
                 "fold_thresholds": ";".join(map(str, fold))})
comp.sort(key=lambda r: (r["delta_FP"], r["delta_TP"]))
with open(os.path.join(RES, "feature_candidate_comparison.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(comp[0].keys())); w.writeheader(); w.writerows(comp)

print(f"\n=== HSV 뒤 추가 검증기: 잔여 FP 89 중 얼마나 줄이는가 (TP 손실 ≤2% 제약, LODO)")
print(f"{'특징':22s} {'군':9s} {'ΔFP':>6} {'ΔTP':>6}  fold별 임계")
for r in comp:
    print(f"{r['feature']:22s} {r['group']:9s} {r['delta_FP']:>+6} {r['delta_TP']:>+6}  "
          f"{r['fold_thresholds'][:46]}")
print(f"\n-> {RES}")
