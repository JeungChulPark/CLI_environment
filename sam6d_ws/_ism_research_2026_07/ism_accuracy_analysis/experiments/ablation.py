#!/usr/bin/env python3
"""ablation.py — B0~B10 ablation, bag 분리 calibration (READ-ONLY).

calib bags 에서만 threshold/weight 를 정하고 eval bags 에서만 성능을 보고한다.
모든 변형에 대해 (1) 운영 threshold 고정 결과와 (2) 재보정 결과를 함께 낸다.
"""
import csv, json, os, statistics
from collections import defaultdict, Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RES = os.path.join(ROOT, "results")

CALIB = {"sam_105314", "SAM_loop2", "SAM_circle", "SAM_occlusion"}
EVAL = {"sam_110633", "SAM_loop1"}
DOLLS = {"Bear", "Rabbit", "Dinosaur"}

pairs = list(csv.DictReader(open(os.path.join(ROOT, "features", "pairs.csv"))))
labels = {r["uid"]: r["true_class"] for r in csv.DictReader(open(os.path.join(ROOT, "labels", "box_labels.csv")))}
emb = np.load(os.path.join(ROOT, "features", "box_emb.npz"), allow_pickle=True)
EI = {u: i for i, u in enumerate(emb["uid"])}
P11, PHSV = emb["pmean11"], emb["hsv"]
P2 = emb["pmean2"]

NUM = ("sem_top1 sem_top3 sem_top5 sem_median sem_mean sem_margin12 appe11_clstop1 appe11_max42 "
       "appe11_mean42 appe9_max42 appe2_max42 appe2_mean42 color_hsv_masked yolo_conf").split()
for r in pairs:
    for k in NUM:
        r[k] = float(r[k])
    r["sim_thr"] = float(r["sim_thr"]); r["appe_gate"] = float(r["appe_gate"])
    r["is_candidate"] = int(r["is_candidate"])
    r["appe_viewmargin"] = r["appe11_max42"] - r["appe11_mean42"]

OBJS = sorted({r["object"] for r in pairs})


def winners(sem_key):
    cand = defaultdict(list)
    for r in pairs:
        if r["is_candidate"]:
            cand[(r["source"], r["frame"], r["object"])].append(r)
    out, lost = [], {}
    for k, v in cand.items():
        b = max(v, key=lambda x: x[sem_key])
        if b[sem_key] < b["sim_thr"]:
            continue
        out.append(b); lost[id(b)] = [x for x in v if x is not b]
    return out, lost


def ok(r):
    l = labels.get(r["uid"])
    return l is not None and l != "unclear"


# 박스별 전 객체 점수(class competition 용)
bybox = defaultdict(dict)
for r in pairs:
    bybox[r["uid"]][r["object"]] = r


def cm(rows, decide):
    c = Counter(); per = defaultdict(Counter); conf = Counter()
    for r in rows:
        a = decide(r); pos = labels[r["uid"]] == r["object"]
        k = "TP" if (a and pos) else "FP" if a else "FN" if pos else "TN"
        c[k] += 1; per[r["object"]][k] += 1
        if k == "FP":
            conf[f'{labels[r["uid"]]}->{r["object"]}'] += 1
    return c, per, conf


def prf(c):
    tp, fp, fn = c["TP"], c["FP"], c["FN"]
    p = tp/(tp+fp) if tp+fp else 0.0; rr = tp/(tp+fn) if tp+fn else 0.0
    return round(p, 4), round(rr, 4), round(2*p*rr/(p+rr), 4) if p+rr else 0.0


def calibrate_per_object(rows, score, min_recall=None):
    """calib rows 에서 객체별 threshold (F1 최대, 선택적 recall 하한)."""
    th = {}
    for o in OBJS:
        sub = [r for r in rows if r["object"] == o]
        if not sub:
            th[o] = 0.0; continue
        cands = sorted({round(score(r), 4) for r in sub})
        best, bt = -1, cands[0] if cands else 0.0
        for t in cands:
            tp = sum(1 for r in sub if score(r) >= t and labels[r["uid"]] == o)
            fp = sum(1 for r in sub if score(r) >= t and labels[r["uid"]] != o)
            fn = sum(1 for r in sub if score(r) < t and labels[r["uid"]] == o)
            rc = tp/(tp+fn) if tp+fn else 0.0
            if min_recall is not None and rc < min_recall:
                continue
            p = tp/(tp+fp) if tp+fp else 0.0
            f1 = 2*p*rc/(p+rc) if p+rc else 0.0
            if f1 > best:
                best, bt = f1, t
        th[o] = bt
    return th


# ---------------------------------------------------------------- negative bank
def build_negbank(calib_rows):
    """calib bag 의 라벨된 '해당 객체가 아닌' 박스 임베딩 = 객체별 negative bank."""
    bank = {}
    uids_by_obj = defaultdict(set)
    for r in calib_rows:
        o = r["object"]
        if labels[r["uid"]] != o:
            uids_by_obj[o].add(r["uid"])
    for o in OBJS:
        idx = [EI[u] for u in uids_by_obj[o] if u in EI]
        bank[o] = (P11[idx], PHSV[idx], P2[idx]) if idx else (None, None, None)
    return bank


def negscore(bank, r):
    """query 와 negative bank 의 최대 유사도 (patch-mean cos, color hist-inter)."""
    o = r["object"]; i = EI.get(r["uid"])
    B11, BH, B2 = bank.get(o, (None, None, None))
    if i is None or B11 is None or len(B11) == 0:
        return 0.0, 0.0
    # 같은 박스가 bank 에 있으면 제외 (self-match 방지)
    c = B11 @ P11[i]
    h = np.minimum(BH, PHSV[i][None]).sum(1)
    m = c < 0.9999
    c = c[m] if m.any() else c
    h = h[m] if m.any() else h
    return float(c.max()), float(h.max())


# ================================================================== 실험 실행
W_top5, LOST_top5 = winners("sem_top5")
W_mean, LOST_mean = winners("sem_mean")
Wc = [r for r in W_top5 if ok(r) and r["source"] in CALIB]
We = [r for r in W_top5 if ok(r) and r["source"] in EVAL]
Wc_m = [r for r in W_mean if ok(r) and r["source"] in CALIB]
We_m = [r for r in W_mean if ok(r) and r["source"] in EVAL]
bank = build_negbank(Wc)
print(f"calib winners {len(Wc)} / eval winners {len(We)}")

for r in pairs:
    if r["uid"] in EI:
        n11, nh = negscore(bank, r)
        r["neg_patch"] = n11; r["neg_color"] = nh
    else:
        r["neg_patch"] = r["neg_color"] = 0.0
    r["disc_patch"] = r["appe11_clstop1"] - r["neg_patch"]
    r["disc_color"] = r["color_hsv_masked"] - r["neg_color"]


def class_margin(r, key="appe11_clstop1"):
    others = [v[key] for o, v in bybox[r["uid"]].items() if o != r["object"]]
    return r[key] - max(others) if others else r[key]


for r in pairs:
    r["margin_appe"] = class_margin(r, "appe11_clstop1")
    r["margin_sem"] = class_margin(r, "sem_mean")

EXPS = {}
def add(name, rows_c, rows_e, score, desc, fixed=None):
    th = calibrate_per_object(rows_c, score)
    c, per, conf = cm(rows_e, lambda r: score(r) >= th[r["object"]])
    p, rc, f1 = prf(c)
    e = {"desc": desc, "recalibrated": {"TP": c["TP"], "FP": c["FP"], "FN": c["FN"], "TN": c["TN"],
         "precision": p, "recall": rc, "f1": f1,
         "carton_to_choco_FP": conf.get("carton->choco_hazelnut_high", 0),
         "doll_confusion_FP": sum(v for k, v in conf.items()
                                  if k.split("->")[0] in DOLLS and k.split("->")[1] in DOLLS),
         "saffron_to_febreze_FP": conf.get("saffron->Febreze_high", 0),
         "confusion": dict(conf.most_common()),
         "thresholds": {k: round(v, 4) for k, v in th.items()}}}
    if fixed is not None:
        c2, per2, conf2 = cm(rows_e, fixed)
        p2, r2, f2 = prf(c2)
        e["fixed_threshold"] = {"TP": c2["TP"], "FP": c2["FP"], "FN": c2["FN"], "TN": c2["TN"],
                                "precision": p2, "recall": r2, "f1": f2,
                                "carton_to_choco_FP": conf2.get("carton->choco_hazelnut_high", 0),
                                "doll_confusion_FP": sum(v for k, v in conf2.items()
                                                         if k.split("->")[0] in DOLLS and k.split("->")[1] in DOLLS),
                                "saffron_to_febreze_FP": conf2.get("saffron->Febreze_high", 0),
                                "confusion": dict(conf2.most_common())}
    e["per_object"] = {o: dict(per[o]) for o in sorted(per)}
    EXPS[name] = e
    fx = e.get("fixed_threshold")
    print(f"{name:5s} {desc[:44]:44s} | recal P{p:.3f} R{rc:.3f} F{f1:.3f} FP{c['FP']:3d} FN{c['FN']:3d}"
          + (f" | fixed P{p2:.3f} R{r2:.3f} FP{c2['FP']:3d} FN{c2['FN']:3d}" if fx else ""))


add("B0", Wc, We, lambda r: r["appe11_clstop1"], "현재 운영 점수 (appe11 @ CLS-top1 view)",
    fixed=lambda r: r["appe11_clstop1"] >= r["appe_gate"])
add("B1", Wc, We, lambda r: r["appe11_clstop1"], "= B0, 객체별 threshold 재보정")
add("B2", Wc, We, lambda r: r["margin_appe"], "class margin (appe11 best - 2nd class)")
add("B2b", Wc, We, lambda r: r["margin_sem"], "class margin (sem_mean best - 2nd class)")
add("B3", Wc, We, lambda r: r["disc_patch"], "negative prototype (appe11 - max neg patch cos)")
add("B3b", Wc, We, lambda r: r["disc_color"], "negative prototype (color - max neg color)")
add("B4", Wc, We, lambda r: r["color_hsv_masked"], "masked HSV color 단독")
add("B4b", Wc, We, lambda r: 0.5*r["appe11_clstop1"] + 0.5*r["color_hsv_masked"], "appe11 + color 균등 가중")
add("B5", Wc, We, lambda r: r["appe2_max42"], "DINOv2 block2 masked patch 단독")
add("B6", Wc, We, lambda r: 0.5*r["appe11_max42"] + 0.5*r["appe2_max42"], "block2 + block11 (42-view max) 균등")
add("B7", Wc_m, We_m, lambda r: r["appe11_clstop1"],
    "semantic 선택/게이트를 sem_mean(42평균)으로 교체")
add("B7b", Wc_m, We_m, lambda r: r["sem_mean"], "sem_mean 자체를 게이트로")
add("B8", Wc, We, lambda r: r["appe_viewmargin"], "appe view-margin (max42 - mean42)")
add("B10", Wc_m, We_m, lambda r: 0.5*r["sem_mean"] + 0.5*r["color_hsv_masked"],
    "권장조합: sem_mean 선택 + (sem_mean+color)/2 게이트")
add("B10b", Wc_m, We_m, lambda r: (r["sem_mean"] + r["color_hsv_masked"] + r["appe11_clstop1"])/3,
    "권장조합+appe11 3항 평균")

# --------------------------------------------------- 특징 간 오류 상관/보완성
def err_set(rows, score, th):
    return {(r["uid"], r["object"]) for r in rows
            if (score(r) >= th[r["object"]]) != (labels[r["uid"]] == r["object"])}


th_a = calibrate_per_object(Wc, lambda r: r["appe11_clstop1"])
th_c = calibrate_per_object(Wc, lambda r: r["color_hsv_masked"])
th_b2 = calibrate_per_object(Wc, lambda r: r["appe2_max42"])
th_sm = calibrate_per_object(Wc_m, lambda r: r["sem_mean"])
ea = err_set(We, lambda r: r["appe11_clstop1"], th_a)
ec = err_set(We, lambda r: r["color_hsv_masked"], th_c)
eb = err_set(We, lambda r: r["appe2_max42"], th_b2)
esm = err_set(We_m, lambda r: r["sem_mean"], th_sm)
comp = {
    "appe11_errors": len(ea), "color_errors": len(ec), "block2_errors": len(eb), "sem_mean_errors": len(esm),
    "appe11 ∩ color": len(ea & ec), "appe11 only": len(ea - ec), "color only": len(ec - ea),
    "appe11 ∩ block2": len(ea & eb), "block2 fixes appe11": len(ea - eb), "block2 breaks": len(eb - ea),
    "color fixes appe11": len(ea - ec), "color breaks": len(ec - ea),
    "appe11 ∩ sem_mean": len(ea & esm), "sem_mean fixes appe11": len(ea - esm), "sem_mean breaks": len(esm - ea),
}
print("\n=== 오류 집합 보완성 (eval bags) ===")
for k, v in comp.items():
    print(f"  {k:26s} {v}")

# 점수 상관 (Spearman 근사: 순위 피어슨)
def spearman(a, b):
    ra = np.argsort(np.argsort(a)); rb = np.argsort(np.argsort(b))
    return float(np.corrcoef(ra, rb)[0, 1])


CAND = [r for r in pairs if r["is_candidate"] and ok(r)]
corr = {}
for f in ("appe11_clstop1", "appe2_max42", "color_hsv_masked", "sem_mean", "sem_top5"):
    for g in ("appe11_clstop1", "appe2_max42", "color_hsv_masked", "sem_mean"):
        if f < g:
            corr[f"{f} ~ {g}"] = round(spearman(np.array([r[f] for r in CAND]),
                                                np.array([r[g] for r in CAND])), 3)
print("\n=== 점수 간 Spearman 상관 (후보쌍 전체) ===")
for k, v in corr.items():
    print(f"  {k:40s} {v}")

# ------------------------------------------------ E-stage: 잃어버린 후보(E3) 상한
e3 = 0; e3_examples = []
for r in W_top5:
    for x in LOST_top5[id(r)]:
        if labels.get(x["uid"]) == r["object"] and labels.get(r["uid"]) != r["object"]:
            e3 += 1; e3_examples.append((r["source"], r["frame"], r["object"], x["uid"]))
print(f"\n=== E3 (semantic 선택 실패: 진짜 객체 박스가 후보에 있었는데 다른 박스가 선택됨) = {e3} 건 ===")

json.dump({"experiments": EXPS, "complementarity": comp, "score_correlation": corr,
           "E3_selection_failure": e3,
           "split": {"calib": sorted(CALIB), "eval": sorted(EVAL),
                     "calib_decisions": len(Wc), "eval_decisions": len(We)}},
          open(os.path.join(RES, "ablation.json"), "w"), indent=2)
print(f"\n-> {RES}/ablation.json")

# ------------------------------------------------ 추가 조합 (권장안 후보 탐색)
add("B11", Wc_m, We_m, lambda r: min(r["sem_mean"], 0.5 + r["margin_sem"]),
    "sem_mean 선택 + (sem_mean AND class margin) 결합")
add("B12", Wc_m, We_m, lambda r: 0.5*r["sem_mean"] + 0.5*r["margin_sem"],
    "sem_mean 선택 + (sem_mean + class margin)/2")
add("B13", Wc_m, We_m, lambda r: (r["sem_mean"] + r["color_hsv_masked"] + r["margin_sem"])/3,
    "sem_mean 선택 + (sem_mean + color + margin)/3")
add("B14", Wc_m, We_m, lambda r: (r["sem_mean"] + r["appe11_clstop1"])/2,
    "sem_mean 선택 + (sem_mean + appe11)/2  (color 없음)")

print("\n=== 주요 후보 객체별 상세 (eval bags) ===")
for nm in ("B0", "B7", "B10b", "B13", "B14"):
    print(f"-- {nm}: {EXPS[nm]['desc']}")
    for o, d in EXPS[nm]["per_object"].items():
        print(f"     {o:22s} TP{d.get('TP',0):3d} FP{d.get('FP',0):3d} FN{d.get('FN',0):3d} TN{d.get('TN',0):3d}")
    print(f"     confusion: {EXPS[nm]['recalibrated']['confusion']}")

json.dump({"experiments": EXPS, "complementarity": comp, "score_correlation": corr,
           "split": {"calib": sorted(CALIB), "eval": sorted(EVAL),
                     "calib_decisions": len(Wc), "eval_decisions": len(We)}},
          open(os.path.join(RES, "ablation.json"), "w"), indent=2)
