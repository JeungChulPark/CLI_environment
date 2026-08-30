#!/usr/bin/env python3
"""sweep_aggregation.py — aggregation 변형 스윕 + LODO 검증 (READ-ONLY).

analyze_tail.py 에서 mean 우열 = 꼬리 변별력 임이 확정됐다(corr +1.000).
순위별 AUROC 곡선을 보면 객체군마다 신호 구간이 다르므로 여기서는
"어떤 aggregation 이 구조적으로 맞는가" 를 데이터 분리 하에 확인한다.

변형:
  top{k}           상위 k개 평균          (k = 1,3,5,10,20,30,42)
  trim{a}_{b}      상·하위 절단 평균
  bottom{k}        하위 k개 평균          (Dinosaur 가설 검증용)
  rank_weighted    순위 가중 (선형 감쇠)

데이터 분리:
  객체별 최적 k 를 전체 데이터로 고르고 같은 데이터로 성능을 보고하면 과적합이다.
  → LODO: 6개 데이터셋 중 5개로 k 를 고르고 남은 1개에서만 채점, 6회 반복.
  → 비교 대상인 top5 / mean 은 고정 규칙이므로 같은 held-out 에서 함께 채점한다.

산출: results/semantic_aggregation_sweep.csv, results/semantic_lodo_aggregation.csv
"""
import csv, os
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RES = os.path.join(ROOT, "results")


def auroc(pos, neg):
    if len(pos) == 0 or len(neg) == 0:
        return None
    p, n = np.asarray(pos, float), np.asarray(neg, float)
    return float(((p[:, None] > n[None, :]).sum() + 0.5 * (p[:, None] == n[None, :]).sum())
                 / (len(p) * len(n)))


def make_variants(K=42):
    V = {}
    for k in (1, 2, 3, 5, 8, 10, 15, 20, 25, 30, 35, 42):
        V[f"top{k}"] = (lambda k: (lambda S: S[:, :k].mean(1)))(k)
    for k in (1, 3, 5, 10):
        V[f"bottom{k}"] = (lambda k: (lambda S: S[:, -k:].mean(1)))(k)
    for a, b in ((0, 1), (0, 2), (0, 5), (2, 2), (5, 5), (5, 10), (10, 10)):
        V[f"trim{a}_{b}"] = (lambda a, b: (lambda S: S[:, a:S.shape[1] - b].mean(1)))(a, b)
    w = np.linspace(1.0, 0.0, K)
    V["rank_linear"] = lambda S: (S * w).sum(1) / w.sum()
    w2 = 1.0 / np.arange(1, K + 1)
    V["rank_harmonic"] = lambda S: (S * w2).sum(1) / w2.sum()
    return V


rows = list(csv.DictReader(open(os.path.join(RES, "semantic_view_similarity.csv"))))
for r in rows:
    r["S"] = np.sort(np.array([float(x) for x in r["sims"].split(";")]))[::-1]
    r["is_positive"] = int(r["is_positive"])

VAR = make_variants()
by_obj = defaultdict(list)
for r in rows:
    by_obj[r["object"]].append(r)

# ------------------------------------------------------------ 1) 전체 데이터 스윕 (탐색용)
sweep = []
for obj in sorted(by_obj):
    R = by_obj[obj]
    pos = [r for r in R if r["is_positive"]]
    neg = [r for r in R if not r["is_positive"]]
    if len(pos) < 3 or len(neg) < 3:
        continue
    P = np.stack([r["S"] for r in pos]); N = np.stack([r["S"] for r in neg])
    row = {"object": obj, "n_pos": len(P), "n_neg": len(N)}
    for name, fn in VAR.items():
        row[name] = round(auroc(fn(P), fn(N)), 4)
    best = max(VAR, key=lambda k: row[k])
    row["best_variant"] = best; row["best_auroc"] = row[best]
    row["gain_vs_top5"] = round(row[best] - row["top5"], 4)
    sweep.append(row)

with open(os.path.join(RES, "semantic_aggregation_sweep.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(sweep[0].keys())); w.writeheader(); w.writerows(sweep)

keys = ["top1", "top3", "top5", "top10", "top20", "top42", "trim0_1", "trim0_5",
        "trim5_5", "rank_linear", "bottom5"]
print("=== 전체 데이터 스윕 (탐색용 — 이 표로 성능을 주장하지 않는다)")
print(f"{'객체':22s} " + " ".join(f"{k:>10s}" for k in keys) + "   best")
for r in sweep:
    print(f"{r['object']:22s} " + " ".join(f"{r[k]:>10.3f}" for k in keys)
          + f"   {r['best_variant']}({r['best_auroc']:.3f})")
print(f"{'평균':22s} " + " ".join(
    f"{np.mean([r[k] for r in sweep]):>10.3f}" for k in keys))

# ------------------------------------------------------------ 2) LODO 검증
DS = sorted({r["dataset"] for r in rows})
lodo = []
for held in DS:
    for obj in sorted(by_obj):
        R = by_obj[obj]
        tr = [r for r in R if r["dataset"] != held]
        te = [r for r in R if r["dataset"] == held]
        trp = [r for r in tr if r["is_positive"]]; trn = [r for r in tr if not r["is_positive"]]
        tep = [r for r in te if r["is_positive"]]; ten = [r for r in te if not r["is_positive"]]
        if len(trp) < 3 or len(trn) < 3 or len(tep) < 2 or len(ten) < 2:
            continue
        TP = np.stack([r["S"] for r in trp]); TN = np.stack([r["S"] for r in trn])
        EP = np.stack([r["S"] for r in tep]); EN = np.stack([r["S"] for r in ten])
        # 학습 fold 에서만 최적 변형 선택
        sel = max(VAR, key=lambda k: auroc(VAR[k](TP), VAR[k](TN)))
        lodo.append({
            "held_out": held, "object": obj,
            "n_test_pos": len(EP), "n_test_neg": len(EN),
            "selected_variant": sel,
            "auroc_selected": round(auroc(VAR[sel](EP), VAR[sel](EN)), 4),
            "auroc_top5": round(auroc(VAR["top5"](EP), VAR["top5"](EN)), 4),
            "auroc_mean": round(auroc(VAR["top42"](EP), VAR["top42"](EN)), 4),
            "auroc_trim0_1": round(auroc(VAR["trim0_1"](EP), VAR["trim0_1"](EN)), 4),
            "auroc_trim0_5": round(auroc(VAR["trim0_5"](EP), VAR["trim0_5"](EN)), 4),
        })

with open(os.path.join(RES, "semantic_lodo_aggregation.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(lodo[0].keys())); w.writeheader(); w.writerows(lodo)

print("\n=== LODO held-out 평균 (fold 에서 고른 변형을 남은 데이터셋에서만 채점)")
per = defaultdict(lambda: defaultdict(list))
for r in lodo:
    for k in ("auroc_selected", "auroc_top5", "auroc_mean", "auroc_trim0_1", "auroc_trim0_5"):
        per[r["object"]][k].append(r[k])
print(f"{'객체':22s} {'n_fold':>6} {'선택형':>8} {'top5':>8} {'mean':>8} {'trim0_1':>8} {'trim0_5':>8}  최빈선택")
tot = defaultdict(list)
for o in sorted(per):
    v = per[o]
    picks = [r["selected_variant"] for r in lodo if r["object"] == o]
    mode = max(set(picks), key=picks.count)
    print(f"{o:22s} {len(v['auroc_top5']):>6} "
          f"{np.mean(v['auroc_selected']):>8.3f} {np.mean(v['auroc_top5']):>8.3f} "
          f"{np.mean(v['auroc_mean']):>8.3f} {np.mean(v['auroc_trim0_1']):>8.3f} "
          f"{np.mean(v['auroc_trim0_5']):>8.3f}  {mode} ({picks.count(mode)}/{len(picks)})")
    for k in v:
        tot[k] += v[k]
print(f"{'전체평균':22s} {len(tot['auroc_top5']):>6} " + " ".join(
    f"{np.mean(tot[k]):>8.3f}" for k in
    ("auroc_selected", "auroc_top5", "auroc_mean", "auroc_trim0_1", "auroc_trim0_5")))
