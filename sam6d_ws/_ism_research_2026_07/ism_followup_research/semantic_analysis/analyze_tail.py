#!/usr/bin/env python3
"""analyze_tail.py — sem_mean 우열의 직접 분해 (READ-ONLY).

sem_mean = (5/42)·sem_top5 + (37/42)·sem_tail    (sem_tail = 6~42위 view 의 평균)

따라서 mean 이 top5 보다 나은지는 오직 두 가지가 결정한다.
  (1) 꼬리(6~42위) 자체가 변별력을 갖는가         → auroc_tail
  (2) 꼬리가 top5 와 다른 오류를 내는가(비중복)   → corr(top5, tail)

꼬리가 변별력이 있으면 섞을수록 좋아지고, 잡음이면 희석된다.
추가로 순위 k 별 AUROC 곡선을 그려 "어느 순위 구간이 신호를 갖는가" 를 본다.

산출:
  results/semantic_rank_auroc.csv        객체 × 순위 k 의 AUROC
  results/semantic_tail_decomposition.csv
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
    gt = (p[:, None] > n[None, :]).sum()
    eq = (p[:, None] == n[None, :]).sum()
    return float((gt + 0.5 * eq) / (len(p) * len(n)))


rows = list(csv.DictReader(open(os.path.join(RES, "semantic_view_similarity.csv"))))
for r in rows:
    r["sims"] = np.array([float(x) for x in r["sims"].split(";")])
    r["is_positive"] = int(r["is_positive"])

by_obj = defaultdict(list)
for r in rows:
    by_obj[r["object"]].append(r)

dec, rank_rows = [], []
for obj in sorted(by_obj):
    R = by_obj[obj]
    pos = [r for r in R if r["is_positive"]]
    neg = [r for r in R if not r["is_positive"]]
    if len(pos) < 3 or len(neg) < 3:
        continue
    P = np.sort(np.stack([r["sims"] for r in pos]), 1)[:, ::-1]   # 내림차순 정렬
    N = np.sort(np.stack([r["sims"] for r in neg]), 1)[:, ::-1]
    K = P.shape[1]

    a_top5 = auroc(P[:, :5].mean(1), N[:, :5].mean(1))
    a_tail = auroc(P[:, 5:].mean(1), N[:, 5:].mean(1))
    a_mean = auroc(P.mean(1), N.mean(1))
    a_top1 = auroc(P[:, 0], N[:, 0])
    a_bot5 = auroc(P[:, -5:].mean(1), N[:, -5:].mean(1))

    # top5 와 tail 이 서로 다른 샘플에서 틀리는가 (오류 독립성)
    s5 = np.concatenate([P[:, :5].mean(1), N[:, :5].mean(1)])
    st = np.concatenate([P[:, 5:].mean(1), N[:, 5:].mean(1)])
    y = np.concatenate([np.ones(len(P)), np.zeros(len(N))])
    corr = float(np.corrcoef(s5, st)[0, 1])

    # top5 로는 틀리는데 tail 로는 맞는 pos/neg 쌍의 비율 (역전 가능량)
    thr5 = np.median(s5); thrt = np.median(st)
    fix = int(np.sum((y == 1) & (s5 < np.median(s5[y == 0])) & (st > np.median(st[y == 0]))))

    dec.append({"object": obj, "n_pos": len(P), "n_neg": len(N),
                "auroc_top1": round(a_top1, 4), "auroc_top5": round(a_top5, 4),
                "auroc_tail6_42": round(a_tail, 4), "auroc_bottom5": round(a_bot5, 4),
                "auroc_mean": round(a_mean, 4),
                "tail_minus_top5": round(a_tail - a_top5, 4),
                "mean_minus_top5": round(a_mean - a_top5, 4),
                "corr_top5_tail": round(corr, 3),
                "pos_tail_mu": round(float(P[:, 5:].mean()), 4),
                "neg_tail_mu": round(float(N[:, 5:].mean()), 4),
                "tail_sep": round(float(P[:, 5:].mean() - N[:, 5:].mean()), 4),
                "top5_sep": round(float(P[:, :5].mean() - N[:, :5].mean()), 4),
                "n_pos_recoverable_by_tail": fix})

    for k in range(K):
        rank_rows.append({"object": obj, "rank": k + 1,
                          "auroc": round(auroc(P[:, k], N[:, k]), 4),
                          "pos_mu": round(float(P[:, k].mean()), 4),
                          "neg_mu": round(float(N[:, k].mean()), 4),
                          "sep": round(float(P[:, k].mean() - N[:, k].mean()), 4)})

dec.sort(key=lambda r: -r["mean_minus_top5"])
with open(os.path.join(RES, "semantic_tail_decomposition.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(dec[0].keys())); w.writeheader(); w.writerows(dec)
with open(os.path.join(RES, "semantic_rank_auroc.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rank_rows[0].keys())); w.writeheader(); w.writerows(rank_rows)

print(f"{'객체':22s} {'AUC@1':>6} {'AUC_t5':>7} {'AUC_꼬리':>8} {'AUC_mn':>7} "
      f"{'꼬리-t5':>8} {'mn-t5':>7} {'corr':>6} {'t5분리':>7} {'꼬리분리':>8}")
for r in dec:
    print(f"{r['object']:22s} {r['auroc_top1']:>6.3f} {r['auroc_top5']:>7.3f} "
          f"{r['auroc_tail6_42']:>8.3f} {r['auroc_mean']:>7.3f} "
          f"{r['tail_minus_top5']:>+8.3f} {r['mean_minus_top5']:>+7.3f} "
          f"{r['corr_top5_tail']:>6.2f} {r['top5_sep']:>+7.4f} {r['tail_sep']:>+8.4f}")

t = np.array([r["tail_minus_top5"] for r in dec])
m = np.array([r["mean_minus_top5"] for r in dec])
print(f"\ncorr(꼬리AUROC-top5AUROC , meanAUROC-top5AUROC) = {np.corrcoef(t, m)[0,1]:+.3f}  (n={len(dec)})")

# 순위별 AUROC 곡선 요약: 신호가 어느 순위까지 살아있는가
print(f"\n{'객체':22s} " + " ".join(f"r{k:<2d}" for k in (1, 3, 5, 10, 20, 30, 42)))
by = defaultdict(dict)
for r in rank_rows:
    by[r["object"]][r["rank"]] = r["auroc"]
for o in sorted(by):
    print(f"{o:22s} " + " ".join(f"{by[o][k]:.2f}" for k in (1, 3, 5, 10, 20, 30, 42)))
