#!/usr/bin/env python3
"""analyze_aggregation.py — sem_top5 vs sem_mean 의 객체별 우열 원인 규명 (READ-ONLY).

가설:
  두 점수의 차이는 오직 "42개 유사도 곡선의 모양" 에서 나온다.
    sem_top5 = 상위 5개만 본다  → "최선의 view 가 얼마나 잘 맞는가"
    sem_mean = 42개 전부를 본다 → "모든 view 에 걸쳐 평균적으로 얼마나 맞는가"
  따라서 positive 곡선이 뾰족하면(소수 view 만 높음) top5 가 유리하고,
  negative 곡선이 뾰족하면(우연히 몇 view 만 맞음) mean 이 유리하다.

  검증량: peakiness = (top5평균 - 전체평균) / (전체 std + eps)
          Δpeak = mean(peakiness | negative) - mean(peakiness | positive)
          Δpeak > 0  →  negative 가 더 뾰족 → mean 이 유리해야 함

산출:
  results/semantic_aggregation_by_object.csv
  results/semantic_view_profile.csv          (view 단위 집계: 불량 view 탐지)
"""
import csv, json, os
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RES = os.path.join(ROOT, "results")
EPS = 1e-9


def auroc(pos, neg):
    if not pos or not neg:
        return None
    a = np.array(pos)[:, None] > np.array(neg)[None, :]
    t = np.array(pos)[:, None] == np.array(neg)[None, :]
    return float((a.sum() + 0.5 * t.sum()) / (len(pos) * len(neg)))


def pr_auc(scores, ys):
    o = np.argsort(-np.asarray(scores))
    y = np.asarray(ys)[o]
    tp = np.cumsum(y); fp = np.cumsum(1 - y)
    prec = tp / np.maximum(tp + fp, 1)
    rec = tp / max(y.sum(), 1)
    return float(np.sum(np.diff(np.concatenate([[0.], rec])) * prec))


rows = list(csv.DictReader(open(os.path.join(RES, "semantic_view_similarity.csv"))))
for r in rows:
    r["sims"] = np.array([float(x) for x in r["sims"].split(";")], dtype=np.float64)
    r["is_positive"] = int(r["is_positive"])
print(f"pair {len(rows)}  (positive {sum(r['is_positive'] for r in rows)})")

by_obj = defaultdict(list)
for r in rows:
    by_obj[r["object"]].append(r)

out, view_rows = [], []
for obj in sorted(by_obj):
    R = by_obj[obj]
    pos = [r for r in R if r["is_positive"]]
    neg = [r for r in R if not r["is_positive"]]
    if len(pos) < 3 or len(neg) < 3:
        continue

    def agg(rs, fn):
        return [float(fn(r["sims"])) for r in rs]

    top5 = lambda s: np.sort(s)[::-1][:5].mean()
    mean = lambda s: s.mean()

    a5 = auroc(agg(pos, top5), agg(neg, top5))
    am = auroc(agg(pos, mean), agg(neg, mean))
    sc5 = agg(pos, top5) + agg(neg, top5)
    scm = agg(pos, mean) + agg(neg, mean)
    ys = [1] * len(pos) + [0] * len(neg)

    def peak(rs):
        v = []
        for r in rs:
            s = r["sims"]
            v.append((np.sort(s)[::-1][:5].mean() - s.mean()) / (s.std() + EPS))
        return np.array(v)

    pk_p, pk_n = peak(pos), peak(neg)
    # 곡선 폭: 상위 5 view 가 전체 대비 얼마나 앞서는가 (절대 격차)
    gap_p = np.array([np.sort(r["sims"])[::-1][:5].mean() - r["sims"].mean() for r in pos])
    gap_n = np.array([np.sort(r["sims"])[::-1][:5].mean() - r["sims"].mean() for r in neg])
    # 곡선 전체 표준편차 (view 간 변동)
    sd_p = np.array([r["sims"].std() for r in pos])
    sd_n = np.array([r["sims"].std() for r in neg])

    out.append({
        "object": obj, "n_pos": len(pos), "n_neg": len(neg),
        "auroc_top5": round(a5, 4), "auroc_mean": round(am, 4),
        "delta_auroc": round(am - a5, 4),
        "prauc_top5": round(pr_auc(sc5, ys), 4), "prauc_mean": round(pr_auc(scm, ys), 4),
        "pos_top5_mu": round(float(np.mean(agg(pos, top5))), 4),
        "pos_mean_mu": round(float(np.mean(agg(pos, mean))), 4),
        "neg_top5_mu": round(float(np.mean(agg(neg, top5))), 4),
        "neg_mean_mu": round(float(np.mean(agg(neg, mean))), 4),
        # 분리도 (Cohen's d): 두 점수 체계가 pos/neg 를 얼마나 떼어놓는가
        "d_top5": round(float((np.mean(agg(pos, top5)) - np.mean(agg(neg, top5))) /
                              (np.sqrt((np.var(agg(pos, top5)) + np.var(agg(neg, top5))) / 2) + EPS)), 3),
        "d_mean": round(float((np.mean(agg(pos, mean)) - np.mean(agg(neg, mean))) /
                              (np.sqrt((np.var(agg(pos, mean)) + np.var(agg(neg, mean))) / 2) + EPS)), 3),
        "peakiness_pos": round(float(pk_p.mean()), 4),
        "peakiness_neg": round(float(pk_n.mean()), 4),
        "delta_peakiness": round(float(pk_n.mean() - pk_p.mean()), 4),
        "gap_pos": round(float(gap_p.mean()), 4),
        "gap_neg": round(float(gap_n.mean()), 4),
        "delta_gap": round(float(gap_n.mean() - gap_p.mean()), 4),
        "viewstd_pos": round(float(sd_p.mean()), 4),
        "viewstd_neg": round(float(sd_n.mean()), 4),
    })

    # view 단위 프로파일 (불량 view 탐지)
    P = np.stack([r["sims"] for r in pos])
    N = np.stack([r["sims"] for r in neg])
    for v in range(P.shape[1]):
        a = auroc(list(P[:, v]), list(N[:, v]))
        view_rows.append({
            "object": obj, "view": v,
            "pos_mu": round(float(P[:, v].mean()), 4),
            "neg_mu": round(float(N[:, v].mean()), 4),
            "sep": round(float(P[:, v].mean() - N[:, v].mean()), 4),
            "auroc_view": round(a, 4) if a is not None else "",
            "pos_top1_share": round(float(np.mean(np.argmax(P, 1) == v)), 4),
            "neg_top1_share": round(float(np.mean(np.argmax(N, 1) == v)), 4),
            "in_pos_top5_share": round(float(np.mean(
                [v in np.argsort(-P[i])[:5] for i in range(len(P))])), 4),
            "in_neg_top5_share": round(float(np.mean(
                [v in np.argsort(-N[i])[:5] for i in range(len(N))])), 4),
        })

out.sort(key=lambda r: -r["delta_auroc"])
with open(os.path.join(RES, "semantic_aggregation_by_object.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(out[0].keys())); w.writeheader(); w.writerows(out)
with open(os.path.join(RES, "semantic_view_profile.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(view_rows[0].keys())); w.writeheader(); w.writerows(view_rows)

print(f"\n{'객체':22s} {'pos':>4} {'neg':>4} {'AUC_t5':>7} {'AUC_mn':>7} {'Δ':>7} "
      f"{'peak_p':>7} {'peak_n':>7} {'Δpeak':>7} {'d_t5':>6} {'d_mn':>6}")
for r in out:
    print(f"{r['object']:22s} {r['n_pos']:>4} {r['n_neg']:>4} {r['auroc_top5']:>7.3f} "
          f"{r['auroc_mean']:>7.3f} {r['delta_auroc']:>+7.3f} {r['peakiness_pos']:>7.3f} "
          f"{r['peakiness_neg']:>7.3f} {r['delta_peakiness']:>+7.3f} {r['d_top5']:>6.2f} {r['d_mean']:>6.2f}")

# 상관: Δpeakiness 가 ΔAUROC 를 설명하는가
dp = np.array([r["delta_peakiness"] for r in out])
da = np.array([r["delta_auroc"] for r in out])
dg = np.array([r["delta_gap"] for r in out])
print(f"\ncorr(Δpeakiness, ΔAUROC) = {np.corrcoef(dp, da)[0,1]:+.3f}   (n={len(out)})")
print(f"corr(Δgap,       ΔAUROC) = {np.corrcoef(dg, da)[0,1]:+.3f}")
json.dump({"corr_delta_peakiness_vs_delta_auroc": float(np.corrcoef(dp, da)[0, 1]),
           "corr_delta_gap_vs_delta_auroc": float(np.corrcoef(dg, da)[0, 1]),
           "n_objects": len(out)},
          open(os.path.join(RES, "aggregation_mechanism.json"), "w"), indent=2)
