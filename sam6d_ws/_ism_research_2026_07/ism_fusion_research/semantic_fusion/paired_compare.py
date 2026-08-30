#!/usr/bin/env python3
"""paired_compare.py — Top-5 대비 각 방식의 FP/FN 변화를 짝지은 비교로 검증한다 (READ-ONLY).

eval_fusion.py 의 합계 표는 FN 이 11~22 건 규모라 1~3 건 차이가 잡음일 수 있다.
여기서는 **같은 (dataset, frame, object) 그룹**에서 두 방식의 결과를 짝지어
  b = Top-5 는 맞았는데 새 방식이 틀린 그룹  (신규 오류)
  c = Top-5 는 틀렸는데 새 방식이 맞은 그룹  (수정된 오류)
를 세고, McNemar 정확검정(이항)과 FP/FN 차이의 부트스트랩 95% 구간을 낸다.

산출: results/semantic_paired_vs_top5.csv, results/semantic_error_cases.csv
"""
import csv, math, os
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RES = os.path.join(ROOT, "results")

rows = list(csv.DictReader(open(os.path.join(RES, "semantic_error_delta.csv"))))
by = defaultdict(dict)          # scheme -> key -> outcome
for r in rows:
    by[r["scheme"]][(r["dataset"], r["frame_id"], r["object"])] = r
SCHEMES = sorted(by)
BASE = "top5"
keys = sorted(by[BASE])
print(f"짝지을 그룹 {len(keys)} | 방식 {SCHEMES}")


def binom_two_sided(b, c):
    """McNemar 정확검정 (b+c 시행, p=0.5)."""
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    s = sum(math.comb(n, i) for i in range(k + 1)) / (2 ** n)
    return min(1.0, 2 * s)


def boot_ci(pairs, fn, B=5000, seed=0):
    """그룹 단위 부트스트랩으로 (새방식 - Top5) 차이의 95% 구간."""
    rs = np.random.RandomState(seed)
    n = len(pairs)
    idx = rs.randint(0, n, size=(B, n))
    v = np.array([fn([pairs[j] for j in row]) for row in idx])
    return float(np.percentile(v, 2.5)), float(np.percentile(v, 97.5))


out, cases = [], []
for s in SCHEMES:
    if s == BASE:
        continue
    pairs = [(by[BASE][k]["outcome"], by[s].get(k, {}).get("outcome")) for k in keys
             if k in by[s]]
    if not pairs:
        continue
    correct = lambda o: o in ("TP", "TN")
    b = sum(1 for a, x in pairs if correct(a) and not correct(x))
    c = sum(1 for a, x in pairs if not correct(a) and correct(x))
    dFP = sum(1 for a, x in pairs if x == "FP") - sum(1 for a, x in pairs if a == "FP")
    dFN = sum(1 for a, x in pairs if x == "FN") - sum(1 for a, x in pairs if a == "FN")
    fp_lo, fp_hi = boot_ci(pairs, lambda P: sum(1 for a, x in P if x == "FP")
                           - sum(1 for a, x in P if a == "FP"))
    fn_lo, fn_hi = boot_ci(pairs, lambda P: sum(1 for a, x in P if x == "FN")
                           - sum(1 for a, x in P if a == "FN"))
    out.append({
        "scheme": s, "n_paired_group": len(pairs),
        "new_errors_b": b, "fixed_errors_c": c,
        "mcnemar_p": round(binom_two_sided(b, c), 4),
        "delta_FP": dFP, "delta_FP_ci95": f"[{fp_lo:+.0f}, {fp_hi:+.0f}]",
        "delta_FN": dFN, "delta_FN_ci95": f"[{fn_lo:+.0f}, {fn_hi:+.0f}]",
        "FP_significantly_better": int(fp_hi < 0),
        "FN_significantly_worse": int(fn_lo > 0),
        "neither_worse_point": int(dFP <= 0 and dFN <= 0),
    })
    for k in keys:
        if k not in by[s]:
            continue
        a, x = by[BASE][k]["outcome"], by[s][k]["outcome"]
        if a != x:
            cases.append({"scheme": s, "dataset": k[0], "frame_id": k[1], "object": k[2],
                          "top5_outcome": a, "scheme_outcome": x,
                          "top5_selected_true": by[BASE][k]["selected_true_class"],
                          "scheme_selected_true": by[s][k]["selected_true_class"],
                          "changed_selection": int(by[BASE][k]["selected_uid"]
                                                   != by[s][k]["selected_uid"])})

out.sort(key=lambda r: (r["delta_FP"] + r["delta_FN"]))
with open(os.path.join(RES, "semantic_paired_vs_top5.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(out[0].keys())); w.writeheader(); w.writerows(out)
with open(os.path.join(RES, "semantic_error_cases.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(cases[0].keys())); w.writeheader(); w.writerows(cases)

print(f"\n{'scheme':16s} {'신규오류b':>8} {'수정오류c':>8} {'McNemar p':>10} "
      f"{'ΔFP':>5} {'ΔFP 95%CI':>13} {'ΔFN':>5} {'ΔFN 95%CI':>13}  판정")
for r in out:
    v = ("FP 유의개선·FN 악화없음" if r["FP_significantly_better"] and not r["FN_significantly_worse"]
         else "FN 유의악화" if r["FN_significantly_worse"]
         else "차이 불충분")
    print(f"{r['scheme']:16s} {r['new_errors_b']:>8} {r['fixed_errors_c']:>8} "
          f"{r['mcnemar_p']:>10.4f} {r['delta_FP']:>+5} {r['delta_FP_ci95']:>13} "
          f"{r['delta_FN']:>+5} {r['delta_FN_ci95']:>13}  {v}")
print(f"\n-> {os.path.join(RES,'semantic_paired_vs_top5.csv')}")
print(f"-> {os.path.join(RES,'semantic_error_cases.csv')}  ({len(cases)} 건)")
