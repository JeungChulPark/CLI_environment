#!/usr/bin/env python3
"""Template Selection Failure 분석 (stdlib 전용, 레포 코드 미수정).

입력:
  --debug   _run/debug/sam6d_debug.csv  (best_template_id / top5_template_scores 계측 포함)
  --frames  frame_results.csv           (frame_id, decision_type=TP/FP/FN/TN)

출력(요청 형식):
  1) Template Usage Histogram
  2) TP vs FP Template Usage
  3) Per-template TP / FP / Precision
  4) Top1-Top2 Margin
  5) Top1-Top5 Margin
  6) Selection Failure Verdict (Confirmed / Rejected / Inconclusive)

판정 기준은 아래 verdict() 에 명시. Inconclusive 는 계측/정합 실패 시에만.
"""
import csv
import json
import argparse
import statistics as st
from collections import Counter, defaultdict

BASE = "outputs/validation/SLAM_with_milk_nomilk"


def load_frames(path):
    """frame_id -> decision_type (TP/FP/FN/TN)."""
    dt = {}
    with open(path) as fh:
        for r in csv.DictReader(fh):
            dt[r["frame_id"]] = r.get("decision_type", "")
    return dt


def load_best_rows(path):
    """프레임별 is_best 후보 1행만 추출. frame_id -> row dict."""
    best = {}
    with open(path) as fh:
        for r in csv.DictReader(fh):
            if str(r.get("is_best", "")).strip().lower() != "true":
                continue
            best[r["frame_id"]] = r
    return best


def _to_int(x):
    try:
        return int(float(x))
    except (TypeError, ValueError):
        return None


def _parse_list(x):
    if not x:
        return []
    try:
        v = json.loads(x)
        return v if isinstance(v, list) else []
    except (TypeError, ValueError, json.JSONDecodeError):
        return []


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--debug", default=f"{BASE}/_run/debug/sam6d_debug.csv")
    ap.add_argument("--frames", default=f"{BASE}/frame_results.csv")
    args = ap.parse_args()

    dt = load_frames(args.frames)
    best = load_best_rows(args.debug)

    # join: frame_id 기준
    common = [f for f in best if f in dt]
    print("=" * 64)
    print("TEMPLATE SELECTION ANALYSIS")
    print("=" * 64)
    print(f"debug is_best rows : {len(best)}")
    print(f"frame_results rows : {len(dt)}")
    print(f"joined frames      : {len(common)}")

    # 계측/정합 실패 감지
    have_tid = [best[f].get("best_template_id") for f in common]
    n_valid_tid = sum(1 for x in have_tid if _to_int(x) is not None)
    instrumentation_ok = (len(common) > 0 and n_valid_tid >= max(1, int(0.5 * len(common))))
    print(f"valid best_template_id : {n_valid_tid}/{len(common)}")

    # decision_type 별 best_template_id 수집
    by_dt = defaultdict(list)   # dt -> [tid,...]
    margins12, margins15 = [], []
    for f in common:
        row = best[f]
        tid = _to_int(row.get("best_template_id"))
        d = dt[f]
        if tid is not None:
            by_dt[d].append(tid)
        top5 = _parse_list(row.get("top5_template_scores"))
        if len(top5) >= 2:
            margins12.append(round(float(top5[0]) - float(top5[1]), 4))
        if len(top5) >= 2:
            margins15.append(round(float(top5[0]) - float(top5[-1]), 4))

    all_tids = [t for lst in by_dt.values() for t in lst]
    usage = Counter(all_tids)
    tp_use = Counter(by_dt.get("TP", []))
    fp_use = Counter(by_dt.get("FP", []))

    # ── 1) Usage Histogram ──
    print("\n[1] TEMPLATE USAGE HISTOGRAM (is_best 기준, 전체)")
    if usage:
        for t in range(0, 42):
            n = usage.get(t, 0)
            if n:
                print(f"  template_{t:02d} : {n}  {'#'*min(n,50)}")
        print(f"  (사용된 distinct template = {len(usage)}/42, 총 {sum(usage.values())}프레임)")
    else:
        print("  (no data)")

    # ── 2) TP vs FP usage ──
    print("\n[2] TP vs FP TEMPLATE USAGE")
    tp_total = sum(tp_use.values())
    fp_total = sum(fp_use.values())
    print(f"  TP frames={tp_total}, FP frames={fp_total}")
    print(f"  {'tid':>5} | {'TP':>4} | {'FP':>4}")
    for t in sorted(set(tp_use) | set(fp_use)):
        print(f"  {t:>5} | {tp_use.get(t,0):>4} | {fp_use.get(t,0):>4}")
    # FP 집중도
    fp_top_share = (fp_use.most_common(1)[0][1] / fp_total) if fp_total else 0.0
    fp_top_tid = fp_use.most_common(1)[0][0] if fp_total else None
    print(f"  → FP 최다 template = t{fp_top_tid}, 점유율 = {fp_top_share:.2%}")

    # ── 3) Per-template precision ──
    print("\n[3] PER-TEMPLATE PRECISION (TP/(TP+FP))")
    print(f"  {'tid':>5} | {'TP':>4} | {'FP':>4} | {'prec':>6} | support")
    precisions = []
    for t in sorted(set(tp_use) | set(fp_use)):
        tp_n, fp_n = tp_use.get(t, 0), fp_use.get(t, 0)
        sup = tp_n + fp_n
        prec = tp_n / sup if sup else 0.0
        if sup >= 5:
            precisions.append(prec)
        print(f"  {t:>5} | {tp_n:>4} | {fp_n:>4} | {prec:>6.3f} | {sup}")
    prec_std = round(st.pstdev(precisions), 3) if len(precisions) >= 2 else 0.0
    print(f"  → support>=5 template precision std = {prec_std} (n={len(precisions)})")

    # ── 4)/5) margins ──
    def desc(v):
        if not v:
            return "n=0"
        return (f"n={len(v)} mean={st.mean(v):.4f} median={st.median(v):.4f} "
                f"min={min(v):.4f} max={max(v):.4f}")
    print("\n[4] TOP1-TOP2 MARGIN (argmax 안정성)")
    print(f"  {desc(margins12)}")
    print("\n[5] TOP1-TOP5 MARGIN")
    print(f"  {desc(margins15)}")
    med_m12 = st.median(margins12) if margins12 else None

    # ── 6) Verdict ──
    print("\n" + "=" * 64)
    print("[6] SELECTION FAILURE VERDICT")
    print("=" * 64)
    reasons = []
    if not instrumentation_ok:
        verdict = "Inconclusive"
        reasons.append(f"계측/정합 실패: valid best_template_id {n_valid_tid}/{len(common)}")
    else:
        confirmed_signals = []
        # (a) FP collapse: 단일 template 가 FP 의 50%+ 점유
        if fp_total >= 10 and fp_top_share >= 0.50:
            confirmed_signals.append(
                f"FP collapse: t{fp_top_tid} 가 FP {fp_top_share:.0%} 점유")
        # (b) 저-precision 고-FP template
        low_prec = [(t, tp_use.get(t,0), fp_use.get(t,0))
                    for t in set(fp_use)
                    if fp_use.get(t,0) >= 10 and
                    (tp_use.get(t,0)/(tp_use.get(t,0)+fp_use.get(t,0))) <= 0.20]
        if low_prec:
            confirmed_signals.append(f"저-precision 고-FP template: {low_prec}")
        # (c) 근소차 argmax (near-tie)
        if med_m12 is not None and med_m12 < 0.005:
            confirmed_signals.append(f"argmax near-tie: median top1-top2 margin={med_m12:.4f}")
        # (d) TP/FP 최다 template 분리 + 집중
        tp_top = tp_use.most_common(1)[0][0] if tp_total else None
        if (fp_total >= 10 and tp_total >= 10 and fp_top_tid != tp_top
                and fp_top_share >= 0.40):
            confirmed_signals.append(
                f"TP최다 t{tp_top} ≠ FP최다 t{fp_top_tid} 이며 FP 집중 {fp_top_share:.0%}")

        if confirmed_signals:
            verdict = "Confirmed"
            reasons = confirmed_signals
        else:
            verdict = "Rejected"
            reasons.append(f"FP 최다 점유율 {fp_top_share:.0%}(<50%)로 분산")
            reasons.append(f"support>=5 precision std {prec_std} (편차 작음)")
            if med_m12 is not None:
                reasons.append(f"median top1-top2 margin {med_m12:.4f} (near-tie 아님)")

    print(f"\n  Selection Failure = {verdict}")
    for r in reasons:
        print(f"   - {r}")
    print("\n  [판정기준] Confirmed=(a)FP collapse≥50% | (b)FP≥10 & precision≤0.20 |")
    print("             (c)median top1-top2 margin<0.005 | (d)TP최다≠FP최다 & FP집중≥40%")
    print("            Rejected=위 신호 없음. Inconclusive=계측/정합 실패만.")
    return verdict


if __name__ == "__main__":
    main()
