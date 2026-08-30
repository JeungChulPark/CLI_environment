#!/usr/bin/env python3
"""eval_location_verified.py — 사람 위치검증(Y/N/U) 결과로 성능 재계산.

전제: 기존 TP 판정은 위치 무검증. 위치가 틀린 TP(=N)는 사실
  (a) 잘못된 검출 → FP  이고 동시에
  (b) 실제 객체는 못 찾음 → FN  이다.
따라서 클래스별 Y비율로 보정한다:
  TP_loc = TP * Yrate
  FP_loc = FP + TP * Nrate
  FN_loc = FN + TP * Nrate      (TP_loc + FN_loc = 원래 visible 총수 유지)

U(불확실)는 분모에서 제외하고 별도 보고한다(추정 불확실성).
rescue/rerank 로 새로 살린 셀은 전수 검증이므로 직접 집계한다.

산출: outputs/phase23_location_gt/metrics/location_verified_metrics.json
      outputs/phase23_location_gt/csv/location_verified_by_class.csv
"""
import csv, json, os, sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
OUT = os.path.join(REPO, "outputs", "phase23_location_gt")
TASK = os.path.join(OUT, "location_verification_task.csv")
IDX = os.path.join(REPO, "outputs", "phase21_fn_root_cause_audit", "csv", "gt_object_index.csv")


def main():
    if not os.path.isfile(TASK):
        raise SystemExit(f"[err] {TASK} 없음 — build_location_verification_task.py 먼저 실행")
    rows = list(csv.DictReader(open(TASK)))
    filled = [r for r in rows if r["box_on_target"].strip().upper() in ("Y", "N", "U")]
    if not filled:
        raise SystemExit("[err] box_on_target 이 아직 비어 있습니다. README.md 참고해 채워주세요.")
    print(f"검증 입력: {len(filled)}/{len(rows)} 행")

    # Phase 1C 원본 집계 (클래스별 TP/FP/FN)
    base = defaultdict(lambda: {"TP": 0, "FP": 0, "FN": 0})
    for r in csv.DictReader(open(IDX)):
        lab = r["phase1c_label"]
        if lab in ("TP", "FP", "FN"):
            base[r["object"]][lab] += 1

    # 그룹별 Y/N/U 집계
    vb = defaultdict(lambda: defaultdict(lambda: {"Y": 0, "N": 0, "U": 0}))
    for r in filled:
        vb[r["group"]][r["class_name"]][r["box_on_target"].strip().upper()] += 1

    # ---- phase1c_TP 보정 ----
    tot = {"TP": 0, "FP": 0, "FN": 0, "TP_loc": 0.0, "FP_loc": 0.0, "FN_loc": 0.0}
    per_rows = []
    unc = 0
    for o, b in sorted(base.items()):
        v = vb["phase1c_TP"].get(o, {"Y": 0, "N": 0, "U": 0})
        n_eff = v["Y"] + v["N"]; unc += v["U"]
        yrate = (v["Y"] / n_eff) if n_eff else None
        tp, fp, fn = b["TP"], b["FP"], b["FN"]
        if yrate is None:
            tp_l, fp_l, fn_l = tp, fp, fn      # 검증 없음 → 보정 불가(원값 유지, 표시)
        else:
            tp_l = tp * yrate
            wrong = tp * (1 - yrate)
            fp_l = fp + wrong
            fn_l = fn + wrong
        per_rows.append({"class": o, "TP": tp, "FP": fp, "FN": fn,
                         "verified_Y": v["Y"], "verified_N": v["N"], "verified_U": v["U"],
                         "Y_rate": round(yrate, 3) if yrate is not None else "",
                         "TP_loc": round(tp_l, 1), "FP_loc": round(fp_l, 1), "FN_loc": round(fn_l, 1)})
        for k, x in (("TP", tp), ("FP", fp), ("FN", fn), ("TP_loc", tp_l), ("FP_loc", fp_l), ("FN_loc", fn_l)):
            tot[k] += x

    def prf(tp, fp, fn):
        p = tp / max(tp + fp, 1e-9); r = tp / max(tp + fn, 1e-9)
        return round(p, 4), round(r, 4), round(2 * p * r / max(p + r, 1e-9), 4)

    p0 = prf(tot["TP"], tot["FP"], tot["FN"])
    pl = prf(tot["TP_loc"], tot["FP_loc"], tot["FN_loc"])

    # ---- rescue / rerank 전수 검증 ----
    grp = {}
    for g in ("rescue_recovered", "rerank_recovered"):
        y = sum(c["Y"] for c in vb[g].values()); n = sum(c["N"] for c in vb[g].values())
        u = sum(c["U"] for c in vb[g].values())
        grp[g] = {"Y": y, "N": n, "U": u, "claimed_TP_gain": y + n + u,
                  "real_TP_gain": y, "wrong_location": n,
                  "note": "Y만 실제 TP 증가; N은 오히려 FP"}

    os.makedirs(os.path.join(OUT, "csv"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "metrics"), exist_ok=True)
    with open(os.path.join(OUT, "csv", "location_verified_by_class.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(per_rows[0].keys())); w.writeheader(); w.writerows(per_rows)

    out = {"verified_rows": len(filled), "uncertain_U": unc,
           "phase1c_as_reported": {"TP": tot["TP"], "FP": tot["FP"], "FN": tot["FN"],
                                   "precision": p0[0], "recall": p0[1], "f1": p0[2]},
           "phase1c_location_verified": {"TP": round(tot["TP_loc"], 1), "FP": round(tot["FP_loc"], 1),
                                         "FN": round(tot["FN_loc"], 1),
                                         "precision": pl[0], "recall": pl[1], "f1": pl[2]},
           "per_class": per_rows, "recovery_groups": grp,
           "method": "클래스별 Y비율로 TP 보정; 위치오류 TP는 FP+FN 양쪽에 가산. U는 분모 제외.",
           "caveat": "클래스별 층화표본 외삽 — 표본이 적은 클래스는 신뢰구간 넓음."}
    json.dump(out, open(os.path.join(OUT, "metrics", "location_verified_metrics.json"), "w"),
              indent=2, ensure_ascii=False)

    print("\n=== Phase 1C: 보고값 vs 위치검증 보정값 ===")
    print(f"  보고값   TP {tot['TP']}  FP {tot['FP']}  FN {tot['FN']}  P {p0[0]} R {p0[1]} F1 {p0[2]}")
    print(f"  위치검증 TP {tot['TP_loc']:.0f}  FP {tot['FP_loc']:.0f}  FN {tot['FN_loc']:.0f}  P {pl[0]} R {pl[1]} F1 {pl[2]}")
    print(f"  (불확실 U {unc}건은 제외)")
    print("\n=== 클래스별 위치정확도(Y비율) ===")
    for r in per_rows:
        if r["Y_rate"] != "":
            print(f"  {r['class']:22s} Y{r['verified_Y']:3d}/N{r['verified_N']:3d}  Y비율 {r['Y_rate']}  TP {r['TP']}→{r['TP_loc']}")
    print("\n=== Phase 2.2 복구 주장 검증 ===")
    for g, v in grp.items():
        print(f"  {g:20s} 주장 +{v['claimed_TP_gain']}  실제 +{v['real_TP_gain']}  위치오류 {v['wrong_location']}")
    print(f"\n-> {OUT}/metrics/location_verified_metrics.json")


if __name__ == "__main__":
    main()
