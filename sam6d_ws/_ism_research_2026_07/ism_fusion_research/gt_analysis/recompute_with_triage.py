#!/usr/bin/env python3
"""recompute_with_triage.py — triage 결과로 recall 하한을 좁힌다 (READ-ONLY).

배경
  가시성 GT 는 "아주 조금만 보여도 visible" 로 관대하게 작성됐다(작성자 진술).
  그래서 처음 계산한 recall(64.3%)은 **하한**이었다.
  FN 334건을 다시 보고 "검출될 만큼 충분히 보였나" 를 판정했으므로 이제 좁힐 수 있다.

  D  잘 보임 — 검출됐어야 함   → 진짜 실패. 분모·분자에 그대로 둔다
  B  아주 일부만 보임          → 검출 난이도 문제. **분모에서 제외**한 값을 함께 보고한다
  N  사실 안 보임 (라벨 오류)  → 가시가 아니었으므로 분모에서 제외

  주의: triage 는 **FN 사례만** 했다. 이미 검출된 601건(TP)은 재검수하지 않았지만
        '검출됐다 = 검출 가능했다' 이므로 D 와 같은 취급으로 일관된다.

산출: results/gt_recall_corrected.csv
"""
import csv, os
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
GT = os.path.join(RSRCH, "gt_input")
RES = os.path.join(ROOT, "results")
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]

tri = {}
for r in csv.DictReader(open(os.path.join(GT, "triage_answers.csv"), encoding="utf-8")):
    tri[(r["dataset"], int(r["frame_id"]), r["object"])] = r

fo = list(csv.DictReader(open(os.path.join(RES, "gt_frame_object.csv"))))
print(f"frame×object {len(fo)} / triage 답변 {len(tri)}")

S = defaultdict(lambda: defaultdict(int))
for r in fo:
    o = r["object"]
    k = (r["dataset"], int(r["frame_id"]), o)
    vis = r["gt_visible"] == "1"
    acc = r["accepted"] == "1"
    hascand = int(r["n_candidate"]) > 0
    t = tri.get(k, {}).get("verdict", "")

    if vis and acc:
        S[o]["TP"] += 1
        S[o]["den_all"] += 1; S[o]["den_det"] += 1
        S[o]["cand_all"] += 1; S[o]["cand_det"] += 1
    elif vis and not acc:
        S[o]["den_all"] += 1
        if hascand:
            S[o]["cand_all"] += 1
        if t == "N":                      # 라벨 오류 — 가시가 아니었음
            S[o]["FN_labelerr"] += 1
        elif t == "B":                    # 아주 일부만 보임
            S[o]["FN_barely"] += 1
            if hascand:
                S[o]["cand_barely"] += 1
        else:                             # D 또는 미답변 → 진짜 실패로 센다
            S[o]["FN_real"] += 1
            S[o]["den_det"] += 1
            if hascand:
                S[o]["cand_det"] += 1
            else:
                S[o]["miss_det"] += 1
        if not hascand:
            S[o]["miss_all"] += 1
    elif acc:
        S[o]["FP"] += 1

rows = []
for o in OBJECTS:
    s = S[o]
    da, dd = s["den_all"], s["den_det"]
    rows.append({
        "object": o,
        "visible_all": da, "visible_detectable": dd,
        "excluded_barely": s["FN_barely"], "excluded_label_error": s["FN_labelerr"],
        "TP": s["TP"], "FP": s["FP"],
        "FN_real": s["FN_real"],
        "recall_all": round(s["TP"] / da, 4) if da else "",
        "recall_detectable": round(s["TP"] / dd, 4) if dd else "",
        "cand_recall_all": round(s["cand_all"] / da, 4) if da else "",
        "cand_recall_detectable": round(s["cand_det"] / dd, 4) if dd else "",
        "proposal_miss_all": s["miss_all"], "proposal_miss_detectable": s["miss_det"],
        "precision": (round(s["TP"] / (s["TP"] + s["FP"]), 4)
                      if (s["TP"] + s["FP"]) else "")})

with open(os.path.join(RES, "gt_recall_corrected.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

T = defaultdict(int)
for o in OBJECTS:
    for k, v in S[o].items():
        T[k] += v
print(f"\n{'객체':22s} {'가시(전체)':>10} {'가시(검출가능)':>13} {'제외B':>6} {'제외N':>6} "
      f"{'후보recall':>18} {'최종recall':>18}")
for r in rows:
    print(f"{r['object']:22s} {r['visible_all']:>10} {r['visible_detectable']:>13} "
          f"{r['excluded_barely']:>6} {r['excluded_label_error']:>6} "
          f"{str(r['cand_recall_all']):>8} → {str(r['cand_recall_detectable']):>7} "
          f"{str(r['recall_all']):>8} → {str(r['recall_detectable']):>7}")
da, dd = T["den_all"], T["den_det"]
print(f"{'합계':22s} {da:>10} {dd:>13} {T['FN_barely']:>6} {T['FN_labelerr']:>6} "
      f"{T['cand_all']/da:>8.4f} → {T['cand_det']/dd:>7.4f} "
      f"{T['TP']/da:>8.4f} → {T['TP']/dd:>7.4f}")
print(f"\nproposal miss : 전체 {T['miss_all']} → 검출 가능한 것만 {T['miss_det']}")
print(f"최종 FN       : 전체 {T['FN_real']+T['FN_barely']+T['FN_labelerr']} "
      f"→ 진짜 실패 {T['FN_real']} (난이도 {T['FN_barely']} / 라벨오류 {T['FN_labelerr']})")
print(f"-> {os.path.join(RES,'gt_recall_corrected.csv')}")
