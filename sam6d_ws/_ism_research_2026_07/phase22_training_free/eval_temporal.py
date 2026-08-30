#!/usr/bin/env python3
"""eval_temporal.py — 방법 H: online 인접-프레임 일관성 (미래 frame 미사용, 학습 0).

규칙(H1): 현재 셀이 gate borderline-fail 이지만, 같은 (bag,object)의 **직전** GT 프레임에서
해당 객체가 accept 였다면 제한적 rescue 검토. persistent ID/tracking 미구현.
주의: gt_input 프레임은 strided(비연속) → 시간 일관성 신호가 약함(보고에 명시).

산출: csv/temporal_consistency.csv, metrics/temporal_metrics.json
"""
import csv, json, os, sys
from collections import defaultdict
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rules as R
from eval_training_free_methods import load_pool, load_gt_and_rc, grid, OUT
import phase21_common as p21  # noqa (via path from eval import)


def main():
    cells = load_pool(); gt, rc = load_gt_and_rc()
    base, base_dec = grid(cells, gt, R.phase1c)

    # (bag,object) -> sorted 프레임의 Phase1C accept 여부
    seq = defaultdict(list)
    for (ds, fr), vis in gt.items():
        for o in cells_objects(cells, ds, fr):
            seq[(ds, o)].append((fr, base_dec[(ds, fr, o)][0]))
    for k in seq:
        seq[k].sort()
    prev_accept = {}
    for (ds, o), lst in seq.items():
        for i, (fr, acc) in enumerate(lst):
            prev_accept[(ds, fr, o)] = lst[i - 1][1] if i > 0 else None  # None=첫프레임(online fallback)

    def method_H1(cs, ds=None, fr=None, o=None):
        # single-frame Phase1C + 직전 accept시 borderline rescue
        acc, uid, r = R.phase1c(cs)
        if acc or not cs:
            return acc, uid, r
        s = R._sel(cs)
        pv = prev_accept.get((ds, fr, o))
        if pv and s and (not s["passS"]) and s["sem_top5"] >= 0.9 * s["sim_thr"] \
                and s["passA"] and s["passH"]:
            return True, s["uid"], "temporal_prev_accept_rescue"
        return False, None, "temporal_no"

    # grid with H1 (needs ds,fr,o in method -> wrap)
    TP = FP = FN = 0
    for (ds, fr), vis in gt.items():
        for o in p21.OBJECTS:
            cs = cells.get((ds, fr, o))
            acc, uid, r = method_H1(cs, ds, fr, o) if cs else (False, None, "no_candidate")
            v = o in vis
            if v and acc: TP += 1
            elif acc: FP += 1
            elif v: FN += 1
    res = {"H0_single_frame": {"TP": base["TP"], "FP": base["FP"], "FN": base["FN"], "f1": base["f1"]},
           "H1_prev_accept_rescue": {"TP": TP, "FP": FP, "FN": FN,
                                     "TP_recovered": TP - base["TP"], "FP_readmitted": FP - base["FP"]},
           "note": "gt_input 프레임 strided(비연속) → 시간 일관성 신호 약함. online(미래 frame 미사용)."}
    os.makedirs(os.path.join(OUT, "csv"), exist_ok=True)
    with open(os.path.join(OUT, "csv", "temporal_consistency.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["config", "TP", "FP", "FN", "TP_recovered", "FP_readmitted"])
        w.writerow(["H0_single_frame", base["TP"], base["FP"], base["FN"], 0, 0])
        w.writerow(["H1_prev_accept_rescue", TP, FP, FN, TP - base["TP"], FP - base["FP"]])
    json.dump(res, open(os.path.join(OUT, "metrics", "temporal_metrics.json"), "w"), indent=2, ensure_ascii=False)
    print("=== 방법 H 시간 일관성 (strided frames) ===")
    print(f"  H0 single-frame: TP{base['TP']} FP{base['FP']} FN{base['FN']}")
    print(f"  H1 prev-accept rescue: TP{TP} FP{FP} FN{FN} (ΔTP {TP-base['TP']:+d}, ΔFP {FP-base['FP']:+d})")
    print(f"-> {OUT}/metrics/temporal_metrics.json")


def cells_objects(cells, ds, fr):
    return [o for (d, f, o) in cells if d == ds and f == fr]


if __name__ == "__main__":
    main()
