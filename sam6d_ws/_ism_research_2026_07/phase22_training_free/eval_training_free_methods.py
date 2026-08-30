#!/usr/bin/env python3
"""eval_training_free_methods.py — P0~P10 training-free 방법 평가 (학습·GT-fit 없음).

candidate_pool.csv 로 cell 단위 결정 → grid TP/FP/FN. GT는 평가에만.
LOOO(Leave-One-Object-Out) + 신규객체 시뮬(규칙 불변 적용) 로 일반화 확인.
산출: csv/method_comparison.csv, method_by_class.csv, method_by_bag.csv,
      recovered_fn_cases.csv, readmitted_fp_cases.csv, metrics/tf_metrics.json
"""
import csv, json, os, sys
from collections import defaultdict
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rules as R
P21 = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "phase21_fn_audit")
sys.path.insert(0, P21)
import phase21_common as p21   # noqa

OUT = os.path.join(p21.REPO, "outputs", "phase22_training_free_gate")
OBJECTS = p21.OBJECTS; DATASETS = p21.DATASETS
FLOATS = {"conf", "sem_top5", "appe11", "hsv", "sim_thr", "appe_gate", "hsv_thr", "sem_all_mean",
          "sem_top1", "sem_median", "appe2", "appe9", "sem_margin_vs_2nd", "appe_margin_vs_2nd",
          "view_median", "view_std", "view_top1_minus_median"}
INTS = {"gt_visible", "conf_rank", "n_cands", "is_selected", "passS", "passA", "passH", "accept",
        "view_pass_count", "frame_id"}


def load_pool():
    cells = defaultdict(list)
    for r in csv.DictReader(open(os.path.join(OUT, "csv", "candidate_pool.csv"))):
        d = dict(r)
        for k in FLOATS:
            d[k] = float(d[k])
        for k in INTS:
            d[k] = int(d[k])
        cells[(d["dataset"], d["frame_id"], d["object"])].append(d)
    return cells


def load_gt_and_rc():
    gt = p21.load_gt()
    rc = {}
    p = os.path.join(OUT, "..", "phase21_fn_root_cause_audit", "csv", "fn_root_cause_per_object.csv")
    if os.path.isfile(p):
        for r in csv.DictReader(open(p)):
            rc[(r["dataset"], int(r["frame_id"]), r["object"])] = r["primary_root_cause"]
    return gt, rc


def grid(cells, gt, method, objects=OBJECTS):
    TP = FP = FN = TN = 0; per = defaultdict(lambda: [0, 0, 0, 0]); perds = defaultdict(lambda: [0, 0, 0, 0])
    decisions = {}
    for (ds, fr), vis in gt.items():
        for o in objects:
            cs = cells.get((ds, fr, o))
            acc, uid, reason = method(cs) if cs else (False, None, "no_candidate")
            decisions[(ds, fr, o)] = (acc, uid, reason)
            v = o in vis; idx = 0 if (v and acc) else 1 if acc else 2 if v else 3
            if idx == 0: TP += 1
            elif idx == 1: FP += 1
            elif idx == 2: FN += 1
            else: TN += 1
            per[o][idx] += 1; perds[ds][idx] += 1
    P = TP / max(TP + FP, 1); Rr = TP / max(TP + FN, 1)
    return {"TP": TP, "FP": FP, "FN": FN, "TN": TN, "precision": round(P, 4), "recall": round(Rr, 4),
            "f1": round(2 * P * Rr / max(P + Rr, 1e-9), 4),
            "per_class": {k: list(v) for k, v in per.items()},
            "per_ds": {k: list(v) for k, v in perds.items()}}, decisions


def main():
    cells = load_pool(); gt, rc = load_gt_and_rc()
    base, base_dec = grid(cells, gt, R.phase1c)
    assert base["TP"] == 622 and base["FP"] == 86, f"P0 재현 실패 {base}"

    results = {}; all_dec = {}
    for name, fn in R.METHODS.items():
        m, dec = grid(cells, gt, fn); all_dec[name] = dec
        m["TP_recovered"] = m["TP"] - base["TP"]; m["FP_readmitted"] = m["FP"] - base["FP"]
        m["net_TP_gain"] = m["TP_recovered"]; results[name] = m

    # 복구/재유입 사례 + RC 분포
    rec_rows, fp_rows = [], []; rc_recovered = defaultdict(lambda: defaultdict(int))
    for name, dec in all_dec.items():
        for (ds, fr, o), (acc, uid, reason) in dec.items():
            b_acc = base_dec[(ds, fr, o)][0]; v = o in gt.get((ds, fr), set())
            if v and acc and not b_acc:      # recovered FN
                rc_recovered[name][rc.get((ds, fr, o), "?")] += 1
                rec_rows.append({"method": name, "dataset": ds, "frame_id": fr, "object": o,
                                 "root_cause": rc.get((ds, fr, o), "?"), "reason": reason, "uid": uid})
            if (not v) and acc and not b_acc:  # readmitted FP
                fp_rows.append({"method": name, "dataset": ds, "frame_id": fr, "object": o, "reason": reason})

    # ---- LOOO: 동일 고정규칙을 전 객체 적용, 각 held-out object 성능 집계 ----
    looo = {}
    for name, fn in R.METHODS.items():
        agg = defaultdict(lambda: [0, 0, 0])
        for test_o in OBJECTS:
            m, _ = grid(cells, gt, fn, objects=[test_o])
            agg[test_o] = [m["TP"], m["FP"], m["FN"]]
        looo[name] = agg

    os.makedirs(os.path.join(OUT, "csv"), exist_ok=True)
    with open(os.path.join(OUT, "csv", "method_comparison.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["method", "TP", "FP", "FN", "precision", "recall", "F1",
                                       "TP_recovered", "FP_readmitted"])
        for n, m in results.items():
            w.writerow([n, m["TP"], m["FP"], m["FN"], m["precision"], m["recall"], m["f1"],
                        m["TP_recovered"], m["FP_readmitted"]])
    with open(os.path.join(OUT, "csv", "method_by_class.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["method", "class", "TP", "FP", "FN"])
        for n, m in results.items():
            for o in OBJECTS:
                a = m["per_class"].get(o, [0, 0, 0, 0]); w.writerow([n, o, a[0], a[1], a[2]])
    with open(os.path.join(OUT, "csv", "method_by_bag.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["method", "bag", "TP", "FP", "FN"])
        for n, m in results.items():
            for ds in DATASETS:
                a = m["per_ds"].get(ds, [0, 0, 0, 0]); w.writerow([n, ds, a[0], a[1], a[2]])
    with open(os.path.join(OUT, "csv", "recovered_fn_cases.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["method", "dataset", "frame_id", "object", "root_cause", "reason", "uid"])
        w.writeheader(); w.writerows(rec_rows)
    with open(os.path.join(OUT, "csv", "readmitted_fp_cases.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["method", "dataset", "frame_id", "object", "reason"])
        w.writeheader(); w.writerows(fp_rows)

    out = {"baseline": {k: base[k] for k in ("TP", "FP", "FN", "f1")},
           "methods": {n: {k: m[k] for k in ("TP", "FP", "FN", "precision", "recall", "f1",
                                             "TP_recovered", "FP_readmitted")} for n, m in results.items()},
           "recovered_rc_distribution": {n: dict(v) for n, v in rc_recovered.items()},
           "adoption_note": "채택기준: FP<=86 유지하며 TP↑ 또는 F1↑ & 다수class/bag & 학습0"}
    json.dump(out, open(os.path.join(OUT, "metrics", "tf_metrics.json"), "w"), indent=2, ensure_ascii=False)

    print(f"P0 baseline: TP{base['TP']} FP{base['FP']} FN{base['FN']} F1{base['f1']}")
    print("=== training-free 방법 (FP<=86 유지가 관건) ===")
    for n, m in results.items():
        flag = "  ★FP<=86&TP↑" if (m["FP"] <= 86 and m["TP"] > base["TP"]) else \
               ("  (F1↑)" if m["f1"] > base["f1"] else "")
        print(f"  {n:26s} TP {m['TP']} FP {m['FP']} FN {m['FN']} F1 {m['f1']} "
              f"(ΔTP {m['TP_recovered']:+d}, ΔFP {m['FP_readmitted']:+d}){flag}")
    print("=== 복구 FN 의 root-cause 분포 ===")
    for n, v in rc_recovered.items():
        if v: print(f"  {n:26s} {dict(v)}")
    print(f"-> {OUT}/metrics/tf_metrics.json")


if __name__ == "__main__":
    main()
