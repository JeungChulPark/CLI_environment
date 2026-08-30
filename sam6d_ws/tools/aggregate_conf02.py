#!/usr/bin/env python3
"""Aggregate conf=0.02 metrics by REUSING baseline frame-intrinsic VLM labels
(milk_present/view_state) + fresh VLM box judgments for the 117 new/changed
milk decisions. Compare against the conf=0.05 baseline.
"""
import csv
import json
import os
import collections

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
C02 = os.path.join(REPO, "outputs", "yolo_ism_conf02")
BASE_VLM = os.path.join(REPO, "outputs", "yolo_ism_audit", "vlm_raw")
C02_VLM = os.path.join(REPO, "outputs", "yolo_ism_audit_conf02", "vlm_raw")
BASE_JSON = os.path.join(REPO, "outputs", "yolo_ism_audit", "metrics_summary.json")
OUT = os.path.join(REPO, "outputs", "yolo_ism_audit_conf02")

BAGS = ["high_texture_around", "high_texture_far_close", "two_table_around",
        "two_table_around_goback", "two_table_diagonal1", "two_table_diagonal2",
        "two_table_goback", "only_milk", "milk_nomilk_bag"]
MILK_BOX = {"correct_milk_box", "partial_milk_box"}
NONMILK_BOX = {"background_box", "other_object_box"}


def classify(mp, dec_milk, box):
    if mp == "uncertain":
        return "UNCERTAIN"
    if dec_milk and box == "uncertain":
        return "UNCERTAIN"
    if dec_milk:
        if box in NONMILK_BOX:
            return "FP"
        if mp == "no":
            return "FP"
        if box in MILK_BOX or box == "na":
            return "TP"
        return "UNCERTAIN"
    return "FN" if mp == "yes" else "TN"


def safe_div(a, b):
    return round(a / b, 4) if b else None


def metrics(counts):
    P = safe_div(counts["TP"], counts["TP"] + counts["FP"])
    R = safe_div(counts["TP"], counts["TP"] + counts["FN"])
    F = safe_div(2*(P or 0)*(R or 0), (P or 0)+(R or 0)) if P and R else None
    ev = counts["TP"]+counts["FP"]+counts["FN"]+counts["TN"]
    return P, R, F, safe_div(counts["TP"]+counts["TN"], ev)


def main():
    # frame-intrinsic baseline labels
    base_lab = {}
    for bag in BAGS:
        for r in json.load(open(os.path.join(BASE_VLM, f"{bag}.json"))):
            base_lab[(bag, int(r["frame_idx"]))] = r
    # fresh conf02 box judgments (117)
    c02_lab = {}
    import glob
    for f in glob.glob(os.path.join(C02_VLM, "*.json")):
        bag = os.path.basename(f).rsplit("__", 1)[0]
        for r in json.load(open(f)):
            c02_lab[(bag, int(r["frame_idx"]))] = r

    rows = []
    per_bag = {}
    for bag in BAGS:
        cnt = collections.Counter()
        for r in csv.DictReader(open(os.path.join(C02, bag, "yolo_ism_results.csv"))):
            fi = int(r["frame_idx"]); key = (bag, fi)
            dec_milk = r["decision"] == "milk"
            fresh = c02_lab.get(key)
            base = base_lab.get(key, {})
            if fresh is not None:
                mp = fresh.get("milk_present", "uncertain")
                box = fresh.get("selected_box_correct", "na")
                src = "conf02_vlm"
            else:
                mp = base.get("milk_present", "uncertain")
                box = base.get("selected_box_correct", "na") if dec_milk else "na"
                src = "reuse"
            cat = classify(mp, dec_milk, box)
            cnt[cat] += 1
            rows.append([bag, fi, r["timestamp"], mp, r["decision"], box, cat,
                         r["best_semantic_score"], src])
        P, R, F, A = metrics(cnt)
        per_bag[bag] = {"frames": sum(cnt.values()),
                        **{k: cnt[k] for k in ["TP", "FP", "FN", "TN", "UNCERTAIN"]},
                        "precision": P, "recall": R, "f1": F, "accuracy": A}

    g = collections.Counter()
    for b in per_bag:
        for k in ["TP", "FP", "FN", "TN", "UNCERTAIN"]:
            g[k] += per_bag[b][k]
    P, R, F, A = metrics(g)
    summary = {
        "pseudo_gt_disclaimer": "pseudo-GT based (VLM-assisted), NOT official GT. "
        "milk_present/view reused from conf05 baseline (frame-intrinsic); box "
        "correctness freshly judged for the 117 new/changed milk decisions.",
        "conf": 0.02, "total_frames": sum(g.values()),
        "TP": g["TP"], "FP": g["FP"], "FN": g["FN"], "TN": g["TN"],
        "uncertain_frames": g["UNCERTAIN"],
        "precision": P, "recall": R, "f1": F, "accuracy": A,
        "per_bag_metrics": per_bag,
    }
    json.dump(summary, open(os.path.join(OUT, "metrics_summary_conf02.json"), "w"),
              indent=2, ensure_ascii=False)
    with open(os.path.join(OUT, "audit_cases_conf02.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["bag", "frame_idx", "name", "milk_present", "decision",
                    "selected_box_correct", "category", "best_semantic_score", "label_source"])
        w.writerows(rows)

    # ---- compare ----
    base = json.load(open(BASE_JSON))
    print("================  conf 0.05 (baseline)  vs  conf 0.02  ================")
    def line(name, a, b):
        print(f"  {name:14s} {a:>8} -> {b:>8}")
    line("TP", base["TP"], g["TP"])
    line("FP", base["FP"], g["FP"])
    line("FN", base["FN"], g["FN"])
    line("TN", base["TN"], g["TN"])
    line("precision", base["precision"], P)
    line("recall", base["recall"], R)
    line("f1", base["f1"], F)
    line("accuracy", base["accuracy"], A)
    line("uncertain", base["uncertain_frames"], g["UNCERTAIN"])
    print("\nper-bag recall / FP:")
    for b in BAGS:
        bb = base["per_bag_metrics"][b]; cc = per_bag[b]
        print(f"  {b:24s} R {bb['recall']} -> {cc['recall']:<7} "
              f"FP {bb['FP']} -> {cc['FP']}  TP {bb['TP']}->{cc['TP']}")
    print(f"\nNEW false positives at conf02: {g['FP']} (baseline {base['FP']})")


if __name__ == "__main__":
    main()
