#!/usr/bin/env python3
"""Evaluate the masked pipeline (conf0.02 + sim0.40 + MobileSAM masked-appe gate)
against existing pseudo-GT labels; compare to the no-gate sweep T0.40 baseline.
"""
import csv
import glob
import json
import os
import collections

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MASKED = os.path.join(REPO, "outputs", "yolo_ism_masked")
BASE_VLM = os.path.join(REPO, "outputs", "yolo_ism_audit", "vlm_raw")
C02_CASES = os.path.join(REPO, "outputs", "yolo_ism_audit_conf02", "audit_cases_conf02.csv")
FLIP_VLM = os.path.join(REPO, "outputs", "yolo_ism_audit_sweep", "vlm_raw")
OUT = os.path.join(REPO, "outputs", "yolo_ism_masked")

BAGS = ["high_texture_around", "high_texture_far_close", "two_table_around",
        "two_table_around_goback", "two_table_diagonal1", "two_table_diagonal2",
        "two_table_goback", "only_milk", "milk_nomilk_bag"]
MILK_BOX = {"correct_milk_box", "partial_milk_box"}
NONMILK_BOX = {"background_box", "other_object_box"}


def classify(mp, dec_milk, box):
    if mp == "uncertain" or (dec_milk and box in ("uncertain", "", "na", None)):
        return "UNC" if not (dec_milk and mp == "no") else "FP"
    if dec_milk:
        return "FP" if (box in NONMILK_BOX or mp == "no") else ("TP" if box in MILK_BOX else "UNC")
    return "FN" if mp == "yes" else "TN"


def safe_div(a, b):
    return round(a / b, 4) if b else None


def main():
    base_mp = {}
    for bag in BAGS:
        for r in json.load(open(os.path.join(BASE_VLM, f"{bag}.json"))):
            base_mp[(bag, int(r["frame_idx"]))] = r.get("milk_present", "uncertain")
    c02 = {}
    for r in csv.DictReader(open(C02_CASES)):
        c02[(r["bag"], int(r["frame_idx"]))] = (r["milk_present"], r["selected_box_correct"])
    flip = {}
    for f in glob.glob(os.path.join(FLIP_VLM, "*.json")):
        bag = os.path.basename(f).rsplit("__", 1)[0]
        for r in json.load(open(f)):
            flip[(bag, int(r["frame_idx"]))] = (
                r.get("milk_present", "uncertain"), r.get("selected_box_correct", "na"))

    def label(bag, fi):
        if (bag, fi) in flip:
            return flip[(bag, fi)]
        if (bag, fi) in c02 and c02[(bag, fi)][1] not in ("", "na"):
            return c02[(bag, fi)]
        return base_mp.get((bag, fi), "uncertain"), "na"

    cnt = collections.Counter()
    per_bag = collections.defaultdict(collections.Counter)
    for bag in BAGS:
        for r in csv.DictReader(open(os.path.join(MASKED, bag, "yolo_ism_results.csv"))):
            fi = int(r["frame_idx"]); dec_milk = r["decision"] == "milk"
            mp, box = label(bag, fi)
            cat = classify(mp, dec_milk, box)
            cnt[cat] += 1
            per_bag[bag][cat] += 1

    def metrics(c):
        P = safe_div(c["TP"], c["TP"] + c["FP"])
        R = safe_div(c["TP"], c["TP"] + c["FN"])
        F = safe_div(2*(P or 0)*(R or 0), (P or 0)+(R or 0)) if P and R else None
        return P, R, F
    P, R, F = metrics(cnt)
    summary = {"pipeline": "conf0.02 + sim0.40 + MobileSAM masked-appe gate 0.59",
               "TP": cnt["TP"], "FP": cnt["FP"], "FN": cnt["FN"], "TN": cnt["TN"],
               "uncertain": cnt["UNC"], "precision": P, "recall": R, "f1": F,
               "per_bag": {b: dict(per_bag[b]) for b in BAGS}}
    json.dump(summary, open(os.path.join(OUT, "eval_masked.json"), "w"), indent=2)

    print("=== pipeline comparison (pseudo-GT) ===")
    print(f"{'pipeline':38s} {'TP':>4} {'FP':>4} {'FN':>4} {'TN':>4} {'prec':>7} {'rec':>7} {'f1':>7}")
    refs = [
        ("conf0.05 sim0.50 (baseline)", 482, 0, 294, 446),
        ("conf0.02 sim0.50", 594, 3, 179, 438),
        ("conf0.02 sim0.40 (no appe gate)", 669, 21, 101, 423),
        ("conf0.02 sim0.35 (no appe gate)", 684, 29, 86, 415),
    ]
    for nm, tp, fp, fn, tn in refs:
        p = safe_div(tp, tp+fp); r = safe_div(tp, tp+fn)
        f = safe_div(2*(p or 0)*(r or 0), (p or 0)+(r or 0)) if p and r else None
        print(f"{nm:38s} {tp:4d} {fp:4d} {fn:4d} {tn:4d} {p:7} {r:7} {f:7}")
    print(f"{'conf0.02 sim0.40 + MASK appe-gate':38s} {cnt['TP']:4d} {cnt['FP']:4d} "
          f"{cnt['FN']:4d} {cnt['TN']:4d} {P:7} {R:7} {F:7}")
    print(f"\nuncertain excluded: {cnt['UNC']}")
    print(f"per-bag FP: " + ", ".join(f"{b.split('_')[0][:6]}:{per_bag[b]['FP']}" for b in BAGS))


if __name__ == "__main__":
    main()
