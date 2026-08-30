#!/usr/bin/env python3
"""Analytic masked-appe gate sweep at conf0.02 + sim0.35.

The sim0.35 run was done with appe-gate=0 (gate off) so masked_appe_score is
recorded for EVERY sem>=0.35 candidate. We sweep the gate threshold here without
re-running, classifying with existing pseudo-GT labels.
"""
import csv
import glob
import json
import os
import collections

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUN = os.path.join(REPO, "outputs", "yolo_ism_masked_sim035")
BASE_VLM = os.path.join(REPO, "outputs", "yolo_ism_audit", "vlm_raw")
C02_CASES = os.path.join(REPO, "outputs", "yolo_ism_audit_conf02", "audit_cases_conf02.csv")
FLIP_VLM = os.path.join(REPO, "outputs", "yolo_ism_audit_sweep", "vlm_raw")

BAGS = ["high_texture_around", "high_texture_far_close", "two_table_around",
        "two_table_around_goback", "two_table_diagonal1", "two_table_diagonal2",
        "two_table_goback", "only_milk", "milk_nomilk_bag"]
MILK_BOX = {"correct_milk_box", "partial_milk_box"}
NONMILK_BOX = {"background_box", "other_object_box"}
GATES = [0.0, 0.55, 0.56, 0.57, 0.58, 0.59, 0.60, 0.61]


def classify(mp, dec_milk, box):
    if mp == "uncertain" or (dec_milk and box in ("uncertain", "", "na", None)):
        return "FP" if (dec_milk and mp == "no") else "UNC"
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

    # collect candidates
    frames = []  # (bag, fi, is_sem_candidate, masked_appe, mp, box)
    for bag in BAGS:
        for r in csv.DictReader(open(os.path.join(RUN, bag, "yolo_ism_results.csv"))):
            fi = int(r["frame_idx"])
            cand = r["decision"] == "milk"   # sem>=0.35 candidate (gate was off)
            ma = float(r["masked_appe_score"]) if r["masked_appe_score"] else 0.0
            mp, box = label(bag, fi)
            frames.append((bag, fi, cand, ma, mp, box))

    print("=== masked-appe gate sweep @ conf0.02 sim0.35 (pseudo-GT) ===")
    print(f"{'gate':>6} {'TP':>4} {'FP':>4} {'FN':>4} {'TN':>4} {'prec':>7} {'rec':>7} {'f1':>7}")
    out = {}
    for g in GATES:
        c = collections.Counter()
        for bag, fi, cand, ma, mp, box in frames:
            dec_milk = cand and ma >= g
            c[classify(mp, dec_milk, box)] += 1
        P = safe_div(c["TP"], c["TP"]+c["FP"]); R = safe_div(c["TP"], c["TP"]+c["FN"])
        F = safe_div(2*(P or 0)*(R or 0), (P or 0)+(R or 0)) if P and R else None
        out[f"{g:.2f}"] = {"TP": c["TP"], "FP": c["FP"], "FN": c["FN"], "TN": c["TN"],
                           "precision": P, "recall": R, "f1": F}
        tag = "(gate off)" if g == 0.0 else ""
        print(f"{g:6.2f} {c['TP']:4d} {c['FP']:4d} {c['FN']:4d} {c['TN']:4d} "
              f"{P:7} {R:7} {F:7} {tag}")
    json.dump(out, open(os.path.join(RUN, "gate_sweep.json"), "w"), indent=2)
    print("\nref: conf0.02 sim0.40 + gate0.59 = TP635 FP3 FN138 prec0.995 rec0.822 f1.900")
    print(f"saved: {os.path.join(RUN, 'gate_sweep.json')}")


if __name__ == "__main__":
    main()
