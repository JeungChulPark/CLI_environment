#!/usr/bin/env python3
"""Evaluate whether the geometric axis (IoU, visible_ratio, final_score) improves
TP/FP separation over masked-appe alone, on sem-passing candidates, using existing
pseudo-GT labels. final_score = (sem + appe + geo_iou*vis)/(2+vis) (SAM-6D form).
"""
import csv
import glob
import json
import os
import statistics as st
import collections

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GEO = os.path.join(REPO, "outputs", "yolo_ism_geo")
BASE_VLM = os.path.join(REPO, "outputs", "yolo_ism_audit", "vlm_raw")
C02_CASES = os.path.join(REPO, "outputs", "yolo_ism_audit_conf02", "audit_cases_conf02.csv")
FLIP_VLM = os.path.join(REPO, "outputs", "yolo_ism_audit_sweep", "vlm_raw")

BAGS = ["high_texture_around", "high_texture_far_close", "two_table_around",
        "two_table_around_goback", "two_table_diagonal1", "two_table_diagonal2",
        "two_table_goback", "only_milk", "milk_nomilk_bag"]
MILK_BOX = {"correct_milk_box", "partial_milk_box"}
NONMILK_BOX = {"background_box", "other_object_box"}


def labels():
    base = {}
    for bag in BAGS:
        for r in json.load(open(os.path.join(BASE_VLM, f"{bag}.json"))):
            base[(bag, int(r["frame_idx"]))] = r.get("milk_present", "uncertain")
    c02 = {}
    for r in csv.DictReader(open(C02_CASES)):
        c02[(r["bag"], int(r["frame_idx"]))] = (r["milk_present"], r["selected_box_correct"])
    flip = {}
    for f in glob.glob(os.path.join(FLIP_VLM, "*.json")):
        bag = os.path.basename(f).rsplit("__", 1)[0]
        for r in json.load(open(f)):
            flip[(bag, int(r["frame_idx"]))] = (
                r.get("milk_present", "uncertain"), r.get("selected_box_correct", "na"))

    def lab(bag, fi):
        if (bag, fi) in flip:
            return flip[(bag, fi)]
        if (bag, fi) in c02 and c02[(bag, fi)][1] not in ("", "na"):
            return c02[(bag, fi)]
        return base.get((bag, fi), "uncertain"), "na"
    return lab


def gt(mp, box):
    if mp == "uncertain" or box in ("uncertain", "", "na", None):
        return "FP" if mp == "no" else "UNC"
    if box in NONMILK_BOX or mp == "no":
        return "FP"
    return "TP" if box in MILK_BOX else "UNC"


def stats(v):
    v = sorted(v)
    return None if not v else {"n": len(v), "mean": round(st.mean(v), 3),
                               "med": round(v[len(v)//2], 3),
                               "min": round(v[0], 3), "max": round(v[-1], 3)}


def best_gate(tp, fp):
    """threshold on a score maximizing Youden (TPR - FPR)."""
    best = None
    for t in sorted(set([round(x, 3) for x in tp + fp])):
        tk = sum(1 for v in tp if v >= t); fk = sum(1 for v in fp if v >= t)
        tpr = tk/len(tp) if tp else 0; fpr = fk/len(fp) if fp else 0
        j = tpr - fpr
        if best is None or j > best["youden"]:
            best = {"thr": t, "youden": round(j, 3), "TP_kept": tk, "TP_tot": len(tp),
                    "FP_kept": fk, "FP_tot": len(fp)}
    return best


def main():
    lab = labels()
    rows = []
    for bag in BAGS:
        p = os.path.join(GEO, bag, "geometric.csv")
        if not os.path.isfile(p):
            continue
        for r in csv.DictReader(open(p)):
            mp, box = lab(bag, int(r["frame_idx"]))
            r["_gt"] = gt(mp, box); r["_bag"] = bag
            rows.append(r)
    groups = collections.Counter(r["_gt"] for r in rows)
    print(f"sem-pass candidates: {len(rows)}  ({dict(groups)})")
    print("\n=== TP vs FP score distributions ===")
    for tag in ["best_semantic_score", "masked_appe_score", "geometric_iou",
                "visible_ratio", "final_score"]:
        tp = [float(r[tag]) for r in rows if r["_gt"] == "TP"]
        fp = [float(r[tag]) for r in rows if r["_gt"] == "FP"]
        print(f"\n{tag}:")
        print(f"  TP {stats(tp)}")
        print(f"  FP {stats(fp)}")
        bg = best_gate(tp, fp)
        print(f"  best gate: thr={bg['thr']} youden={bg['youden']} "
              f"TP_kept={bg['TP_kept']}/{bg['TP_tot']} FP_kept={bg['FP_kept']}/{bg['FP_tot']}")
    json.dump({"n": len(rows), "groups": dict(groups)},
              open(os.path.join(GEO, "eval_geometric.json"), "w"), indent=2)


if __name__ == "__main__":
    main()
