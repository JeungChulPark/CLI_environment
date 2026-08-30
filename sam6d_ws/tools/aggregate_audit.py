#!/usr/bin/env python3
"""Aggregate pseudo-GT VLM labels + yolo_ism outputs into audit metrics.

Inputs per bag (outputs/yolo_ism/<bag>/):
  - yolo_ism_results.csv      system decision + scores + selected box
  - proposals.csv             all YOLO proposals (from build_audit_overlays.py)
  - audit_overlay/<name>.jpg   composite overlay (for visual_audit copies)
VLM labels:
  - outputs/yolo_ism_audit/vlm_raw/<bag>.json   list of per-frame label dicts

Outputs (outputs/yolo_ism_audit/):
  audit_cases.csv, proposal_audit.csv, metrics_summary.json,
  missing_files.csv, human_review_needed.csv,
  visual_audit/{TP,FP,FN,TN,uncertain}/<bag>__<name>.jpg

NOTE: all precision/recall/F1 here are pseudo-GT based (VLM-assisted), NOT
official ground truth.
"""
import csv
import json
import os
import shutil

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ISM_ROOT = os.path.join(REPO, "outputs", "yolo_ism")
AUD = os.path.join(REPO, "outputs", "yolo_ism_audit")
VLM_RAW = os.path.join(AUD, "vlm_raw")

BAGS = ["high_texture_around", "high_texture_far_close", "two_table_around",
        "two_table_around_goback", "two_table_diagonal1", "two_table_diagonal2",
        "two_table_goback", "only_milk", "milk_nomilk_bag"]

MILK_BOX = {"correct_milk_box", "partial_milk_box"}
NONMILK_BOX = {"background_box", "other_object_box"}


def read_csv_dicts(path):
    if not os.path.isfile(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def classify(milk_present, decision_milk, box_label, milk_in_prop):
    """Return (category, note). Categories: TP/FP/FN/TN/UNCERTAIN."""
    if milk_present == "uncertain":
        return "UNCERTAIN", "milk presence uncertain"
    if decision_milk and box_label == "uncertain":
        return "UNCERTAIN", "selected box correctness uncertain"

    if decision_milk:
        if box_label in NONMILK_BOX:
            return "FP", f"decided milk but box={box_label}"
        if milk_present == "no":
            return "FP", "decided milk but no milk in frame"
        # milk present (yes) and box contains milk
        if box_label in MILK_BOX or box_label == "na":
            return "TP", "milk present + decided milk + box on milk"
        return "UNCERTAIN", f"ambiguous box={box_label}"
    else:
        # decision = no-object
        if milk_present == "yes":
            return "FN", "milk present but decided no-object"
        return "TN", "no milk + decided no-object"


def safe_div(a, b):
    return round(a / b, 4) if b else None


def main():
    os.makedirs(AUD, exist_ok=True)
    case_rows, prop_rows, missing, human = [], [], [], []
    per_bag = {}

    for bag in BAGS:
        out_dir = os.path.join(ISM_ROOT, bag)
        results = {r["frame_idx"]: r for r in
                   read_csv_dicts(os.path.join(out_dir, "yolo_ism_results.csv"))}
        props_all = read_csv_dicts(os.path.join(out_dir, "proposals.csv"))
        props_by_frame = {}
        for p in props_all:
            props_by_frame.setdefault(p["frame_idx"], []).append(p)

        vlm_path = os.path.join(VLM_RAW, f"{bag}.json")
        vlm = {}
        if os.path.isfile(vlm_path):
            for rec in json.load(open(vlm_path)):
                vlm[str(rec.get("frame_idx"))] = rec
        else:
            missing.append([bag, "ALL", "vlm_raw json missing", vlm_path])

        counts = {k: 0 for k in ["TP", "FP", "FN", "TN", "UNCERTAIN"]}
        prop_recall_num = prop_recall_den = 0
        good_prop_milkframes = good_prop_decmilk = 0
        cls_tp = cls_fp = cls_fn = 0
        side_total = side_low = 0

        for fidx, r in results.items():
            rec = vlm.get(fidx)
            overlay = os.path.join(out_dir, "audit_overlay",
                                   f"{r.get('timestamp', fidx)}.jpg")
            if not os.path.isfile(overlay):
                missing.append([bag, fidx, "audit_overlay jpg missing", overlay])
            if rec is None:
                missing.append([bag, fidx, "no VLM label", vlm_path])
                human.append([bag, fidx, "missing VLM label", "", ""])
                continue

            milk_present = rec.get("milk_present", "uncertain")
            view = rec.get("view_state", "")
            box_label = rec.get("selected_box_correct", "na")
            box_quality = rec.get("selected_box_quality", "na")
            milk_in_prop = rec.get("milk_in_any_proposal", "uncertain")
            decision = r.get("decision", "")
            decision_milk = decision == "milk"

            cat, note = classify(milk_present, decision_milk, box_label, milk_in_prop)
            counts[cat] += 1

            # proposal recall (only frames where milk truly present)
            if milk_present == "yes":
                prop_recall_den += 1
                if milk_in_prop == "yes":
                    prop_recall_num += 1
            # side/front-back view similarity tendency
            if view in ("side_view", "front_back_view"):
                side_total += 1
                try:
                    if float(r.get("best_semantic_score", 0) or 0) < 0.5:
                        side_low += 1
                except ValueError:
                    pass
            # cls-stage metrics GIVEN a good proposal exists
            good_prop = milk_in_prop == "yes"
            if good_prop and milk_present == "yes":
                good_prop_milkframes += 1
                if decision_milk and box_label in MILK_BOX:
                    cls_tp += 1
                else:
                    cls_fn += 1
            if good_prop and decision_milk:
                good_prop_decmilk += 1
                if box_label in NONMILK_BOX or milk_present == "no":
                    cls_fp += 1

            case_rows.append([
                bag, fidx, r.get("timestamp", ""), milk_present, view,
                milk_in_prop, decision, r.get("best_yolo_score", ""),
                r.get("best_semantic_score", ""), r.get("best_appe_score", ""),
                box_label, box_quality, cat,
                rec.get("fn_reason", ""), rec.get("fp_reason", ""),
                note, rec.get("note", ""), overlay,
            ])

            # proposal-level rows
            for p in props_by_frame.get(fidx, []):
                prop_rows.append([
                    bag, fidx, r.get("timestamp", ""), p["prop_rank"],
                    p["x1"], p["y1"], p["x2"], p["y2"], p["yolo_score"],
                    p["is_selected"],
                    box_label if p["is_selected"] == "1" else "",
                    box_quality if p["is_selected"] == "1" else "",
                ])

            if cat == "UNCERTAIN" or milk_present == "uncertain":
                human.append([bag, fidx, note, milk_present, box_label])

        ev = counts["TP"] + counts["FP"] + counts["FN"] + counts["TN"]
        prec = safe_div(counts["TP"], counts["TP"] + counts["FP"])
        rec_ = safe_div(counts["TP"], counts["TP"] + counts["FN"])
        f1 = safe_div(2 * (prec or 0) * (rec_ or 0), (prec or 0) + (rec_ or 0)) \
            if prec and rec_ else None
        acc = safe_div(counts["TP"] + counts["TN"], ev)
        per_bag[bag] = {
            "frames": len(results), "evaluated": ev,
            "uncertain": counts["UNCERTAIN"],
            **{k: counts[k] for k in ["TP", "FP", "FN", "TN"]},
            "precision": prec, "recall": rec_, "f1": f1, "accuracy": acc,
            "proposal_recall": safe_div(prop_recall_num, prop_recall_den),
            "cls_precision_given_good_proposal": safe_div(cls_tp, cls_tp + cls_fp),
            "cls_recall_given_good_proposal": safe_div(cls_tp, cls_tp + cls_fn),
            "side_view_frames": side_total, "side_view_low_sem": side_low,
        }

    # ---- global ----
    g = {k: sum(per_bag[b][k] for b in per_bag) for k in ["TP", "FP", "FN", "TN"]}
    tot_frames = sum(per_bag[b]["frames"] for b in per_bag)
    tot_unc = sum(per_bag[b]["uncertain"] for b in per_bag)
    ev = g["TP"] + g["FP"] + g["FN"] + g["TN"]
    prec = safe_div(g["TP"], g["TP"] + g["FP"])
    rec_ = safe_div(g["TP"], g["TP"] + g["FN"])
    f1 = safe_div(2 * (prec or 0) * (rec_ or 0), (prec or 0) + (rec_ or 0)) \
        if prec and rec_ else None
    pr_num = sum(per_bag[b]["proposal_recall"] * 0 for b in per_bag)  # placeholder
    summary = {
        "pseudo_gt_disclaimer": "All precision/recall/F1/accuracy are pseudo-GT "
        "based (VLM-assisted), NOT official ground truth.",
        "total_frames": tot_frames, "evaluated_frames": ev,
        "uncertain_frames": tot_unc,
        "TP": g["TP"], "FP": g["FP"], "FN": g["FN"], "TN": g["TN"],
        "precision": prec, "recall": rec_, "f1": f1,
        "accuracy": safe_div(g["TP"] + g["TN"], ev),
        "per_bag_metrics": per_bag,
    }

    # ---- write tables ----
    with open(os.path.join(AUD, "audit_cases.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["bag", "frame_idx", "name", "milk_present", "view_state",
                    "milk_in_any_proposal", "decision", "best_yolo_score",
                    "best_semantic_score", "best_appe_score",
                    "selected_box_correct", "selected_box_quality", "category",
                    "fn_reason", "fp_reason", "class_note", "vlm_note", "overlay"])
        w.writerows(case_rows)

    with open(os.path.join(AUD, "proposal_audit.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["bag", "frame_idx", "name", "prop_rank", "x1", "y1", "x2",
                    "y2", "yolo_score", "is_selected", "selected_box_correct",
                    "selected_box_quality"])
        w.writerows(prop_rows)

    with open(os.path.join(AUD, "missing_files.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["bag", "frame_idx", "issue", "path"])
        w.writerows(missing)

    with open(os.path.join(AUD, "human_review_needed.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["bag", "frame_idx", "reason", "milk_present", "box_label"])
        w.writerows(human)

    json.dump(summary, open(os.path.join(AUD, "metrics_summary.json"), "w"),
              indent=2, ensure_ascii=False)

    # ---- visual audit copies ----
    vis = os.path.join(AUD, "visual_audit")
    for cat in ["TP", "FP", "FN", "TN", "uncertain"]:
        d = os.path.join(vis, cat)
        if os.path.isdir(d):
            shutil.rmtree(d)
        os.makedirs(d, exist_ok=True)
    for row in case_rows:
        bag, fidx, name, cat, overlay = row[0], row[1], row[2], row[12], row[17]
        dst_cat = "uncertain" if cat == "UNCERTAIN" else cat
        if os.path.isfile(overlay):
            shutil.copy(overlay, os.path.join(vis, dst_cat, f"{bag}__{name}.jpg"))

    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
