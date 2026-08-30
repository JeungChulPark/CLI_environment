#!/usr/bin/env python3
"""build_fn_object_index.py — 935 cell 인덱스 + FN 313 배타적 원인분류.

Phase 1C 정본(cur-hsv) 재현(622/86/313 assert) 후 FN 각각에 primary_root_cause.
산출: csv/gt_object_index.csv, fn_root_cause_per_object.csv, fn_root_cause_summary.csv,
      fn_root_cause_by_class.csv, fn_root_cause_by_bag.csv, metrics/root_cause_metrics.json
"""
import csv, json, os, sys
from collections import Counter, defaultdict
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import phase21_common as pc

OUTC = os.path.join(pc.OUT, "csv"); OUTM = os.path.join(pc.OUT, "metrics")


def main():
    gt = pc.load_gt(); PROTO = pc.build_protos(); cells = pc.build_cells(gt, PROTO)
    grid = pc.grid_from_cells(gt, cells)
    print("P0 재현:", grid)
    assert grid["TP"] == 622 and grid["FP"] == 86 and grid["FN"] == 313, \
        f"Phase 1C 재현 실패 {grid} — 원인분석 중단"
    print("  ✅ Phase 1C 정확 재현(622/86/313)")

    os.makedirs(OUTC, exist_ok=True); os.makedirs(OUTM, exist_ok=True)
    # 전체 cell 인덱스
    idx_rows = []; fn_rows = []
    cause = Counter(); byclass = defaultdict(Counter); bybag = defaultdict(Counter)
    for (ds, fr), vis in sorted(gt.items()):
        for o in pc.OBJECTS:
            c = cells.get((ds, fr, o)); v = o in vis; acc = bool(c and c["accept"])
            label = "TP" if (v and acc) else "FP" if acc else "FN" if v else "TN"
            b = c["best"] if c else None
            idx_rows.append({"dataset": ds, "frame_id": fr, "object": o, "gt_visible": int(v),
                             "phase1c_label": label, "has_candidate": int(bool(b)),
                             "best_uid": b["uid"] if b else "", "yolo_conf": round(b["conf"], 4) if b else "",
                             "sem_top5": round(b["sem"], 4) if b else "", "sim_thr": b["simthr"] if b else "",
                             "appe11": round(b["appe"], 4) if b else "", "appe_gate": b["appegate"] if b else "",
                             "hsv": round(b["hsv"], 4) if b else "", "hsv_thr": pc.T})
            if label == "FN":
                prim, sec = pc.fn_primary_cause(c)
                cause[prim] += 1; byclass[o][prim] += 1; bybag[ds][prim] += 1
                fn_rows.append({"dataset": ds, "frame_id": fr, "object": o,
                                "primary_root_cause": prim, "secondary_contributors": ";".join(sec),
                                "has_candidate": int(bool(b)),
                                "yolo_conf": round(b["conf"], 4) if b else "",
                                "sem_top5": round(b["sem"], 4) if b else "",
                                "passS": int(b["passS"]) if b else "",
                                "appe11": round(b["appe"], 4) if b else "", "passA": int(b["passA"]) if b else "",
                                "hsv": round(b["hsv"], 4) if b else "", "passH": int(b["passH"]) if b else "",
                                "n_routed_candidates": len(c["cands"]) if c else 0,
                                "any_cand_accept": int(any(x["accept"] for x in c["cands"])) if c else 0})

    total_fn = sum(cause.values())
    assert total_fn == 313, f"FN 합 {total_fn} != 313 (배타성/일관성 위반)"

    with open(os.path.join(OUTC, "gt_object_index.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(idx_rows[0].keys())); w.writeheader(); w.writerows(idx_rows)
    with open(os.path.join(OUTC, "fn_root_cause_per_object.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(fn_rows[0].keys())); w.writeheader(); w.writerows(fn_rows)

    # summary
    ORDER = ["RC1_no_candidate", "RC8_semantic", "RC9_appearance", "RC10_hsv",
             "RC11_selection", "RC12_unknown"]
    with open(os.path.join(OUTC, "fn_root_cause_summary.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["primary_root_cause", "count", "pct_of_313", "group"])
        GROUP = {"RC1_no_candidate": "proposal", "RC8_semantic": "gate", "RC9_appearance": "gate",
                 "RC10_hsv": "gate", "RC11_selection": "selection", "RC12_unknown": "unknown"}
        for k in ORDER:
            w.writerow([k, cause[k], round(100 * cause[k] / 313, 1), GROUP[k]])
    with open(os.path.join(OUTC, "fn_root_cause_by_class.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["class", "FN_total"] + ORDER)
        for o in pc.OBJECTS:
            w.writerow([o, sum(byclass[o].values())] + [byclass[o][k] for k in ORDER])
    with open(os.path.join(OUTC, "fn_root_cause_by_bag.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["bag", "FN_total"] + ORDER)
        for ds in pc.DATASETS:
            w.writerow([ds, sum(bybag[ds].values())] + [bybag[ds][k] for k in ORDER])

    grp = Counter()
    for k, v in cause.items():
        grp[{"RC1_no_candidate": "proposal", "RC8_semantic": "gate", "RC9_appearance": "gate",
             "RC10_hsv": "gate", "RC11_selection": "selection", "RC12_unknown": "unknown"}[k]] += v
    out = {"phase1c_repro": grid, "fn_total": total_fn, "primary_causes": dict(cause),
           "group_totals": dict(grp),
           "unknown_pct": round(100 * cause["RC12_unknown"] / 313, 2),
           "by_class": {o: dict(byclass[o]) for o in pc.OBJECTS},
           "by_bag": {ds: dict(bybag[ds]) for ds in pc.DATASETS},
           "notes": {"causes_are_upper_bound_gate": "RC8/9/10은 best-ROI oracle 재분류 전 상한(bad-box 포함 가능)",
                     "selection_RC11": "선택 best는 실패했으나 다른 routed 후보가 전 gate 통과한 cell"}}
    json.dump(out, open(os.path.join(OUTM, "root_cause_metrics.json"), "w"), indent=2, ensure_ascii=False)

    print(f"\nFN {total_fn} 원인(oracle 재분류 전):")
    for k in ORDER:
        print(f"  {k:20s} {cause[k]:3d}  ({100*cause[k]/313:.1f}%)")
    print(f"group: {dict(grp)}  | unknown {out['unknown_pct']}%")
    print(f"-> {OUTC}")


if __name__ == "__main__":
    main()
