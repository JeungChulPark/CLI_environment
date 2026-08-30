#!/usr/bin/env python3
"""audit_recall_definition.py — Phase 2 "recall 0.959" 정의 감사 (AC-01).

Phase 2의 visibility recall은 eval_yolo_bbox_recovery.recall_fp 에서
per (frame,object) 셀에 conf>=threshold 인 box가 '1개 이상' 있으면 hit → 즉
**BBox-existence(routed) recall, IoU 미사용, frame-object level**.
이를 코드 근거로 명시하고, BBox-existence recall(shared/perprompt960/1280)을 재보고.
IoU-based recall / usable-mask recall 은 GT bbox 부재로 not_available.
"""
import csv, json, os, sys
from collections import defaultdict
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import phase21_common as pc

RAW = os.path.join(pc.REPO, "outputs", "phase2_appearance_semantic_yolo_bbox", "raw", "yolo_raw")
OUTM = os.path.join(pc.OUT, "metrics"); OUTC = os.path.join(pc.OUT, "csv")


def existence_recall(cfg, gt, conf):
    d = defaultdict(int)
    for ds in pc.DATASETS:
        p = os.path.join(RAW, cfg, f"{ds}.csv")
        if not os.path.isfile(p):
            return None
        for r in csv.DictReader(open(p)):
            if r["object"] and float(r["conf"]) >= conf:
                d[(ds, int(r["frame_id"]), r["object"])] += 1
    POS = hit = 0
    for (ds, fr), vis in gt.items():
        for o in vis:
            POS += 1
            if d.get((ds, fr, o), 0) >= 1:
                hit += 1
    return {"recall": round(hit / max(POS, 1), 4), "hit": hit, "pos": POS}


def main():
    gt = pc.load_gt()
    os.makedirs(OUTM, exist_ok=True); os.makedirs(OUTC, exist_ok=True)
    modes = {c: existence_recall(c, gt, 0.02) for c in
             ("shared960", "shared960_agn", "perprompt960", "perprompt1280", "ext960")}

    audit = {
        "phase2_0959_definition": {
            "source_code": "_ism_research_2026_07/phase2_bottleneck/eval_yolo_bbox_recovery.py:recall_fp",
            "hit_condition": "per (frame,object) cell: count(box with conf>=0.02 routed/labeled to object) >= 1",
            "uses_IoU": False, "level": "frame-object (visibility) level, NOT per-box IoU",
            "denominator": "935 visible (frame×object) cells (GT visibility)",
            "conclusion": "0.959 = BBox-EXISTENCE recall (per-prompt960), not geometric/IoU recall",
        },
        "bbox_existence_recall_by_mode": modes,
        "iou_based_recall": {"Recall@IoU0.3": "not_available", "Recall@IoU0.5": "not_available",
                             "Recall@IoU0.7": "not_available", "GT_coverage0.7": "not_available",
                             "usable_mask_recall": "not_available",
                             "reason": "human GT bbox/mask 부재 (Option1: deferred, annotation CSV 발행)"},
        "three_concepts": {
            "1_bbox_existence": f"perprompt960 = {modes['perprompt960']['recall']} (측정됨)",
            "2_geometrically_valid_IoU": "not_available (GT bbox 필요)",
            "3_downstream_usable": "간접측정: best-ROI oracle(counterfactual_metrics.json)",
        },
    }
    json.dump(audit, open(os.path.join(OUTM, "proposal_metrics.json"), "w"), indent=2, ensure_ascii=False)

    with open(os.path.join(OUTC, "proposal_definition_recall.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["proposal_mode", "bbox_exists_recall", "recall_iou_030",
                                       "recall_iou_050", "recall_iou_070", "gt_coverage_070",
                                       "usable_mask_recall"])
        for c, m in modes.items():
            if m is None: continue
            w.writerow([c, m["recall"], "not_available", "not_available", "not_available",
                        "not_available", "not_available"])

    print("=== AC-01: Phase 2 '0.959' 정의 감사 ===")
    print("  hit 조건 = (frame,object) 셀에 conf>=0.02 box >=1개  →  BBox-EXISTENCE recall (IoU 미사용)")
    print("  BBox-existence recall by mode:")
    for c, m in modes.items():
        print(f"    {c:14s} {m['recall']}" if m else f"    {c}: n/a")
    print("  IoU recall(0.3/0.5/0.7)·usable-mask recall = not_available (GT bbox 부재, deferred)")
    print(f"-> {OUTM}/proposal_metrics.json")


if __name__ == "__main__":
    main()
