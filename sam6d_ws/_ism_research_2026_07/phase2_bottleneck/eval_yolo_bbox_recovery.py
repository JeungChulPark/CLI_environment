#!/usr/bin/env python3
"""eval_yolo_bbox_recovery.py — Workstream C 오프라인 분석 (GPU 불필요).

collect_yolo_bbox_diagnostics.py 의 raw 덤프(conf 0.005) + 사람 GT visibility 로:
  - failure-stage 분류 F0~F6 (GT-visible 인데 운영 shared960@0.02 가 박스 미생성)
  - C1 conf sweep, C2 NMS(shared/agn/perprompt), C3 prompt(ext vs cur), C4 해상도(960/1280)
  - 방법별 visibility recall, FP/frame, boxes/frame

GT는 frame-level visibility 뿐 → per-box IoU recall 불가 → visibility recall 사용.
산출: csv/yolo_bbox_failure_cases.csv, yolo_threshold_sweep.csv, yolo_nms_sweep.csv,
      yolo_prompt_sweep.csv, yolo_resolution_sweep.csv, yolo_bbox_method_metrics.csv,
      metrics/yolo_bbox_metrics.json
"""
import csv, json, os, sys
from collections import defaultdict
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import phase2_common as pc

OUT = pc.OUT; RAW = os.path.join(OUT, "raw", "yolo_raw")
OUTC = os.path.join(OUT, "csv"); OUTM = os.path.join(OUT, "metrics")
DATASETS = pc.DATASETS; OBJECTS = pc.OBJECTS


def load_cfg(cfg):
    """cfg -> {(ds,frame,object): [conf,...]} (박스 conf 목록)."""
    d = defaultdict(list)
    for ds in DATASETS:
        p = os.path.join(RAW, cfg, f"{ds}.csv")
        if not os.path.isfile(p):
            continue
        for r in csv.DictReader(open(p)):
            if r["object"] == "":
                continue
            d[(ds, int(r["frame_id"]), r["object"])].append(float(r["conf"]))
    return d


def recall_fp(cfg_data, gt, conf):
    """visibility recall + FP/frame + boxes/frame at conf threshold."""
    frames = list(gt.keys())
    TPcell = 0; POS = 0; fp_boxes = 0; all_boxes = 0
    for (ds, fr), vis in gt.items():
        for o in OBJECTS:
            n = sum(1 for c in cfg_data.get((ds, fr, o), []) if c >= conf)
            all_boxes += n
            if o in vis:
                POS += 1
                if n >= 1: TPcell += 1
            else:
                fp_boxes += n
    return {"recall": round(TPcell / max(POS, 1), 4), "recovered_cells": TPcell, "pos": POS,
            "fp_per_frame": round(fp_boxes / max(len(frames), 1), 3),
            "boxes_per_frame": round(all_boxes / max(len(frames), 1), 3)}


def main():
    gt = pc.load_gt()
    nframe = len(gt)
    shared = load_cfg("shared960"); agn = load_cfg("shared960_agn")
    pp = load_cfg("perprompt960"); pp1280 = load_cfg("perprompt1280"); ext = load_cfg("ext960")

    # ---------- failure taxonomy ----------
    tax = defaultdict(int); cases = []
    for (ds, fr), vis in gt.items():
        for o in vis:
            shared_final = sum(1 for c in shared.get((ds, fr, o), []) if c >= 0.02)
            if shared_final >= 1:
                tax["hit_operational"] += 1; continue
            pp_raw = pp.get((ds, fr, o), []); sh_raw = shared.get((ds, fr, o), [])
            pp_002 = sum(1 for c in pp_raw if c >= 0.02)
            if len(pp_raw) == 0 and len(sh_raw) == 0:
                st, reason = "F0_no_raw_prediction", "prompt raw 0 (<0.005)"
            elif pp_002 == 0:
                st, reason = "F1_below_confidence", f"perprompt top {max(pp_raw+[0]):.3f} < 0.02"
            elif pp_002 >= 1:
                st, reason = "F2_shared_nms_suppression", f"perprompt {pp_002} box>=0.02 but shared 0"
            else:
                st, reason = "F6_other", ""
            tax[st] += 1
            cases.append({"dataset": ds, "frame_id": fr, "object": o, "failure_stage": st,
                          "reason": reason, "shared_raw": len(sh_raw),
                          "perprompt_raw": len(pp_raw), "perprompt_conf002": pp_002,
                          "perprompt_top_conf": round(max(pp_raw + [0.0]), 4)})

    # ---------- C1 conf sweep (shared960) ----------
    conf_grid = [0.005, 0.01, 0.02, 0.03, 0.05, 0.075, 0.10]
    c1 = {c: recall_fp(shared, gt, c) for c in conf_grid}

    # ---------- C2 NMS: shared(0.02) vs agn(0.02) vs perprompt(0.02) ----------
    c2 = {"shared960_default_nms": recall_fp(shared, gt, 0.02),
          "shared960_agnostic_nms": recall_fp(agn, gt, 0.02),
          "perprompt960_no_crossclass_nms": recall_fp(pp, gt, 0.02)}

    # ---------- C3 prompt: cur(perprompt) vs ext ----------
    c3 = {"cur_prompt_perprompt960": recall_fp(pp, gt, 0.02),
          "ext_prompt_ext960": recall_fp(ext, gt, 0.02)}

    # ---------- C4 resolution: perprompt 960 vs 1280 ----------
    c4 = {"perprompt960": recall_fp(pp, gt, 0.02),
          "perprompt1280": recall_fp(pp1280, gt, 0.02)}

    # ---------- method summary (visibility recall vs FP/frame) ----------
    methods = {"C0_operational_shared960@0.02": recall_fp(shared, gt, 0.02),
               "C1_shared960@0.01": recall_fp(shared, gt, 0.01),
               "C1_shared960@0.005": recall_fp(shared, gt, 0.005),
               "C2_shared960_agnostic@0.02": c2["shared960_agnostic_nms"],
               "C2_perprompt960@0.02": c2["perprompt960_no_crossclass_nms"],
               "C3_ext960@0.02": c3["ext_prompt_ext960"],
               "C4_perprompt1280@0.02": c4["perprompt1280"]}

    # ---------- write ----------
    os.makedirs(OUTC, exist_ok=True); os.makedirs(OUTM, exist_ok=True)
    with open(os.path.join(OUTC, "yolo_bbox_failure_cases.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(cases[0].keys()) if cases else
                           ["dataset", "frame_id", "object", "failure_stage", "reason",
                            "shared_raw", "perprompt_raw", "perprompt_conf002", "perprompt_top_conf"])
        w.writeheader(); w.writerows(cases)

    def wsweep(name, d, keycol):
        with open(os.path.join(OUTC, name), "w", newline="") as f:
            w = csv.writer(f); w.writerow([keycol, "recall", "recovered_cells", "pos",
                                           "fp_per_frame", "boxes_per_frame"])
            for k, v in d.items():
                w.writerow([k, v["recall"], v["recovered_cells"], v["pos"],
                            v["fp_per_frame"], v["boxes_per_frame"]])
    wsweep("yolo_threshold_sweep.csv", c1, "conf")
    wsweep("yolo_nms_sweep.csv", c2, "nms_config")
    wsweep("yolo_prompt_sweep.csv", c3, "prompt_config")
    wsweep("yolo_resolution_sweep.csv", c4, "resolution_config")
    wsweep("yolo_bbox_method_metrics.csv", methods, "method")

    out = {"gt_frames": nframe, "gt_positive_cells": sum(len(v) for v in gt.values()),
           "failure_taxonomy": dict(tax),
           "operational_visibility_recall": methods["C0_operational_shared960@0.02"],
           "conf_sweep": c1, "nms": c2, "prompt": c3, "resolution": c4, "methods": methods,
           "notes": {"gt_type": "frame-level visibility only → visibility recall (not IoU recall)",
                     "operational_detector": "shared960 single multi-label pass @ conf0.02, ultralytics NMS iou0.7 class-aware",
                     "F2_meaning": "perprompt yields >=0.02 box but shared pass NMS/class-competition drops it",
                     "note_1920_omitted": "perprompt1920 omitted for GPU cost; 1280 covers higher-res hypothesis"}}
    json.dump(out, open(os.path.join(OUTM, "yolo_bbox_metrics.json"), "w"), indent=2, ensure_ascii=False)

    print("=== Workstream C: failure taxonomy (GT-visible cells) ===")
    for k, v in sorted(tax.items(), key=lambda x: -x[1]):
        print(f"  {k:28s} {v}")
    print("=== visibility recall vs FP/frame ===")
    for k, v in methods.items():
        print(f"  {k:32s} recall {v['recall']}  FP/frame {v['fp_per_frame']}  box/frame {v['boxes_per_frame']}")
    print(f"-> {OUTM}/yolo_bbox_metrics.json")


if __name__ == "__main__":
    main()
