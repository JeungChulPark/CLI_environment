#!/usr/bin/env python3
"""emit_annotation_targets.py — GT bbox 부재 지표(IoU/CF-1/2/3)용 사람 annotation 타깃 발행.

FN 313 전수 + TP/FP control sample을 대상으로 (bag,frame,object,routed_bbox,image_path,
gt_bbox=빈칸) CSV를 만든다. 자동 GT 생성 금지 — 빈칸으로 두고 사람이 채운다.
산출: csv/yolo_fn_root_cause_human_bbox_gt.csv
"""
import csv, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import phase21_common as pc

OUTC = os.path.join(pc.OUT, "csv")
FRAMES = pc.FRAMES


def main():
    gt = pc.load_gt(); PROTO = pc.build_protos(); cells = pc.build_cells(gt, pc.build_protos())
    rows = []
    tp_ctrl = defaultdict(int); fp_ctrl = 0
    from collections import Counter
    tp_ctrl = Counter()
    for (ds, fr), vis in sorted(gt.items()):
        for o in pc.OBJECTS:
            c = cells.get((ds, fr, o)); v = o in vis; acc = bool(c and c["accept"])
            label = "TP" if (v and acc) else "FP" if acc else "FN" if v else "TN"
            b = c["best"] if c else None
            img = os.path.join("_ism_research_2026_07/gt_input/frames", ds, f"frame_{fr:06d}.png")
            routed = ""
            if b:
                routed = b["uid"].rsplit("|", 1)[1]
            take = False; role = ""
            if label == "FN":
                take = True; role = "FN(전수)"
            elif label == "TP" and tp_ctrl[o] < 3:
                take = True; role = "TP_control"; tp_ctrl[o] += 1
            elif label == "FP" and fp_ctrl < 30:
                take = True; role = "FP_control"; fp_ctrl += 1
            if take:
                rows.append({"role": role, "bag_name": ds, "frame_id": fr, "object_class": o,
                             "image_path": img, "routed_bbox_x1y1x2y2": routed,
                             "gt_bbox_x1y1x2y2": "", "gt_mask_available": "no", "notes": ""})
    os.makedirs(OUTC, exist_ok=True)
    with open(os.path.join(OUTC, "yolo_fn_root_cause_human_bbox_gt.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    n_fn = sum(1 for r in rows if r["role"].startswith("FN"))
    print(f"annotation targets: {len(rows)} rows (FN {n_fn} 전수 + TP/FP control). gt_bbox=빈칸(사람 입력 대기)")
    print(f"-> {OUTC}/yolo_fn_root_cause_human_bbox_gt.csv")


if __name__ == "__main__":
    from collections import defaultdict
    main()
