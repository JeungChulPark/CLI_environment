#!/usr/bin/env python3
"""build_deploy_viz.py — 승격된 배포 설정으로 ros2 bag 을 돌린 결과를 프레임별로 렌더 (READ-ONLY).

입력: outputs/phase35_deploy_bag/<bag>/<object>_results.csv  (운영 진입점이 직접 쓴 것)
      + 같은 bag 을 stride 로 다시 읽은 원본 프레임
패널: [원본 | 인식 결과]
색  : GT 가 있는 프레임 -> 초록=GT에 있는 객체 / 빨강=GT에 없는 객체
      GT 가 없는 프레임 -> 파랑(판정 보류: 정답 라벨이 없는 프레임)
산출: outputs/phase35_deploy_bag_viz/{viz,csv}/
"""
import csv, os, sys
from collections import defaultdict
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, os.path.join(RSRCH, "phase21_fn_audit")); sys.path.insert(0, REPO)
import phase21_common as p21
import yolo_ism_object_n as o_n
import yolo_ism as yi

RUN = os.path.join(REPO, "outputs", "phase35_deploy_bag")
OUT = os.path.join(REPO, "outputs", "phase35_deploy_bag_viz")
FPAUDIT = os.path.join(REPO, "outputs", "phase32_fp_audit", "fp_audit_task.csv")
BAGS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]


def corrected_gt():
    gt = {k: set(v) for k, v in p21.load_gt().items()}
    if os.path.isfile(FPAUDIT):
        for r in csv.DictReader(open(FPAUDIT)):
            if r["answer"].strip().upper() == "T":
                gt[(r["bag_name"], int(r["frame_id"]))].add(r["class_name"])
    return gt


def main():
    for d in ("viz", "csv"):
        os.makedirs(os.path.join(OUT, d), exist_ok=True)
    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    stride = int(defaults.get("stride", 10))
    gt = corrected_gt(); man = []
    for bag in BAGS:
        run = os.path.join(RUN, bag)
        det = defaultdict(dict)
        for o in objs:
            f = os.path.join(run, f"{o['name']}_results.csv")
            if not os.path.isfile(f): continue
            for r in csv.DictReader(open(f)):
                if not r["selected_bbox_xyxy"] or not r["decision"].startswith("detected"):
                    continue
                det[int(r["frame_idx"])][o["name"]] = [int(v) for v in r["selected_bbox_xyxy"].split(";")]
        n = 0
        for frame_idx, name, bgr in yi.resolve_frames(
                os.path.join(REPO, "data", "ros2_bag", bag), "", stride, 0,
                "/camera/camera/color/image_raw"):
            acc = det.get(frame_idx, {})
            has_gt = (bag, frame_idx) in gt
            vis = gt.get((bag, frame_idx), set())
            img = bgr.copy(); drawn = []
            for ob, (x1, y1, x2, y2) in sorted(acc.items(),
                                               key=lambda kv: (kv[0] not in vis, kv[0])):
                if not has_gt: col = (235, 170, 60)
                else: col = (90, 220, 90) if ob in vis else (60, 60, 235)
                ins = sum(5 for b in drawn if max(abs(b[0]-x1), abs(b[1]-y1),
                                                  abs(b[2]-x2), abs(b[3]-y2)) <= 4)
                drawn.append((x1, y1, x2, y2))
                X1, Y1, X2, Y2 = x1+ins, y1+ins, x2-ins, y2-ins
                cv2.rectangle(img, (X1, Y1), (X2, Y2), col, 3)
                t = ob.replace("_high", "")
                ty = max(0, Y1-19)
                cv2.rectangle(img, (X1, ty), (X1+8*len(t), ty+19), col, -1)
                cv2.putText(img, t, (X1+2, ty+14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1)
            fn = f"{bag}_{frame_idx:06d}.png"
            cv2.imwrite(os.path.join(OUT, "viz", fn), np.hstack([bgr, img]))
            man.append({"name": f"{bag}_{frame_idx:06d}", "bag": bag, "frame": frame_idx,
                        "has_gt": int(has_gt), "gt_visible": ";".join(sorted(vis)),
                        "detected": ";".join(sorted(acc)),
                        "tp": ";".join(sorted(set(acc) & vis)) if has_gt else "",
                        "fp": ";".join(sorted(set(acc) - vis)) if has_gt else "",
                        "miss": ";".join(sorted(vis - set(acc))) if has_gt else "",
                        "n_det": len(acc), "image": fn})
            n += 1
        print(f"  {bag:12s} {n} 프레임")
    with open(os.path.join(OUT, "csv", "deploy_manifest.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(man[0].keys())); w.writeheader(); w.writerows(man)
    g = [m for m in man if m["has_gt"]]
    TP = sum(len([x for x in m["tp"].split(";") if x]) for m in g)
    FP = sum(len([x for x in m["fp"].split(";") if x]) for m in g)
    FN = sum(len([x for x in m["miss"].split(";") if x]) for m in g)
    print(f"\n총 {len(man)} 프레임 (GT 있는 프레임 {len(g)})")
    print(f"GT 프레임 집계: TP {TP} FP {FP} FN {FN}  (위치 미검증 집계)")
    print(f"검출 총계 {sum(m['n_det'] for m in man)} -> {OUT}")


if __name__ == "__main__":
    main()
