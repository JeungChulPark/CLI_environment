#!/usr/bin/env python3
"""build_change_task.py — Phase3 OFF→ON 으로 '달라진 검출' 하나하나를 사람이 판정 (READ-ONLY).

집계 라벨(TP↑/FP↓/TP↓/FP↑)은 GT 가시목록에만 의존하므로 **박스 위치를 보지 않는다**.
갈색 택배박스를 초코하임으로 잡은 검출이 사라져도 'TP 감소'로 집계되는 것이 그 증상이다.
그래서 묻는 것은 집계 라벨이 아니라 늘 같은 한 가지다:

    이 박스가 실제로 그 객체 위에 있습니까?   T = 그렇다 / F = 아니다(다른 물체)

변화 방향(추가/제거)과 조합하면 좋은 변화인지 자동으로 결정된다:
    추가 + T = 개선 | 추가 + F = 악화 | 제거 + T = 악화 | 제거 + F = 개선

패널: [프레임 + 대상 박스 | 확대 crop | 그 객체의 렌더 템플릿]
산출: outputs/phase34_change_review/{cases,change_review_task.csv}
"""
import csv, glob, json, os, sys
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
import yolo_ism_object_n as o_n

AB = os.path.join(REPO, "outputs", "phase33_pipeline_ab")
OUT = os.path.join(REPO, "outputs", "phase34_change_review")
CASES = os.path.join(OUT, "cases")
FRAMES = os.path.join(RSRCH, "gt_input", "frames")
KIND = {"gain_TP": ("추가", "집계상 TP 증가"), "new_FP": ("추가", "집계상 FP 증가"),
        "lost_TP": ("제거", "집계상 TP 감소"), "removed_FP": ("제거", "집계상 FP 감소")}


def main():
    os.makedirs(CASES, exist_ok=True)
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    tdir = {o["name"]: o["template_dir"] for o in objs}
    boxes = json.load(open(os.path.join(AB, "csv", "pipeline_ab_boxes.json")))
    rows = []
    for r in csv.DictReader(open(os.path.join(AB, "csv", "pipeline_ab_manifest.csv"))):
        events = [(k, o) for k in KIND for o in filter(None, r[k].split(";"))]
        if not events: continue
        ds, fr = r["bag"], int(r["frame"])
        bgr = cv2.imread(os.path.join(FRAMES, ds, f"frame_{fr:06d}.png"))
        if bgr is None: continue
        H, W = bgr.shape[:2]; bx = boxes.get(r["name"], {})
        for kind, ob in events:
            act, agg = KIND[kind]
            src = "ON" if act == "추가" else "OFF"
            box = (bx.get(src) or {}).get(ob)
            if not box: continue
            x1, y1, x2, y2 = [int(v) for v in box]
            other = bx.get("ON" if act == "추가" else "OFF") or {}
            full = bgr.copy()
            for k, b2 in other.items():
                if k == ob: continue
                cv2.rectangle(full, (int(b2[0]), int(b2[1])), (int(b2[2]), int(b2[3])), (150, 150, 150), 1)
            col = (235, 170, 60) if act == "추가" else (60, 60, 235)
            cv2.rectangle(full, (x1, y1), (x2, y2), col, 3)
            full = cv2.resize(full, (760, 570))
            crop = bgr[max(0, y1-12):min(H, y2+12), max(0, x1-12):min(W, x2+12)]
            crop = cv2.resize(crop if crop.size else bgr, (285, 285))
            tps = sorted(glob.glob(os.path.join(tdir.get(ob, ""), "rgb_*.png")))
            tp = cv2.imread(tps[len(tps)//2]) if tps else None
            tp = cv2.resize(tp if tp is not None else np.full((285, 285, 3), 40, np.uint8), (285, 285))
            cv2.putText(crop, "in question", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
            cv2.putText(tp, ob.replace("_high", ""), (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)
            right = np.vstack([crop, tp])
            if right.shape[0] < 570:
                right = np.vstack([right, np.full((570-right.shape[0], 285, 3), 18, np.uint8)])
            cid = f"{ds}_{fr:06d}_{ob}_{kind}"
            cv2.imwrite(os.path.join(CASES, f"{cid}.png"), np.hstack([full, right]))
            rows.append({"case_id": cid, "bag_name": ds, "frame_id": fr, "class_name": ob,
                         "kind": kind, "action": act, "agg_label": agg,
                         "box": f"{x1}_{y1}_{x2}_{y2}", "gt_visible": r["gt_visible"],
                         "OFF": r["OFF"], "ON": r["ON"],
                         "image_path": os.path.join("cases", f"{cid}.png"),
                         "answer": "", "notes": ""})
    path = os.path.join(OUT, "change_review_task.csv")
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    from collections import Counter
    print(f"변화 판정 대상 {len(rows)}건 -> {path}")
    print("종류별:", dict(Counter(r["kind"] for r in rows)))
    print("클래스별:", dict(Counter(r["class_name"] for r in rows).most_common()))


if __name__ == "__main__":
    main()
