#!/usr/bin/env python3
"""build_fp_audit_task.py — 'FP로 판정된 검출'을 사람이 직접 재판정하는 과제 생성 (READ-ONLY).

대상: A(현행 block11) / B1 / B2(block2+9) 중 **어느 하나라도** ACCEPT 했지만
      해당 프레임 GT 가시 목록에 없어 FP로 집계된 (bag, frame, object) 전부.
질문: 이 빨간 박스가 실제로 그 객체 위에 있는가?  T=맞음(GT 누락) / F=오검출 / U=불확실

패널: [프레임+빨간박스 | 후보 확대 crop | 그 객체의 렌더 템플릿]
산출: outputs/phase32_fp_audit/{cases,fp_audit_task.csv}
"""
import csv, glob, os, sys
from collections import defaultdict
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, os.path.join(RSRCH, "phase21_fn_audit")); sys.path.insert(0, REPO)
import phase21_common as p21
import yolo_ism_object_n as o_n

OUT = os.path.join(REPO, "outputs", "phase32_fp_audit")
CASES = os.path.join(OUT, "cases")
FRAMES = os.path.join(RSRCH, "gt_input", "frames")
POOL = os.path.join(REPO, "outputs", "phase22_training_free_gate", "csv", "candidate_pool.csv")
CFG = [("A", lambda c: c["a11"], 0.550),
       ("B1", lambda c: (c["a2"] + c["a9"]) / 2, 0.620),
       ("B2", lambda c: (c["a2"] + c["a9"]) / 2, 0.605)]


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    it = ix * iy; ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - it
    return it / ua if ua else 0.0


def main():
    os.makedirs(CASES, exist_ok=True)
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    tdir = {o["name"]: o["template_dir"] for o in objs}
    cells = defaultdict(list)
    for r in csv.DictReader(open(POOL)):
        x1, y1, x2, y2 = [int(v) for v in r["uid"].rsplit("|", 1)[1].split("_")]
        cells[(r["dataset"], int(r["frame_id"]), r["object"])].append(dict(
            sem=float(r["sem_top5"]), a11=float(r["appe11"]), a2=float(r["appe2"]),
            a9=float(r["appe9"]), hsv=float(r["hsv"]), st=float(r["sim_thr"]),
            ht=float(r["hsv_thr"]), conf=float(r["conf"]),
            sel=r["is_selected"] == "1", box=(x1, y1, x2, y2)))

    def pick(cs):
        for c in cs:
            if c["sel"]: return c
        return max(cs, key=lambda c: c["sem"])

    gt = p21.load_gt(); rows = []
    for (ds, fr) in sorted(gt.keys()):
        vis = gt[(ds, fr)]
        acc = {}
        for tag, fn, g in CFG:
            d = {}
            for o in p21.OBJECTS:
                cs = cells.get((ds, fr, o))
                if not cs: continue
                c = pick(cs)
                if c["sem"] >= c["st"] and fn(c) >= g and c["hsv"] >= c["ht"]:
                    d[o] = c
            acc[tag] = d
        fps = sorted((set(acc["A"]) | set(acc["B1"]) | set(acc["B2"])) - vis)
        if not fps: continue
        bgr = cv2.imread(os.path.join(FRAMES, ds, f"frame_{fr:06d}.png"))
        if bgr is None: continue
        H, W = bgr.shape[:2]
        for ob in fps:
            c = acc["B2"].get(ob) or acc["A"].get(ob) or acc["B1"].get(ob)
            x1, y1, x2, y2 = c["box"]
            who = [t for t, _, _ in CFG if ob in acc[t]]
            # 같은 박스를 함께 받은 다른 클래스(중복 라벨)
            dup = sorted(k for k in acc["B2"]
                         if k != ob and iou(acc["B2"][k]["box"], c["box"]) >= 0.9)
            full = bgr.copy()
            for k, cc in sorted(acc["B2"].items()):
                if k == ob: continue
                bx = cc["box"]
                cv2.rectangle(full, bx[:2], bx[2:], (150, 150, 150), 1)
            cv2.rectangle(full, (x1, y1), (x2, y2), (60, 60, 235), 3)
            full = cv2.resize(full, (760, 570))
            crop = bgr[max(0, y1-12):min(H, y2+12), max(0, x1-12):min(W, x2+12)]
            crop = cv2.resize(crop if crop.size else bgr, (285, 285))
            tps = sorted(glob.glob(os.path.join(tdir.get(ob, ""), "rgb_*.png")))
            tp = cv2.imread(tps[len(tps)//2]) if tps else None
            tp = cv2.resize(tp if tp is not None else np.full((285,285,3),40,np.uint8), (285, 285))
            cv2.putText(crop, "detected", (6,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60,60,235), 2)
            cv2.putText(tp, ob.replace("_high",""), (6,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,200,0), 2)
            right = np.vstack([crop, tp])
            pad = np.full((570 - right.shape[0], 285, 3), 18, np.uint8) if right.shape[0] < 570 else None
            if pad is not None: right = np.vstack([right, pad])
            img = np.hstack([full, right])
            cid = f"{ds}_{fr:06d}_{ob}"
            cv2.imwrite(os.path.join(CASES, f"{cid}.png"), img)
            rows.append({"case_id": cid, "bag_name": ds, "frame_id": fr, "class_name": ob,
                         "box": f"{x1}_{y1}_{x2}_{y2}", "configs": ";".join(who),
                         "dup_with": ";".join(k.replace("_high","") for k in dup),
                         "gt_visible": ";".join(sorted(vis)),
                         "conf": round(c["conf"], 3), "sem": round(c["sem"], 3),
                         "appe11": round(c["a11"], 3), "appe29": round((c["a2"]+c["a9"])/2, 3),
                         "hsv": round(c["hsv"], 3),
                         "image_path": os.path.join("cases", f"{cid}.png"),
                         "answer": "", "notes": ""})
    path = os.path.join(OUT, "fp_audit_task.csv")
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    from collections import Counter
    print(f"FP 재판정 대상 {len(rows)}건 -> {path}")
    print("클래스별:", dict(Counter(r["class_name"] for r in rows).most_common()))
    print("구성별  :", dict(Counter(r["configs"] for r in rows).most_common()))
    print("중복라벨 :", sum(1 for r in rows if r["dup_with"]))


if __name__ == "__main__":
    main()
