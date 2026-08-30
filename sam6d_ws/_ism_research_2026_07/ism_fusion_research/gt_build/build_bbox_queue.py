#!/usr/bin/env python3
"""build_bbox_queue.py — 가시성 GT 만으로는 확정할 수 없는 항목의 검수 목록 (READ-ONLY).

`visible_objects`(이름만) 로는 다음이 확정되지 않는다.
  (1) 후보 BBox 가 실제 객체 위치인가 (IoU)
  (2) MobileSAM mask 품질
  (3) 같은 프레임 안 동일 클래스 다중 인스턴스

세 항목이 **실제로 문제가 될 수 있는 프레임만** 추려 후속 BBox 검수 대상으로 남긴다.
전 프레임에 BBox 를 요구하지 않기 위한 목록이다.

선정 기준
  I  IoU 불확실   한 proposal group 에 2개 이상 클래스가 수락됨 (같은 물체에 두 이름)
  M  mask 의심    수락 박스의 mask/bbox 비율이 하위 2% 이거나 연결성분이 3개 이상
  X  다중 인스턴스 같은 객체의 후보 박스가 2개 이상이고 서로 IoU<0.1 (멀리 떨어짐)

산출: results/bbox_review_queue.csv
"""
import csv, os
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
RES = os.path.join(ROOT, "results")
GT = os.path.join(RSRCH, "gt_input")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    u = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / u if u > 0 else 0.0


boxes, pairs = {}, defaultdict(list)
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_boxes.csv"))):
        boxes[r["uid"]] = {"ds": ds, "f": int(r["frame_id"]), "g50": int(r["group_iou50"]),
                           "bbox": [int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"])],
                           "mask_ratio": float(r["mask_bbox_ratio"] or 0),
                           "ncomp": int(r["n_components"] or 0)}
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) != 1:
            continue
        pairs[(ds, int(r["frame_id"]))].append(
            {"uid": r["uid"], "object": r["object"], "sem": float(r["sem_top5"]),
             "appe": float(r["appe11_clstop1"]), "sim_thr": float(r["sim_thr"]),
             "appe_gate": float(r["appe_gate"])})

acc = defaultdict(dict)           # (ds,f) -> object -> uid
for k, v in pairs.items():
    by = defaultdict(list)
    for p in v:
        by[p["object"]].append(p)
    for o, lst in by.items():
        b = max(lst, key=lambda x: x["sem"])
        if b["sem"] >= b["sim_thr"] and b["appe"] >= b["appe_gate"]:
            acc[k][o] = b["uid"]

mr = np.array([boxes[u]["mask_ratio"] for k in acc for u in acc[k].values()])
MR_TH = float(np.percentile(mr, 2)) if len(mr) else 0.0
print(f"mask/bbox 비율 하위 2% 기준값 = {MR_TH:.4f}  (수락 박스 {len(mr):,})")

sel = defaultdict(lambda: {"tags": set(), "detail": []})
for k, objmap in acc.items():
    # I : 같은 group 에 2개 이상 클래스가 수락
    g = defaultdict(list)
    for o, u in objmap.items():
        g[boxes[u]["g50"]].append(o)
    for gid, os_ in g.items():
        if len(os_) >= 2:
            sel[k]["tags"].add("I")
            sel[k]["detail"].append(f"group{gid}:{'+'.join(sorted(os_))}")
    # M : mask 의심
    for o, u in objmap.items():
        b = boxes[u]
        if b["mask_ratio"] <= MR_TH or b["ncomp"] >= 3:
            sel[k]["tags"].add("M")
            sel[k]["detail"].append(f"mask:{o}(ratio={b['mask_ratio']:.2f},comp={b['ncomp']})")
    # X : 다중 인스턴스 의심
    #   top_k=3 이라 한 객체가 통상 3개 후보를 받으므로 "후보가 여럿" 만으로는 근거가 안 된다
    #   (그 기준으로는 2,064 프레임 = 79% 가 걸린다).
    #   두 박스가 **각각 독립적으로 두 게이트를 통과**하고 서로 멀리 떨어진 경우만 남긴다
    #   — 현재는 객체당 슬롯이 1개라 둘 중 하나만 수락되지만, 실제로 인스턴스가 2개라는 신호다.
    byobj = defaultdict(list)
    for p in pairs[k]:
        if p["sem"] >= p["sim_thr"] and p["appe"] >= p["appe_gate"]:
            byobj[p["object"]].append(p["uid"])
    for o, us in byobj.items():
        if len(us) < 2:
            continue
        far = [(a, b) for i, a in enumerate(us) for b in us[i+1:]
               if iou(boxes[a]["bbox"], boxes[b]["bbox"]) < 0.1]
        if far:
            sel[k]["tags"].add("X")
            sel[k]["detail"].append(f"multi:{o} 게이트통과 {len(us)}개")

gt_frames = {(r["dataset_name"], int(r["frame_id"]))
             for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv")))}

rows = []
for (ds, f), v in sorted(sel.items()):
    rows.append({"dataset": ds, "frame_id": f, "reasons": "".join(sorted(v["tags"])),
                 "needs_iou_check": int("I" in v["tags"]),
                 "needs_mask_check": int("M" in v["tags"]),
                 "needs_instance_check": int("X" in v["tags"]),
                 "in_visibility_gt": int((ds, f) in gt_frames),
                 "detail": " | ".join(sorted(set(v["detail"]))[:4])})

with open(os.path.join(RES, "bbox_review_queue.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

from collections import Counter
print(f"\nBBox 검수 후보 {len(rows)} 프레임 (전체 2,596 의 {len(rows)/2596:.1%})")
print("  사유별:", Counter(t for r in rows for t in r["reasons"]).most_common())
print("  조합별:", Counter(r["reasons"] for r in rows).most_common())
print(f"  가시성 GT 목록과 겹침: {sum(r['in_visibility_gt'] for r in rows)}")
print(f"-> {os.path.join(RES,'bbox_review_queue.csv')}")
