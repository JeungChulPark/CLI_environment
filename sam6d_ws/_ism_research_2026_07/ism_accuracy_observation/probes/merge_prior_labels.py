#!/usr/bin/env python3
"""merge_prior_labels.py — 2026-07-20 이전 조사(ism_accuracy_analysis)의 박스 라벨을
이번 전수 덤프의 uid 로 매핑해 재사용한다 (READ-ONLY).

이전 uid : "<source>__frame_<6자리>__x1_y1_x2_y2"   (pem_inputs 번들 기준)
이번 uid : "<dataset>|<frame_id>|x1_y1_x2_y2"       (bag color 메시지 인덱스 기준)
pem_inputs 의 frame_<ci> 는 color 메시지 인덱스이므로 두 인덱스는 동일 좌표계다.
좌표까지 정확히 일치하는 것만 재사용하고, provenance 를 유지한다.
"""
import csv, glob, os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)                     # _ism_research_2026_07 (형제 폴더 기준)
PRIOR = os.path.join(RSRCH, "ism_accuracy_analysis", "labels", "box_labels.csv")

new_uids = set()
for f in sorted(glob.glob(os.path.join(ROOT, "frame_dumps", "*_boxes.csv"))):
    for r in csv.DictReader(open(f)):
        new_uids.add(r["uid"])

rows, hit, miss = [], 0, 0
if os.path.isfile(PRIOR):
    for r in csv.DictReader(open(PRIOR)):
        src, frame, coords = r["uid"].split("__")
        if not src.startswith("sam_"):
            continue                      # SAM_loop* 등은 이번 6개 데이터가 아님
        fid = int(frame.split("_")[1])
        u = f"{src}|{fid}|{coords}"
        if u in new_uids:
            hit += 1
            rows.append({"uid": u, "dataset": src, "frame_id": fid,
                         "true_class": r["true_class"],
                         "label_source": "claude-vision (2026-07-20 prior study, re-used)",
                         "label_confidence": "provisional",
                         "review_status": "not_reviewed", "reviewer": ""})
        else:
            miss += 1
print(f"prior sam_* labels: matched {hit}, unmatched {miss}")
os.makedirs(os.path.join(ROOT, "labels"), exist_ok=True)
out = os.path.join(ROOT, "labels", "box_labels_prior.csv")
if rows:
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print(f"-> {out}")
