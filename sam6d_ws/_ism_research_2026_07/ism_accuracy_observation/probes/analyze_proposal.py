#!/usr/bin/env python3
"""analyze_proposal.py — 검증 3: YOLO-World proposal 단계 전수 통계 (READ-ONLY).

라벨 없이 계산 가능한 부분만 여기서 처리한다:
  · RAW(conf=0.001, iou=0.95) 후보 수 vs 운영(post-NMS/conf) 후보 수 vs top_k 후보 수
  · Y1(raw 자체가 0) 과 Y2/Y3(raw 엔 있는데 conf/NMS/top_k 에서 제거) 의 프레임 수 분리
  · 객체별·데이터셋별 후보 존재율
라벨이 필요한 "실제 객체가 보이는데 후보가 없다(proposal miss)" 비율은
labels/frame_object_labels.csv 가 있을 때만 계산한다.
"""
import csv, glob, json, os
from collections import defaultdict, Counter

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RES = os.path.join(ROOT, "results")

rows = []
for f in sorted(glob.glob(os.path.join(ROOT, "frame_dumps", "*_yolo.csv"))):
    rows += list(csv.DictReader(open(f)))
for r in rows:
    for k in ("raw_candidates", "post_nms_candidates", "after_score_threshold",
              "topk_candidates", "frame_id"):
        r[k] = int(r[k])
    for k in ("raw_max_conf", "post_max_conf", "score_threshold"):
        r[k] = float(r[k])
print(f"frame-object rows = {len(rows)}")

per = defaultdict(lambda: Counter())
for r in rows:
    k = (r["dataset"], r["object"])
    p = per[k]
    p["frames"] += 1
    p["raw_zero"] += (r["raw_candidates"] == 0)
    p["post_zero"] += (r["post_nms_candidates"] == 0)
    p["topk_zero"] += (r["topk_candidates"] == 0)
    p["raw_pos_post_zero"] += (r["raw_candidates"] > 0 and r["post_nms_candidates"] == 0)
    p["post_pos_thr_zero"] += (r["post_nms_candidates"] > 0 and r["after_score_threshold"] == 0)
    p["thr_pos_topk_cut"] += (r["after_score_threshold"] > r["topk_candidates"])

out = []
for (ds, ob), p in sorted(per.items()):
    n = p["frames"]
    out.append({"dataset": ds, "object": ob, "frames": n,
                "raw_zero": p["raw_zero"], "raw_zero_rate": round(p["raw_zero"]/n, 4),
                "post_zero": p["post_zero"], "post_zero_rate": round(p["post_zero"]/n, 4),
                "topk_zero": p["topk_zero"], "topk_zero_rate": round(p["topk_zero"]/n, 4),
                "removed_by_conf_or_nms": p["raw_pos_post_zero"],
                "removed_by_score_threshold": p["post_pos_thr_zero"],
                "truncated_by_topk": p["thr_pos_topk_cut"]})
with open(os.path.join(RES, "yolo_candidate_stage_stats.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(out[0].keys())); w.writeheader(); w.writerows(out)

obj = defaultdict(lambda: Counter())
for r in out:
    o = obj[r["object"]]
    for k in ("frames", "raw_zero", "post_zero", "topk_zero", "removed_by_conf_or_nms",
              "removed_by_score_threshold", "truncated_by_topk"):
        o[k] += r[k]
print(f"\n{'object':22s} {'frames':>7s} {'raw=0':>7s} {'post=0':>7s} {'topk=0':>7s} "
      f"{'conf/NMS제거':>11s} {'thr제거':>8s} {'topk절단':>8s}")
summary = {}
for o, c in sorted(obj.items()):
    n = c["frames"]
    print(f"{o:22s} {n:7d} {c['raw_zero']:7d} {c['post_zero']:7d} {c['topk_zero']:7d} "
          f"{c['removed_by_conf_or_nms']:11d} {c['removed_by_score_threshold']:8d} {c['truncated_by_topk']:8d}")
    summary[o] = {k: c[k] for k in c} | {
        "raw_zero_rate": round(c["raw_zero"]/n, 4), "topk_zero_rate": round(c["topk_zero"]/n, 4)}
json.dump(summary, open(os.path.join(RES, "yolo_candidate_stage_summary.json"), "w"), indent=2)
print(f"\n-> {RES}/yolo_candidate_stage_stats.csv")
