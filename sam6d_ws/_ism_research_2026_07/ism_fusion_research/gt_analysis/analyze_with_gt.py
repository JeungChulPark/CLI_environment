#!/usr/bin/env python3
"""analyze_with_gt.py — 사람이 입력한 가시성 GT 로 proposal recall / FN / confusion 산출.

이 스크립트가 처음으로 답하는 것: **분모**.
지금까지의 모든 분석은 "모델이 검출한 것" 위에서만 이뤄져
"객체가 보이는데 후보조차 없었던" 사건을 셀 수 없었다.

⚠ 가시성 기준에 관한 사용자 진술 (2026-07-21, 라벨 작성자 본인):
    "대부분의 프레임에서 객체의 정말 미세한 일부만 보였고,
     탐지하기 힘들 것 같은 부분에 대해서는 분별력이 없는 경우가 있을 것 같다."
  → 즉 visible 판정이 **관대**하고 기준이 프레임마다 일정하지 않을 수 있다.
  → 여기서 나오는 recall 은 **하한(비관적 추정)** 이다. 절대값을 그대로 인용하면 안 되고
     "관대한 가시성 기준 하에서" 라는 조건을 반드시 붙여야 한다.
  → 이 불확실성을 좁히는 방법은 §후속: miss 사례만 재검수(triage) 하는 것이다.

단계 분해 (yolo.csv 의 열을 그대로 사용)
  Y1 raw 부재        raw_candidates == 0          (conf 0.001 / iou 0.95 로도 없음)
  Y2 conf/NMS 제거   raw>0 이나 post_nms == 0
  Y3 threshold 제거  post_nms>0 이나 after_score_threshold == 0
  Y4 top_k 절단      after_score_threshold > topk_candidates 이고 topk == 0
  Y5 선택/게이트     후보는 있었으나 최종 수락 안 됨

산출
  results/gt_object_recall.csv       객체별 분모/후보/수락/recall
  results/gt_proposal_miss.csv       가시 positive 인데 후보 없음 — 단계별
  results/gt_frame_object.csv        (프레임 × 객체) 전수 판정표
  results/gt_confusion.csv           안 보이는데 수락됨 (FP) 쌍
  results/gt_triage_queue.csv        후속 재검수 대상 (miss 사례만)
"""
import csv, os
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                        # ism_fusion_research
RSRCH = os.path.dirname(ROOT)                       # _ism_research_2026_07
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
GT = os.path.join(RSRCH, "gt_input")
RES = os.path.join(ROOT, "results")
os.makedirs(RES, exist_ok=True)
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]

# ---------------------------------------------------------------- GT 적재
gt = {}
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] != "yes":
        continue
    toks = [t.strip() for t in r["visible_objects"].split(";") if t.strip()]
    gt[(r["dataset_name"], int(r["frame_id"]))] = {
        "visible": set(t for t in toks if t in OBJECTS),
        "none": "none" in toks, "unsure": "unsure" in toks,
        "priority": r["priority"], "notes": r["notes"]}
print(f"GT 프레임 {len(gt)}  (unsure {sum(1 for v in gt.values() if v['unsure'])})")

# ---------------------------------------------------------------- 관찰 덤프
yolo = {}                       # (ds, frame, object) -> yolo 단계 통계
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_yolo.csv"))):
        yolo[(ds, int(r["frame_id"]), r["object"])] = {
            k: (float(r[k]) if "conf" in k else int(r[k]))
            for k in ("raw_candidates", "raw_max_conf", "post_nms_candidates",
                      "post_max_conf", "after_score_threshold", "topk_candidates")}

cand = defaultdict(list)        # (ds, frame, object) -> 후보 pair
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) != 1:
            continue
        cand[(ds, int(r["frame_id"]), r["object"])].append(
            {"uid": r["uid"], "sem": float(r["sem_top5"]), "appe": float(r["appe11_clstop1"]),
             "sim_thr": float(r["sim_thr"]), "appe_gate": float(r["appe_gate"])})

accepted = {}                   # (ds, frame, object) -> 선택된 uid
for k, v in cand.items():
    b = max(v, key=lambda x: x["sem"])
    if b["sem"] >= b["sim_thr"] and b["appe"] >= b["appe_gate"]:
        accepted[k] = b["uid"]

# ---------------------------------------------------------------- 프레임×객체 판정
rows, triage = [], []
stat = defaultdict(lambda: defaultdict(int))
for (ds, f), g in sorted(gt.items()):
    for o in OBJECTS:
        vis = o in g["visible"]
        y = yolo.get((ds, f, o), {})
        nc = len(cand.get((ds, f, o), []))
        acc = (ds, f, o) in accepted

        if vis:
            stat[o]["visible"] += 1
            if nc > 0:
                stat[o]["has_candidate"] += 1
            if acc:
                stat[o]["accepted_TP"] += 1
        else:
            stat[o]["not_visible"] += 1
            if acc:
                stat[o]["accepted_FP"] += 1

        # 단계 판정 (가시 positive 인데 후보가 없을 때만 의미 있음)
        stage = ""
        if vis and nc == 0:
            if y.get("raw_candidates", 0) == 0:
                stage = "Y1_raw_none"
            elif y.get("post_nms_candidates", 0) == 0:
                stage = "Y2_conf_or_nms"
            elif y.get("after_score_threshold", 0) == 0:
                stage = "Y3_score_threshold"
            elif y.get("topk_candidates", 0) == 0:
                stage = "Y4_topk"
            else:
                stage = "Y_unknown"
            stat[o][stage] += 1
            triage.append({"dataset": ds, "frame_id": f, "object": o,
                           "stage": stage, "priority": g["priority"],
                           "raw_candidates": y.get("raw_candidates", ""),
                           "raw_max_conf": y.get("raw_max_conf", ""),
                           "post_max_conf": y.get("post_max_conf", ""),
                           "image_path": os.path.join(GT, "frames", ds, f"frame_{f:06d}.png"),
                           "really_detectable": "", "reviewer_note": ""})
        elif vis and not acc:
            stat[o]["Y5_select_or_gate"] += 1
            triage.append({"dataset": ds, "frame_id": f, "object": o,
                           "stage": "Y5_select_or_gate", "priority": g["priority"],
                           "raw_candidates": y.get("raw_candidates", ""),
                           "raw_max_conf": y.get("raw_max_conf", ""),
                           "post_max_conf": y.get("post_max_conf", ""),
                           "image_path": os.path.join(GT, "frames", ds, f"frame_{f:06d}.png"),
                           "really_detectable": "", "reviewer_note": ""})

        rows.append({"dataset": ds, "frame_id": f, "object": o,
                     "gt_visible": int(vis), "priority": g["priority"],
                     "n_candidate": nc, "accepted": int(acc),
                     "outcome": ("TP" if vis and acc else "FN" if vis and not acc
                                 else "FP" if acc else "TN"),
                     "miss_stage": stage,
                     "raw_candidates": y.get("raw_candidates", ""),
                     "post_nms_candidates": y.get("post_nms_candidates", ""),
                     "after_score_threshold": y.get("after_score_threshold", ""),
                     "topk_candidates": y.get("topk_candidates", "")})

with open(os.path.join(RES, "gt_frame_object.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
with open(os.path.join(RES, "gt_triage_queue.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(triage[0].keys())); w.writeheader(); w.writerows(triage)

# ---------------------------------------------------------------- 객체별 요약
out = []
for o in OBJECTS:
    s = stat[o]
    vis = s["visible"]
    out.append({
        "object": o, "visible_frames": vis, "not_visible_frames": s["not_visible"],
        "has_candidate": s["has_candidate"],
        "candidate_recall": round(s["has_candidate"] / vis, 4) if vis else "",
        "accepted_TP": s["accepted_TP"],
        "final_recall": round(s["accepted_TP"] / vis, 4) if vis else "",
        "accepted_FP": s["accepted_FP"],
        "precision": (round(s["accepted_TP"] / (s["accepted_TP"] + s["accepted_FP"]), 4)
                      if (s["accepted_TP"] + s["accepted_FP"]) else ""),
        "proposal_miss": vis - s["has_candidate"],
        "Y1_raw_none": s["Y1_raw_none"], "Y2_conf_or_nms": s["Y2_conf_or_nms"],
        "Y3_score_threshold": s["Y3_score_threshold"], "Y4_topk": s["Y4_topk"],
        "Y5_select_or_gate": s["Y5_select_or_gate"]})
with open(os.path.join(RES, "gt_object_recall.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(out[0].keys())); w.writeheader(); w.writerows(out)

miss = [r for r in rows if r["miss_stage"]]
with open(os.path.join(RES, "gt_proposal_miss.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(miss)

conf = defaultdict(int)
for r in rows:
    if r["outcome"] == "FP":
        vs = gt[(r["dataset"], r["frame_id"])]["visible"]
        conf[(r["object"], ";".join(sorted(vs)) or "(아무 객체도 안 보임)")] += 1
with open(os.path.join(RES, "gt_confusion.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["accepted_object", "actually_visible_in_frame", "count"])
    for (a, b), c in sorted(conf.items(), key=lambda x: -x[1]):
        w.writerow([a, b, c])

# ---------------------------------------------------------------- 출력
print(f"\n{'객체':22s} {'가시':>5} {'후보有':>6} {'후보recall':>10} {'수락TP':>6} "
      f"{'최종recall':>10} {'FP':>4} {'precision':>9} {'miss':>5}")
T = defaultdict(int)
for r in out:
    for k in ("visible_frames", "has_candidate", "accepted_TP", "accepted_FP", "proposal_miss",
              "Y1_raw_none", "Y2_conf_or_nms", "Y3_score_threshold", "Y4_topk",
              "Y5_select_or_gate", "not_visible_frames"):
        T[k] += r[k]
    print(f"{r['object']:22s} {r['visible_frames']:>5} {r['has_candidate']:>6} "
          f"{str(r['candidate_recall']):>10} {r['accepted_TP']:>6} "
          f"{str(r['final_recall']):>10} {r['accepted_FP']:>4} "
          f"{str(r['precision']):>9} {r['proposal_miss']:>5}")
cr = T["has_candidate"] / max(T["visible_frames"], 1)
fr = T["accepted_TP"] / max(T["visible_frames"], 1)
pr = T["accepted_TP"] / max(T["accepted_TP"] + T["accepted_FP"], 1)
print(f"{'합계':22s} {T['visible_frames']:>5} {T['has_candidate']:>6} {cr:>10.4f} "
      f"{T['accepted_TP']:>6} {fr:>10.4f} {T['accepted_FP']:>4} {pr:>9.4f} "
      f"{T['proposal_miss']:>5}")

print(f"\n=== proposal miss {T['proposal_miss']}건 단계 분해")
for k, lb in (("Y1_raw_none", "raw YOLO 후보 자체가 없음"),
              ("Y2_conf_or_nms", "conf(0.02)/NMS 에서 제거"),
              ("Y3_score_threshold", "객체별 score_threshold 에서 제거"),
              ("Y4_topk", "top_k 절단")):
    n = T[k]
    print(f"  {k:22s} {n:>5}  {n/max(T['proposal_miss'],1):>6.1%}  {lb}")
print(f"\n  후보는 있었으나 선택/게이트 탈락(Y5): {T['Y5_select_or_gate']}")
print(f"  안 보이는데 수락(FP): {T['accepted_FP']}   / 비가시 (프레임×객체) {T['not_visible_frames']}")
print(f"\n후속 재검수 대상(triage): {len(triage)}건 → results/gt_triage_queue.csv")
