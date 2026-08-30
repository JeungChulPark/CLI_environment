#!/usr/bin/env python3
"""build_gt_plan.py — 최소 수동 GT 계획과 실제 라벨 요청 목록을 만든다 (READ-ONLY).

measure_propagation.py 실측 결과가 계획을 결정한다.
  - 방안 D(불확실 프레임)는 2,596 중 1,894(73%) → 전수와 다름없어 기각
  - 방안 B(1,614) / C(807) 도 방안 A(간격 25 → 101)보다 비싸다
  - 인접 프레임 BBox IoU 중앙값 0.452, IoU>0.7 은 32% → 단순 BBox 복사 전파는 불가

또한 결정적으로, 방안 B/C/D 는 모두 **검출 결과에 조건부**다. proposal miss(객체가
보이는데 후보가 아예 없음)를 재려면 검출과 무관하게 뽑은 프레임이 필요하다.
따라서 1순위는 방안 A(균등 간격) + 오차분석용 D 층(stratified oversample) 이다.

산출:
  results/minimum_gt_plan.csv     방안별 비용/획득물/한계
  results/human_label_request.csv 사용자가 실제로 라벨할 프레임 목록
"""
import csv, os
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
RES = os.path.join(ROOT, "results")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]

# ---------------------------------------------------------------- 방안 비교표
plan = list(csv.DictReader(open(os.path.join(RES, "gt_cost_by_plan.csv"))))
tot = [r for r in plan if r["dataset"] == "합계"][0]
prop = list(csv.DictReader(open(os.path.join(RES, "gt_propagation_stats.csv"))))
iou_med = np.median([float(r["iou_consecutive_median"]) for r in prop
                     if r["iou_consecutive_median"]])
frac07 = np.mean([float(r["frac_iou_gt_0.7"]) for r in prop if r["frac_iou_gt_0.7"]])

PLANS = [
    {"plan": "A-10", "설명": "10프레임마다 프레임 단위 가시객체 라벨",
     "사용자_입력_항목": "프레임에 보이는 객체 이름 목록 (BBox 없음)",
     "프레임수_6데이터": int(tot["planA_every10"]),
     "검출_비의존": 1, "proposal_miss_측정가능": 1, "semantic_HSV_검증가능": 0},
    {"plan": "A-25", "설명": "25프레임마다 프레임 단위 가시객체 라벨",
     "사용자_입력_항목": "프레임에 보이는 객체 이름 목록 (BBox 없음)",
     "프레임수_6데이터": int(tot["planA_every25"]),
     "검출_비의존": 1, "proposal_miss_측정가능": 1, "semantic_HSV_검증가능": 0},
    {"plan": "A-50", "설명": "50프레임마다 프레임 단위 가시객체 라벨",
     "사용자_입력_항목": "프레임에 보이는 객체 이름 목록 (BBox 없음)",
     "프레임수_6데이터": int(tot["planA_every50"]),
     "검출_비의존": 1, "proposal_miss_측정가능": 1, "semantic_HSV_검증가능": 0},
    {"plan": "B", "설명": "객체 상태 변화 지점(등장/소멸)만 라벨",
     "사용자_입력_항목": "등장/소멸 프레임과 객체",
     "프레임수_6데이터": int(tot["planB_state_changes"]),
     "검출_비의존": 0, "proposal_miss_측정가능": 0, "semantic_HSV_검증가능": 0},
    {"plan": "C", "설명": "트랙 시작 프레임의 BBox만 라벨 후 전파",
     "사용자_입력_항목": "객체별 최초 BBox",
     "프레임수_6데이터": int(tot["planC_track_starts"]),
     "검출_비의존": 0, "proposal_miss_측정가능": 0, "semantic_HSV_검증가능": 1},
    {"plan": "D", "설명": "모델이 불확실한 프레임만 검수",
     "사용자_입력_항목": "제시된 Top-3 중 정답 선택",
     "프레임수_6데이터": int(tot["planD_uncertain_frames"]),
     "검출_비의존": 0, "proposal_miss_측정가능": 0, "semantic_HSV_검증가능": 1},
    {"plan": "A-25 + D층(200)", "설명": "권장. 균등 A-25 로 무편향 분모 확보 +"
                                        " 불확실 프레임 200장을 오차분석용으로 추가 검수",
     "사용자_입력_항목": "A-25: 보이는 객체 목록 / D층: Top-3 중 선택",
     "프레임수_6데이터": int(tot["planA_every25"]) + 200,
     "검출_비의존": 1, "proposal_miss_측정가능": 1, "semantic_HSV_검증가능": 1},
]
MIN_PER_FRAME = 0.5      # 프레임당 소요(분) — 10객체 목록 체크 기준의 작업 가정
for p in PLANS:
    p["예상소요_시간"] = round(p["프레임수_6데이터"] * MIN_PER_FRAME / 60, 1)
    p["전체대비_비율"] = round(p["프레임수_6데이터"] / 2596, 3)
    p["자동전파_가능성"] = ("불가 — 인접 IoU 중앙 %.2f, IoU>0.7 은 %.0f%%" % (iou_med, frac07 * 100)
                       if p["plan"] == "C" else "해당없음")
with open(os.path.join(RES, "minimum_gt_plan.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(PLANS[0].keys())); w.writeheader(); w.writerows(PLANS)

print(f"{'방안':16s} {'프레임':>7} {'전체비':>7} {'시간(h)':>7} {'검출비의존':>10} "
      f"{'proposal측정':>12} {'sem/HSV검증':>11}")
for p in PLANS:
    print(f"{p['plan']:16s} {p['프레임수_6데이터']:>7} {p['전체대비_비율']:>7.3f} "
          f"{p['예상소요_시간']:>7} {'O' if p['검출_비의존'] else 'X':>10} "
          f"{'O' if p['proposal_miss_측정가능'] else 'X':>12} "
          f"{'O' if p['semantic_HSV_검증가능'] else 'X':>11}")

# ---------------------------------------------------------------- 실제 라벨 요청 목록
frames = {}
for ds in DATASETS:
    fs = sorted({int(r["frame_id"]) for r in csv.DictReader(
        open(os.path.join(OBS, "frame_dumps", f"{ds}_yolo.csv")))})
    frames[ds] = fs

# 현재 운영 판정 (참고 표시용 — 사용자는 이것을 보고 '맞다/틀리다'가 아니라
# '실제로 무엇이 보이는지' 를 독립적으로 적어야 편향이 없다)
pairs = defaultdict(list)
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) != 1:
            continue
        for k in ("sem_top5", "appe11_clstop1", "sim_thr", "appe_gate"):
            r[k] = float(r[k])
        pairs[(ds, int(r["frame_id"]), r["object"])].append(r)
acc = defaultdict(set)
for (ds, fr, o), v in pairs.items():
    b = max(v, key=lambda x: x["sem_top5"])
    if b["sem_top5"] >= b["sim_thr"] and b["appe11_clstop1"] >= b["appe_gate"]:
        acc[(ds, fr)].add(o)

# 불확실 층: Top1-Top2 gap 이 작은 프레임 상위 N
gaps = []
for ds in DATASETS:
    g = defaultdict(dict)
    for (d, fr, o), v in pairs.items():
        if d == ds:
            g[fr][o] = max(x["appe11_clstop1"] for x in v)
    for fr, sc in g.items():
        if len(sc) >= 2:
            s = sorted(sc.values(), reverse=True)
            gaps.append((s[0] - s[1], ds, fr))
gaps.sort()
D_LAYER = 200
dset = {(ds, fr) for _, ds, fr in gaps[:D_LAYER]}

req = []
for ds in DATASETS:
    for fr in frames[ds][::25]:
        req.append({"dataset": ds, "frame_id": fr, "stratum": "uniform_every25",
                    "priority": 1, "reason": "무편향 분모 — proposal recall 산출용"})
for ds, fr in sorted(dset):
    if not any(r["dataset"] == ds and r["frame_id"] == fr for r in req):
        req.append({"dataset": ds, "frame_id": fr, "stratum": "uncertain_top200",
                    "priority": 2, "reason": "Top1-Top2 gap 최소 — 클래스 경쟁 오차분석용"})
for r in req:
    ds, fr = r["dataset"], r["frame_id"]
    r["current_ism_accepted"] = ";".join(sorted(acc.get((ds, fr), []))) or "(없음)"
    r["n_current_accepted"] = len(acc.get((ds, fr), []))
    # 사용자가 채울 칸
    r["visible_objects"] = ""
    r["occluded_objects"] = ""
    r["label_source"] = ""
    r["confidence"] = ""
    r["review_status"] = "pending"
    r["reviewer"] = ""
    r["notes"] = ""

with open(os.path.join(RES, "human_label_request.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(req[0].keys())); w.writeheader(); w.writerows(req)

n1 = sum(1 for r in req if r["priority"] == 1)
print(f"\n라벨 요청 {len(req)} 프레임 = 균등 {n1} + 불확실층 {len(req)-n1}")
print(f"  전체 2,596 프레임의 {len(req)/2596:.1%}")
print(f"-> {os.path.join(RES,'minimum_gt_plan.csv')}")
print(f"-> {os.path.join(RES,'human_label_request.csv')}")
