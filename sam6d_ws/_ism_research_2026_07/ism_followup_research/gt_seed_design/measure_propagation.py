#!/usr/bin/env python3
"""measure_propagation.py — 최소 GT 설계의 근거를 실측한다 (READ-ONLY).

"N프레임마다 라벨" / "트랙 시작만 라벨" / "불확실 프레임만 검수" 중 무엇이 싼지는
취향이 아니라 데이터의 시간적 성질이 정한다. 여기서 그 성질을 잰다.

측정 항목
  (1) 프레임 간 BBox 이동량 — 인접 프레임 전파(propagation)가 성립하는가
      운영 수락 검출을 (dataset, object) 별 시계열로 보고, 연속 프레임 간 IoU 를 잰다.
      IoU 가 높을수록 "한 번 라벨하고 끌고 가기" 가 싸다.
  (2) 트랙(연속 수락 구간) 개수와 길이 — 방안 C 의 실제 비용
  (3) 상태 변화 지점(등장/소멸) 개수 — 방안 B 의 실제 비용
  (4) 불확실 프레임 수 — 방안 D 의 실제 비용
      정의: proposal group 안에서 Top1-Top2 gap 이 작거나, 후보가 0이거나,
            직전 프레임과 수락 클래스가 바뀌는 프레임
  (5) 간격 N 별 샘플 수 — 방안 A 의 실제 비용

산출: results/gt_propagation_stats.csv, results/gt_cost_by_plan.csv
"""
import csv, glob, json, os
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
RES = os.path.join(ROOT, "results")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    u = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / u if u > 0 else 0.0


# ---------------------------------------------------------------- 적재
boxes = {}
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_boxes.csv"))):
        boxes[r["uid"]] = (ds, int(r["frame_id"]),
                           [int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"])])

pairs = defaultdict(list)     # (ds, frame, object) -> rows
frames_of = defaultdict(set)
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) != 1:
            continue
        for k in ("sem_top5", "appe11_clstop1", "sim_thr", "appe_gate"):
            r[k] = float(r[k])
        pairs[(ds, int(r["frame_id"]), r["object"])].append(r)
        frames_of[ds].add(int(r["frame_id"]))

nframes = {}
for ds in DATASETS:
    yf = os.path.join(OBS, "frame_dumps", f"{ds}_yolo.csv")
    fs = {int(r["frame_id"]) for r in csv.DictReader(open(yf))}
    nframes[ds] = len(fs)
print("프레임 수:", nframes, "합계", sum(nframes.values()))

# 운영 baseline 재현: 객체별 sem_top5 최대 후보 → 두 게이트
accepted = {}
for k, v in pairs.items():
    b = max(v, key=lambda x: x["sem_top5"])
    if b["sem_top5"] >= b["sim_thr"] and b["appe11_clstop1"] >= b["appe_gate"]:
        accepted[k] = b

# ---------------------------------------------------------------- (1)(2)(3)
prop_rows, plan_rows = [], []
for ds in DATASETS:
    objs = sorted({k[2] for k in accepted if k[0] == ds})
    for obj in objs:
        fr = sorted(f for (d, f, o) in accepted if d == ds and o == obj)
        if len(fr) < 2:
            continue
        # 트랙 = 프레임 간격이 GAPMAX 이하로 이어지는 구간
        GAPMAX = 3
        tracks, cur = [], [fr[0]]
        for a, b in zip(fr, fr[1:]):
            (cur.append(b) if b - a <= GAPMAX else (tracks.append(cur), cur := [b]))
        tracks.append(cur)
        # 인접 프레임 IoU
        ious = []
        for a, b in zip(fr, fr[1:]):
            if b - a != 1:
                continue
            ba = boxes[accepted[(ds, a, obj)]["uid"]][2]
            bb = boxes[accepted[(ds, b, obj)]["uid"]][2]
            ious.append(iou(ba, bb))
        prop_rows.append({
            "dataset": ds, "object": obj,
            "n_frames_total": nframes[ds], "n_accepted_frames": len(fr),
            "accept_rate": round(len(fr) / nframes[ds], 4),
            "n_tracks": len(tracks),
            "track_len_median": int(np.median([len(t) for t in tracks])),
            "track_len_max": max(len(t) for t in tracks),
            "n_appear_events": len(tracks),          # 등장 = 트랙 시작
            "n_disappear_events": len(tracks),       # 소멸 = 트랙 끝
            "n_consecutive_pairs": len(ious),
            "iou_consecutive_median": round(float(np.median(ious)), 4) if ious else "",
            "iou_consecutive_p10": round(float(np.percentile(ious, 10)), 4) if ious else "",
            "frac_iou_gt_0.5": round(float(np.mean(np.array(ious) > 0.5)), 4) if ious else "",
            "frac_iou_gt_0.7": round(float(np.mean(np.array(ious) > 0.7)), 4) if ious else "",
        })

with open(os.path.join(RES, "gt_propagation_stats.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(prop_rows[0].keys())); w.writeheader(); w.writerows(prop_rows)

# ---------------------------------------------------------------- (4) 불확실 프레임
groups = defaultdict(dict)     # (ds, frame) -> object -> best appe
for (ds, fr, obj), v in pairs.items():
    b = max(v, key=lambda x: x["appe11_clstop1"])
    groups[(ds, fr)][obj] = b["appe11_clstop1"]

uncertain = defaultdict(set)
for (ds, fr), sc in groups.items():
    if len(sc) >= 2:
        s = sorted(sc.values(), reverse=True)
        if s[0] - s[1] < 0.05:
            uncertain[ds].add(fr)
# 후보 0 프레임
for ds in DATASETS:
    allf = {int(r["frame_id"]) for r in csv.DictReader(
        open(os.path.join(OBS, "frame_dumps", f"{ds}_yolo.csv")))}
    have = {f for (d, f) in groups if d == ds}
    uncertain[ds] |= (allf - have)
# 클래스 전환 프레임
for ds in DATASETS:
    accf = defaultdict(set)
    for (d, f, o) in accepted:
        if d == ds:
            accf[f].add(o)
    fs = sorted(accf)
    for a, b in zip(fs, fs[1:]):
        if b - a <= 3 and accf[a] != accf[b]:
            uncertain[ds].add(b)

# ---------------------------------------------------------------- (5) 방안별 비용
TOT = sum(nframes.values())
n_obj_ds = defaultdict(int)
for r in prop_rows:
    n_obj_ds[r["dataset"]] += 1

for ds in DATASETS:
    P = [r for r in prop_rows if r["dataset"] == ds]
    tracks = sum(r["n_tracks"] for r in P)
    plan_rows.append({
        "dataset": ds, "n_frames": nframes[ds], "n_objects_seen": len(P),
        "planA_every10": nframes[ds] // 10,
        "planA_every25": nframes[ds] // 25,
        "planA_every50": nframes[ds] // 50,
        "planB_state_changes": tracks * 2,           # 등장 + 소멸
        "planC_track_starts": tracks,
        "planD_uncertain_frames": len(uncertain[ds]),
        "planD_rate": round(len(uncertain[ds]) / nframes[ds], 4),
    })
plan_rows.append({
    "dataset": "합계", "n_frames": TOT,
    "n_objects_seen": "",
    **{k: sum(r[k] for r in plan_rows) for k in
       ("planA_every10", "planA_every25", "planA_every50",
        "planB_state_changes", "planC_track_starts", "planD_uncertain_frames")},
})
with open(os.path.join(RES, "gt_cost_by_plan.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(plan_rows[0].keys()), extrasaction="ignore")
    w.writeheader(); w.writerows(plan_rows)

print(f"\n{'데이터셋':14s} {'프레임':>6} {'객체':>4} {'A/10':>6} {'A/25':>6} {'A/50':>6} "
      f"{'B상태변화':>9} {'C트랙시작':>9} {'D불확실':>8} {'D비율':>7}")
for r in plan_rows:
    print(f"{r['dataset']:14s} {r['n_frames']:>6} {str(r['n_objects_seen']):>4} "
          f"{r['planA_every10']:>6} {r['planA_every25']:>6} {r['planA_every50']:>6} "
          f"{r['planB_state_changes']:>9} {r['planC_track_starts']:>9} "
          f"{r['planD_uncertain_frames']:>8} {str(r.get('planD_rate','')):>7}")

print(f"\n인접 프레임 BBox IoU (전파 가능성)")
iv = [r["iou_consecutive_median"] for r in prop_rows if r["iou_consecutive_median"] != ""]
g5 = [r["frac_iou_gt_0.5"] for r in prop_rows if r["frac_iou_gt_0.5"] != ""]
g7 = [r["frac_iou_gt_0.7"] for r in prop_rows if r["frac_iou_gt_0.7"] != ""]
print(f"  중앙값의 중앙값 = {np.median(iv):.3f} | IoU>0.5 비율 평균 = {np.mean(g5):.3f} "
      f"| IoU>0.7 비율 평균 = {np.mean(g7):.3f}")
print(f"  트랙 길이 중앙값 분포: {sorted(r['track_len_median'] for r in prop_rows)}")
print(f"\n-> {os.path.join(RES,'gt_propagation_stats.csv')}\n-> {os.path.join(RES,'gt_cost_by_plan.csv')}")
