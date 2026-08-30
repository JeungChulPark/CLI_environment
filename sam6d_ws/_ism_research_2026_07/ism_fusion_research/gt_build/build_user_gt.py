#!/usr/bin/env python3
"""build_user_gt.py — 사용자가 실제로 채울 GT 입력 파일을 만든다 (READ-ONLY).

프레임 선정 (지시된 5개 층을 합집합하고 중복 제거)
  U  균등          각 데이터셋 25프레임 간격
  Z  후보 0        그 프레임에서 어떤 객체도 YOLO 후보를 얻지 못함
  T  클래스 전환   직전 수락 클래스 집합과 달라지는 지점
  G  gap 작음      한 proposal group 안 Top1-Top2 appearance 차이가 작음
  S  소형/부분가림 수락 박스가 작거나(면적 하위) mask/bbox 비율이 낮음(가림 의심)

각 프레임의 PNG 를 bag 에서 추출해 사용자가 바로 열어볼 수 있게 한다.
사용자가 채울 칸은 visible_objects 와 user_reviewed 둘뿐이다. BBox 는 요구하지 않는다.

산출
  _ism_research_2026_07/gt_input/user_visibility_gt.csv
  _ism_research_2026_07/gt_input/frames/<dataset>/frame_<6자리>.png
  _ism_research_2026_07/gt_input/contact_sheets/<dataset>_sheet_<n>.jpg
  results/gt_frame_selection.csv   (층별 선정 내역과 중복 제거 통계)
"""
import csv, os, sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                       # ism_fusion_research
RSRCH = os.path.dirname(ROOT)                      # _ism_research_2026_07
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
GT = os.path.join(RSRCH, "gt_input")
RES = os.path.join(ROOT, "results")
CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
COLOR_TOPIC = "/camera/camera/color/image_raw"
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
os.makedirs(os.path.join(GT, "frames"), exist_ok=True)
os.makedirs(os.path.join(GT, "contact_sheets"), exist_ok=True)

STRIDE = 25
GAP_TH = 0.05
N_SMALL = 6             # 데이터셋당 소형/가림 대표 프레임 수 (면적 기준·가림 기준 각각)

# 층별 상한 — 상한이 없으면 T(전환)·G(gap) 만으로 전체의 77.7% 가 뽑혀
# "전수 라벨을 피한다" 는 목적이 무너진다. 실측 근거는 results/gt_frame_selection.csv 참조.
CAP = {"U": None,       # 균등층은 무편향 분모라 상한을 걸지 않는다
       "Z": None,       # 후보 0 프레임은 proposal miss 의 직접 증거이고 수가 적다(69)
       "T": 10,         # 데이터셋당 (전환 강도 = 클래스 집합 변화량이 큰 순)
       "G": 10,         # 데이터셋당 (gap 이 작은 순)
       "S": 6}          # 데이터셋당 (면적 최소 / mask 비율 최소 각각)
SHEET_COLS, SHEET_ROWS, THUMB = 3, 3, 420      # 썸네일 420px — 300px 로 판정 실패한 전례 반영

# ---------------------------------------------------------------- 적재
boxes, pairs = {}, defaultdict(list)
allframes = defaultdict(set)
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_boxes.csv"))):
        boxes[r["uid"]] = {"ds": ds, "f": int(r["frame_id"]),
                           "bbox": [int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"])],
                           "area": float(r["bbox_area"]),
                           "mask_ratio": float(r["mask_bbox_ratio"] or 0),
                           "g50": int(r["group_iou50"])}
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) != 1:
            continue
        pairs[(ds, int(r["frame_id"]))].append(
            {"uid": r["uid"], "object": r["object"],
             "sem": float(r["sem_top5"]), "appe": float(r["appe11_clstop1"]),
             "sim_thr": float(r["sim_thr"]), "appe_gate": float(r["appe_gate"])})
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_yolo.csv"))):
        allframes[ds].add(int(r["frame_id"]))

accepted = defaultdict(set)
for (ds, f), v in pairs.items():
    by = defaultdict(list)
    for p in v:
        by[p["object"]].append(p)
    for o, lst in by.items():
        b = max(lst, key=lambda x: x["sem"])
        if b["sem"] >= b["sim_thr"] and b["appe"] >= b["appe_gate"]:
            accepted[(ds, f)].add(o)

# ---------------------------------------------------------------- 층별 선정
sel = defaultdict(set)          # (ds, frame) -> {층코드}
raw_count = defaultdict(lambda: defaultdict(int))     # 상한 적용 전 규모 (보고용)
for ds in DATASETS:
    fs = sorted(allframes[ds])

    # U 균등
    for f in fs[::STRIDE]:
        sel[(ds, f)].add("U")

    # Z 후보 0
    zs = [f for f in fs if not pairs.get((ds, f))]
    raw_count[ds]["Z"] = len(zs)
    for f in zs:
        sel[(ds, f)].add("Z")

    # T 클래스 전환 — 변화량(대칭차 크기)이 큰 순으로 상한만큼
    ts, prev = [], None
    for f in fs:
        cur = accepted.get((ds, f), set())
        if prev is not None and cur != prev:
            ts.append((len(cur ^ prev), f))
        prev = cur
    raw_count[ds]["T"] = len(ts)
    ts.sort(key=lambda x: -x[0])
    for _, f in ts[:CAP["T"]]:
        sel[(ds, f)].add("T")

    # G gap 작음 — 같은 proposal group 안 서로 다른 클래스의 appearance 최고점 차이.
    #   gap 이 작은 순으로 상한만큼 (가장 애매한 프레임 우선)
    gs = []
    for f in fs:
        g = defaultdict(dict)
        for p in pairs.get((ds, f), []):
            k = boxes[p["uid"]]["g50"]
            g[k][p["object"]] = max(g[k].get(p["object"], 0), p["appe"])
        best = None
        for k, sc in g.items():
            if len(sc) >= 2:
                v = sorted(sc.values(), reverse=True)
                best = v[0] - v[1] if best is None else min(best, v[0] - v[1])
        if best is not None and best < GAP_TH:
            gs.append((best, f))
    raw_count[ds]["G"] = len(gs)
    gs.sort()
    for _, f in gs[:CAP["G"]]:
        sel[(ds, f)].add("G")

    # S 소형 / 부분 가림 — 수락 박스 기준 면적 최소 / mask-bbox 비율 최소
    cand = []
    for f in fs:
        for p in pairs.get((ds, f), []):
            b = boxes[p["uid"]]
            if p["object"] in accepted.get((ds, f), set()):
                cand.append((b["area"], b["mask_ratio"], f))
    raw_count[ds]["S"] = len(cand)
    if cand:
        for key in (lambda x: x[0], lambda x: x[1]):
            for _, _, f in sorted(cand, key=key)[:CAP["S"]]:
                sel[(ds, f)].add("S")

print("상한 적용 전 후보 규모 (이것을 그대로 쓰면 전체의 77.7% 가 뽑힌다):")
print(f"  {'데이터셋':14s} {'Z후보0':>7} {'T전환':>7} {'G gap':>7} {'S수락박스':>9}")
for ds in DATASETS:
    r = raw_count[ds]
    print(f"  {ds:14s} {r['Z']:>7} {r['T']:>7} {r['G']:>7} {r['S']:>9}")
print(f"  상한: T={CAP['T']}/데이터셋, G={CAP['G']}/데이터셋, S={CAP['S']}/기준·데이터셋, "
      f"U·Z 는 무상한\n")

stat = defaultdict(lambda: defaultdict(int))
for (ds, f), tags in sel.items():
    for t in tags:
        stat[ds][t] += 1
    stat[ds]["_union"] += 1
print(f"{'데이터셋':14s} {'전체':>6} {'U균등':>6} {'Z후보0':>7} {'T전환':>6} {'G gap':>6} "
      f"{'S소형가림':>9} {'합집합':>7} {'비율':>6}")
tot = defaultdict(int)
for ds in DATASETS:
    s = stat[ds]; n = len(allframes[ds])
    tot["n"] += n
    for k in ("U", "Z", "T", "G", "S", "_union"):
        tot[k] += s[k]
    print(f"{ds:14s} {n:>6} {s['U']:>6} {s['Z']:>7} {s['T']:>6} {s['G']:>6} {s['S']:>9} "
          f"{s['_union']:>7} {s['_union']/n:>6.1%}")
print(f"{'합계':14s} {tot['n']:>6} {tot['U']:>6} {tot['Z']:>7} {tot['T']:>6} {tot['G']:>6} "
      f"{tot['S']:>9} {tot['_union']:>7} {tot['_union']/tot['n']:>6.1%}")

with open(os.path.join(RES, "gt_frame_selection.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["dataset", "n_frames_total", "U_uniform", "Z_no_candidate",
                "T_class_transition", "G_small_gap", "S_small_or_occluded",
                "union_selected", "selected_ratio"])
    for ds in DATASETS:
        s = stat[ds]; n = len(allframes[ds])
        w.writerow([ds, n, s["U"], s["Z"], s["T"], s["G"], s["S"], s["_union"],
                    round(s["_union"] / n, 4)])
    w.writerow(["합계", tot["n"], tot["U"], tot["Z"], tot["T"], tot["G"], tot["S"],
                tot["_union"], round(tot["_union"] / tot["n"], 4)])

# ---------------------------------------------------------------- 프레임 추출
def bag_frames(ds, want):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    ts = get_typestore(Stores.ROS2_HUMBLE)
    hi = max(want)
    with AnyReader([Path(os.path.join(CONV, ds))], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == COLOR_TOPIC]
        i = -1
        for conn, t, raw in reader.messages(connections=conns):
            i += 1
            if i > hi:
                break
            if i not in want:
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            yield i, (cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)
                      if msg.encoding.lower() == "rgb8" else buf.copy())


rows = []
for ds in DATASETS:
    want = {f for (d, f) in sel if d == ds}
    od = os.path.join(GT, "frames", ds)
    os.makedirs(od, exist_ok=True)
    thumbs = []
    for fi, bgr in bag_frames(ds, want):
        p = os.path.join(od, f"frame_{fi:06d}.png")
        cv2.imwrite(p, bgr)
        tags = "".join(sorted(sel[(ds, fi)]))
        # 우선순위: 시간이 부족하면 1 → 2 → 3 순으로만 채워도 의미 있는 분석이 가능하다.
        #   1 = 균등층(무편향 분모, proposal recall 계산에 필수)
        #   2 = 후보 0 프레임(proposal miss 직접 증거)
        #   3 = 오차분석용(전환/애매/소형·가림)
        pri = 1 if "U" in tags else (2 if "Z" in tags else 3)
        rows.append({"dataset_name": ds, "frame_id": fi, "image_path": p,
                     "visible_objects": "", "user_reviewed": "no", "notes": "",
                     "_priority": pri, "_strata": tags,
                     "_current_ism_accepted": ";".join(sorted(accepted.get((ds, fi), []))) or "none",
                     "_n_yolo_candidate_objects": len({p2["object"] for p2 in pairs.get((ds, fi), [])})})
        h, w = bgr.shape[:2]
        sc = THUMB / max(h, w)
        t = cv2.resize(bgr, (int(w * sc), int(h * sc)))
        cv2.rectangle(t, (0, 0), (t.shape[1] - 1, 26), (0, 0, 0), -1)
        cv2.putText(t, f"{ds}  f{fi}  [{tags}]", (5, 19),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
        thumbs.append(t)
    # contact sheet
    per = SHEET_COLS * SHEET_ROWS
    for si in range(0, len(thumbs), per):
        chunk = thumbs[si:si + per]
        hh = max(t.shape[0] for t in chunk); ww = max(t.shape[1] for t in chunk)
        sheet = np.full((hh * SHEET_ROWS, ww * SHEET_COLS, 3), 32, np.uint8)
        for k, t in enumerate(chunk):
            r, c = divmod(k, SHEET_COLS)
            sheet[r * hh:r * hh + t.shape[0], c * ww:c * ww + t.shape[1]] = t
        cv2.imwrite(os.path.join(GT, "contact_sheets",
                                 f"{ds}_sheet_{si // per:03d}.jpg"), sheet,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
    print(f"  {ds}: 프레임 {len(want)} 저장, 시트 {(len(thumbs)+per-1)//per}장")

rows.sort(key=lambda r: (r["_priority"], r["dataset_name"], r["frame_id"]))
cols = ["dataset_name", "frame_id", "image_path", "visible_objects", "user_reviewed", "notes",
        "_priority", "_strata", "_current_ism_accepted", "_n_yolo_candidate_objects"]
with open(os.path.join(GT, "user_visibility_gt.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=cols); w.writeheader(); w.writerows(rows)
print(f"\n-> {os.path.join(GT,'user_visibility_gt.csv')}  ({len(rows)} 행)")
