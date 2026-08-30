#!/usr/bin/env python3
"""fn345_breakdown.py — HSV hard gate 적용 후 FN 345건을 단계별로 분해한다 (READ-ONLY).

구성 검증
    baseline FN 334 + HSV 신규 거절 11 = 345   (HSV 가 복구한 FN 0건 — 게이트는 제거만 함)

"정답 위치 후보가 있었다" 의 판정 — 박스 개수로 판정하지 않는다.
지시된 증거 우선순위대로 사용하고, 확정 불가는 F12(unresolved) 로 남긴다.

  E1 사람 메모(triage note 115건)  "실제 X는 후보에 들어오지 못함" / "실제 X도 선정됨"
  E2 provisional 박스라벨 504건    그 객체의 후보 중 라벨이 그 객체인 것이 있는가
  E3 인접 프레임 위치 일관성        전체 2,596 프레임 덤프에서 f±W 안에 그 객체가 수락된
                                  프레임을 찾아 그 박스와 현재 프레임 후보의 IoU 를 본다
  E4 MobileSAM mask 품질           mask_bbox_ratio / 연결성분 (보조)
  (없으면) F12

분류 (하나만 부여, 위에서부터 우선)
  F11 GT/라벨 오류        triage verdict = N
  F1  검출 난이도          triage verdict = B (아주 일부만 보임)
  F10 HSV 신규 거절        baseline TP → HSV FN
  F2  raw YOLO 부재        후보 0 이고 raw_candidates == 0
  F3  conf/NMS 제거        후보 0 이고 post_nms == 0
  F4  YOLO threshold 제거  후보 0 이고 after_score_threshold == 0
  F6  top_k 제거           후보 0 이고 after_score_threshold > 0 인데 topk == 0
  F5  wrong-location       후보는 있으나 정답 위치가 아님 (E1/E2/E3 근거)
  F7  semantic 선택 실패   정답 위치 후보가 있는데 다른 박스가 선택됨
  F9  appearance 거절      정답 위치 후보가 선택됐으나 appe 게이트 탈락
  F8  mask 실패            정답 위치 후보인데 mask 가 사실상 비어 있음
  F12 unresolved           위 증거로 확정 불가

산출: results/fn345_stage_breakdown.csv, fn345_by_object.csv, correct_prompt_vs_any_prompt.csv
"""
import csv, json, os, re
from collections import Counter, defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
COL = os.path.join(RSRCH, "ism_color_validation")
GT = os.path.join(RSRCH, "gt_input")
RES = os.path.join(ROOT, "results")
os.makedirs(RES, exist_ok=True)
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Rabbit", "Dinosaur", "Bear", "Sauce_high", "Mugcup_high",
           "choco_hazelnut_high", "milk", "saffron", "Febreze_high", "Sikhye_high"]
ADJ_W = 5          # 인접 프레임 탐색 폭
ADJ_IOU = 0.3      # 위치 일치로 볼 IoU

# ---------------------------------------------------------------- 적재
O = np.load(os.path.join(COL, "results", "_outcomes.npy"), allow_pickle=True)[0]
key = lambda s: (s.split("|")[0], int(s.split("|")[1]), s.split("|")[2])
FN_A = {k for k, v in O["A"].items() if v[0] == "FN"}
FN_B = {k for k, v in O["B"].items() if v[0] == "FN"}
NEW_HSV = FN_B - FN_A
assert len(FN_A) == 334 and len(FN_B) == 345 and len(NEW_HSV) == 11

tri = {(r["dataset"], int(r["frame_id"]), r["object"]): r
       for r in csv.DictReader(open(os.path.join(GT, "triage_answers.csv"), encoding="utf-8"))}
prov = {r["uid"]: r["true_class"]
        for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}

coord, cand, accepted_all, yolo = {}, defaultdict(list), defaultdict(dict), {}
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_boxes.csv"))):
        coord[r["uid"]] = (int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"]))
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) != 1:
            continue
        k = (ds, int(r["frame_id"]), r["object"])
        cand[k].append({"uid": r["uid"], "sem": float(r["sem_top5"]),
                        "appe": float(r["appe11_clstop1"]),
                        "sim_thr": float(r["sim_thr"]), "appe_gate": float(r["appe_gate"])})
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_yolo.csv"))):
        yolo[(ds, int(r["frame_id"]), r["object"])] = {
            k2: (float(r[k2]) if "conf" in k2 else int(r[k2]))
            for k2 in ("raw_candidates", "raw_max_conf", "post_nms_candidates",
                       "post_max_conf", "after_score_threshold", "topk_candidates")}
# 전체 2,596 프레임의 운영 수락 (인접 프레임 근거용)
for k, v in cand.items():
    b = max(v, key=lambda c: c["sem"])
    if b["sem"] >= b["sim_thr"] and b["appe"] >= b["appe_gate"]:
        accepted_all[(k[0], k[2])][k[1]] = b["uid"]

mstat = {}
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_boxes.csv"))):
        mstat[r["uid"]] = (float(r["mask_bbox_ratio"] or 0), int(r["n_components"] or 0),
                           int(r["mask_area"] or 0))


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    u = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / u if u > 0 else 0.0


# ---------------------------------------------------------------- E1 메모 파싱
NEG = ["들어오지 못", "들어오지못", "후보에 없", "포함되지 않", "선정되지 않", "선정되지 안",
       "후보에 들어오지 않", "후보에도 포함되지", "후보에 들어가지", "후보에 안", "들어오지 않"]
POS = ["도 선정", "도 포함", "도 들어와", "도 존재", "도 잘 선정", "후보에 있", "일부도 선정",
       "일부도 포함", "일부가 포함", "일부가 잘", "일부는 선정", "실제 객체인데 탈락"]


def memo_verdict(note):
    if not note:
        return None
    hn = any(p in note for p in NEG); hp = any(p in note for p in POS)
    if hn and hp:
        return "mixed"
    return "no_correct_cand" if hn else "has_correct_cand" if hp else None


# ---------------------------------------------------------------- E3 인접 프레임
def adjacency(ds, fr, ob):
    """f±W 안에서 그 객체가 수락된 프레임의 박스를 찾아, 현재 후보와의 최대 IoU 를 낸다."""
    ref = accepted_all.get((ds, ob), {})
    near = [(abs(f2 - fr), f2) for f2 in ref if 0 < abs(f2 - fr) <= ADJ_W]
    if not near:
        return None, None
    near.sort()
    rb = coord[ref[near[0][1]]]
    best, bu = 0.0, None
    for c in cand.get((ds, fr, ob), []):
        v = iou(coord[c["uid"]], rb)
        if v > best:
            best, bu = v, c["uid"]
    return best, bu


# ---------------------------------------------------------------- 분류
rows = []
for s in sorted(FN_B):
    ds, fr, ob = key(s)
    t = tri.get((ds, fr, ob), {})
    v = t.get("verdict", "")
    note = t.get("note", "").strip()
    cs = cand.get((ds, fr, ob), [])
    y = yolo.get((ds, fr, ob), {})
    mv = memo_verdict(note)
    aiou, auid = adjacency(ds, fr, ob)
    labs = [prov[c["uid"]] for c in cs if c["uid"] in prov]
    prov_correct = ob in labs
    prov_any = bool(labs)

    stage, ev, detail = "", "", ""
    if s in NEW_HSV:
        stage, ev = "F10", "HSV gate"
        detail = "baseline TP → HSV 거절"
    elif v == "N":
        stage, ev = "F11", "E1 triage"
    elif v == "B":
        stage, ev = "F1", "E1 triage"
    elif not cs:
        if y.get("raw_candidates", 0) == 0:
            stage = "F2"
        elif y.get("post_nms_candidates", 0) == 0:
            stage = "F3"
        elif y.get("after_score_threshold", 0) == 0:
            stage = "F4"
        elif y.get("topk_candidates", 0) == 0:
            stage = "F6"
        else:
            stage = "F12"
        ev = "yolo.csv 단계통계"
        detail = (f"raw={y.get('raw_candidates')} post={y.get('post_nms_candidates')} "
                  f"thr={y.get('after_score_threshold')} topk={y.get('topk_candidates')}")
    else:
        # 후보는 있다 — 정답 위치인가?
        correct = None
        if mv == "no_correct_cand":
            correct, ev = False, "E1 메모"
        elif mv == "has_correct_cand":
            correct, ev = True, "E1 메모"
        elif prov_correct:
            correct, ev = True, "E2 provisional 라벨"
        elif prov_any and not prov_correct:
            correct, ev = False, "E2 provisional 라벨(전부 타 객체)"
        elif aiou is not None and aiou >= ADJ_IOU:
            correct, ev = True, f"E3 인접프레임 IoU={aiou:.2f}"
        elif aiou is not None and aiou < 0.05:
            correct, ev = False, f"E3 인접프레임 IoU={aiou:.2f}"
        else:
            ev = "근거 부족"
        detail = (f"후보 {len(cs)}개, memo={mv}, prov={labs or '-'}, "
                  f"adjIoU={'-' if aiou is None else round(aiou,3)}")
        if correct is False:
            stage = "F5"
        elif correct is True:
            b = max(cs, key=lambda c: c["sem"])
            mr, nc, ma = mstat.get(b["uid"], (0, 0, 0))
            if ma < 50:
                stage = "F8"
            elif b["sem"] < b["sim_thr"]:
                stage = "F7"
            elif b["appe"] < b["appe_gate"]:
                stage = "F9"
            else:
                stage = "F7"
        else:
            stage = "F12"

    rows.append({"dataset": ds, "frame_id": fr, "object": ob, "stage": stage,
                 "evidence": ev, "triage_verdict": v, "memo_verdict": mv or "",
                 "n_candidate": len(cs), "prov_labels": ";".join(labs),
                 "adj_iou": "" if aiou is None else round(aiou, 3),
                 "raw_candidates": y.get("raw_candidates", ""),
                 "post_nms": y.get("post_nms_candidates", ""),
                 "after_thr": y.get("after_score_threshold", ""),
                 "topk": y.get("topk_candidates", ""),
                 "detail": detail, "note": note})

with open(os.path.join(RES, "fn345_stage_breakdown.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

LB = {"F1": "검출 난이도(아주 일부만 보임)", "F2": "raw YOLO 부재",
      "F3": "confidence/NMS 제거", "F4": "YOLO threshold 제거",
      "F5": "wrong-location proposal", "F6": "top_k 제거",
      "F7": "semantic 선택 실패", "F8": "mask 실패", "F9": "appearance 거절",
      "F10": "HSV 신규 거절", "F11": "GT/라벨 오류", "F12": "unresolved"}
c = Counter(r["stage"] for r in rows)
print(f"=== FN 345 단계 분해")
for k in ("F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12"):
    if c[k]:
        print(f"  {k:4s} {LB[k]:28s} {c[k]:>4}  {c[k]/345:>6.1%}")
print(f"  {'합계':33s} {sum(c.values()):>4}")

# 객체별
byo = defaultdict(Counter)
for r in rows:
    byo[r["object"]][r["stage"]] += 1
ks = [k for k in ("F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12") if c[k]]
with open(os.path.join(RES, "fn345_by_object.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["object", "FN_total"] + ks)
    for o in OBJECTS:
        w.writerow([o, sum(byo[o].values())] + [byo[o][k] for k in ks])
print(f"\n=== 객체별 FN 원인 분포")
print(f"{'객체':22s} {'FN':>4} " + " ".join(f"{k:>5}" for k in ks))
for o in OBJECTS:
    print(f"{o:22s} {sum(byo[o].values()):>4} " + " ".join(f"{byo[o][k]:>5}" for k in ks))

# A/B 비율: 해당 프롬프트가 정답 위치를 제안했는가 vs 다른 프롬프트 포함
res = [r for r in rows if r["stage"] in ("F5", "F7", "F8", "F9")]
nA = sum(1 for r in res if r["stage"] != "F5")
print(f"\n=== 정답 위치 후보 존재 여부 (확정된 {len(res)}건 기준)")
print(f"  A. 해당 객체 프롬프트가 정답 위치 제안: {nA} / {len(res)} = {nA/max(len(res),1):.1%}")

# B: 다른 프롬프트까지 포함하면? — 프레임 내 모든 박스 중 정답 위치가 있는지
allbox = defaultdict(list)
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_boxes.csv"))):
        allbox[(ds, int(r["frame_id"]))].append(r["uid"])
nB = 0
bdet = []
for r in rows:
    if r["stage"] not in ("F2", "F3", "F4", "F5", "F6"):
        continue
    ds, fr, ob = r["dataset"], r["frame_id"], r["object"]
    ok = False
    for u in allbox.get((ds, fr), []):
        if prov.get(u) == ob:
            ok = True; break
    if not ok:
        ref = accepted_all.get((ds, ob), {})
        near = sorted([(abs(f2 - fr), f2) for f2 in ref if 0 < abs(f2 - fr) <= ADJ_W])
        if near:
            rb = coord[ref[near[0][1]]]
            ok = any(iou(coord[u], rb) >= ADJ_IOU for u in allbox.get((ds, fr), []))
    nB += ok
    bdet.append({"dataset": ds, "frame_id": fr, "object": ob, "stage": r["stage"],
                 "any_prompt_has_correct_location": int(ok)})
den = len(bdet)
print(f"  B. 다른 프롬프트까지 포함하면 정답 위치 존재: {nB} / {den} = {nB/max(den,1):.1%}")
print(f"     (대상 = proposal 관련 단계 F2/F3/F4/F5/F6)")
with open(os.path.join(RES, "correct_prompt_vs_any_prompt.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(bdet[0].keys())); w.writeheader(); w.writerows(bdet)

prop = sum(c[k] for k in ("F2", "F3", "F4", "F5", "F6"))
ism = sum(c[k] for k in ("F7", "F8", "F9", "F10"))
print(f"\n=== 큰 갈래")
print(f"  proposal 관련 (F2~F6)  {prop:>4}  {prop/345:>6.1%}")
print(f"  ISM 판별 관련 (F7~F10) {ism:>4}  {ism/345:>6.1%}")
print(f"  난이도/라벨 (F1,F11)   {c['F1']+c['F11']:>4}  {(c['F1']+c['F11'])/345:>6.1%}")
print(f"  unresolved (F12)      {c['F12']:>4}  {c['F12']/345:>6.1%}")
json.dump({"stage_counts": dict(c), "A_correct_prompt": nA, "A_den": len(res),
           "B_any_prompt": nB, "B_den": den}, open(os.path.join(RES, "fn345_summary.json"), "w"),
          indent=2, ensure_ascii=False)
print(f"\n-> {RES}")
