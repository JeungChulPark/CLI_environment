#!/usr/bin/env python3
"""wrong_location_taxonomy.py — wrong-location 오류를 L1~L10 으로 분해한다 (READ-ONLY).

직전 연구(ism_residual_accuracy_research)의 F5 63건 + 관련 FN/FP 를 대상으로 한다.

⚠ 직전 메모 파서의 결함을 여기서 고친다.
   "식혜·곰인형은 잘못 선정되었지만, **실제 머그컵도 후보에 들어와있음**" 처럼
   부정·긍정 표현이 한 문장에 섞이면 직전 파서는 'mixed' 로 흘려 E3(인접프레임)으로 넘겼다.
   여기서는 **절 단위로 쪼갠 뒤 '실제/진짜' 가 들어간 절**의 극성으로 판정한다.

L 유형
  L1 비슷한 색의 배경 검출         L2 비슷한 형태의 다른 객체
  L3 같은 범주 객체 혼동           L4 작은 실제 객체 대신 큰 주변 객체
  L5 인접 영역 검출                L6 프롬프트가 잘못된 위치에만 반응
  L7 정답 후보도 있으나 오답이 슬롯 차지   L8 모든 프롬프트에서 정답 위치 없음
  L9 NMS/후처리가 정답 후보 제거   L10 판정 불가

산출: results/wrong_location_taxonomy.csv, results/memo_parser_correction.csv
"""
import csv, os, re
from collections import Counter, defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
RESID = os.path.join(RSRCH, "ism_residual_accuracy_research")
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
GT = os.path.join(RSRCH, "gt_input")
RES = os.path.join(ROOT, "results")
os.makedirs(RES, exist_ok=True)
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]

# 객체 범주 (L2/L3 구분용)
CATEG = {"milk": "box", "choco_hazelnut_high": "box",
         "Febreze_high": "bottle", "saffron": "bottle", "Sauce_high": "bottle",
         "Sikhye_high": "can", "Mugcup_high": "cup",
         "Bear": "doll", "Rabbit": "doll", "Dinosaur": "doll"}
KOR = {"소스": "Sauce_high", "식혜": "Sikhye_high", "페브리즈": "Febreze_high",
       "샤프란": "saffron", "우유": "milk", "머그": "Mugcup_high", "머그컵": "Mugcup_high",
       "초코": "choco_hazelnut_high", "과자": "choco_hazelnut_high",
       "곰": "Bear", "토끼": "Rabbit", "공룡": "Dinosaur",
       "갈색 상자": "carton", "갈색상자": "carton", "택배": "carton", "박스": "carton"}

NEG = ["들어오지 못", "들어오지못", "후보에 없", "포함되지 않", "선정되지 않", "선정되지 안",
       "들어오지 않", "후보에 들어가지", "후보에 안", "후보에도 포함되지"]
POS = ["도 선정", "도 포함", "도 들어와", "도 존재", "후보에 있", "일부도 선정", "일부도 포함",
       "일부가 포함", "일부가 잘", "일부는 선정", "도 잘 선정", "도 후보에"]


def clause_polarity(note):
    """절 단위로 쪼갠 뒤 '실제/진짜' 가 들어간 절의 극성으로 판정한다."""
    if not note:
        return None, []
    cls = [c.strip() for c in re.split(r"[,،;·]|그리고|하지만|이며|되었으며|되었지만|되어있지만", note)
           if c.strip()]
    real = [c for c in cls if ("실제" in c or "진짜" in c)]
    tgt = real or cls
    hn = any(p in c for c in tgt for p in NEG)
    hp = any(p in c for c in tgt for p in POS)
    if hn and not hp:
        v = "no_correct_cand"
    elif hp and not hn:
        v = "has_correct_cand"
    elif hn and hp:
        v = "mixed"
    else:
        v = None
    # 오답으로 지목된 객체
    wrong = []
    for c in cls:
        if "잘못" in c or "오" in c[:2]:
            for k, o in KOR.items():
                if k in c:
                    wrong.append(o)
    return v, sorted(set(wrong))


# ---------------------------------------------------------------- 적재
FN = list(csv.DictReader(open(os.path.join(RESID, "results", "fn345_stage_breakdown.csv"))))
coord, cand = {}, defaultdict(list)
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_boxes.csv"))):
        coord[r["uid"]] = (int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"]))
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) == 1:
            cand[(ds, int(r["frame_id"]), r["object"])].append(
                {"uid": r["uid"], "conf": float(r["yolo_conf"]), "sem": float(r["sem_top5"]),
                 "appe": float(r["appe11_clstop1"]), "sim_thr": float(r["sim_thr"]),
                 "appe_gate": float(r["appe_gate"])})
prov = {r["uid"]: r["true_class"]
        for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}
gtvis = {}
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] == "yes":
        gtvis[(r["dataset_name"], int(r["frame_id"]))] = set(
            t.strip() for t in r["visible_objects"].split(";") if t.strip())

# ---------------------------------------------------------------- 메모 파서 정정
corr = []
for r in FN:
    if not r["note"].strip():
        continue
    old = r["memo_verdict"]
    new, wrong = clause_polarity(r["note"])
    if old != (new or ""):
        corr.append({"dataset": r["dataset"], "frame_id": r["frame_id"], "object": r["object"],
                     "old_stage": r["stage"], "old_memo": old, "new_memo": new or "",
                     "note": r["note"][:110]})
with open(os.path.join(RES, "memo_parser_correction.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(corr[0].keys())); w.writeheader(); w.writerows(corr)
print(f"메모 파서 정정: {len(corr)}건이 재판정됨")
print("  변화:", Counter((c["old_memo"] or "-") + "→" + (c["new_memo"] or "-")
                        for c in corr).most_common())
print("  영향받은 기존 단계:", Counter(c["old_stage"] for c in corr).most_common())

# ---------------------------------------------------------------- L 유형 분류
rows = []
for r in FN:
    if r["stage"] not in ("F5", "F12"):
        continue
    ds, fr, ob = r["dataset"], int(r["frame_id"]), r["object"]
    mv, wrong = clause_polarity(r["note"])
    cs = cand.get((ds, fr, ob), [])
    vis = gtvis.get((ds, fr), set())
    n_vis = len(vis)
    # 오답 후보의 크기 (작은 객체 대신 큰 것?)
    areas = [(coord[c["uid"]][2]-coord[c["uid"]][0]) * (coord[c["uid"]][3]-coord[c["uid"]][1])
             for c in cs]
    labs = [prov[c["uid"]] for c in cs if c["uid"] in prov]

    L, ev = "L10", "판정 불가"
    if mv == "has_correct_cand":
        L, ev = "L7", "메모: 정답 후보도 있으나 오답이 슬롯 차지"
    elif wrong:
        same = [w for w in wrong if CATEG.get(w) == CATEG.get(ob)]
        if any(w == "carton" for w in wrong):
            L, ev = "L2", f"메모: 갈색 택배박스가 슬롯 차지"
        elif same:
            L, ev = "L3", f"메모: 같은 범주({CATEG.get(ob)}) 객체 혼동 — {','.join(same)}"
        else:
            L, ev = "L2", f"메모: 다른 범주 객체 검출 — {','.join(wrong)}"
    elif mv == "no_correct_cand" and not cs:
        L, ev = "L8", "메모+후보0: 모든 프롬프트에서 정답 위치 없음"
    elif mv == "no_correct_cand":
        L, ev = "L6", "메모: 프롬프트가 잘못된 위치에만 반응"
    elif "carton" in labs:
        L, ev = "L2", "provisional 라벨: 갈색 택배박스"
    elif labs and ob not in labs:
        same = [l for l in labs if CATEG.get(l) == CATEG.get(ob)]
        L, ev = ("L3", f"provisional: 같은 범주 {','.join(same)}") if same else \
                ("L2", f"provisional: {','.join(labs)}")
    elif r["adj_iou"] not in ("", None) and float(r["adj_iou"]) < 0.05 and cs:
        L, ev = "L6", f"인접프레임 IoU {r['adj_iou']} — 후보는 있으나 전혀 다른 위치"

    rows.append({"dataset": ds, "frame_id": fr, "object": ob,
                 "orig_stage": r["stage"], "L_type": L, "evidence": ev,
                 "memo_verdict_fixed": mv or "", "wrong_objects": ";".join(wrong),
                 "n_candidate": len(cs), "n_visible_in_frame": n_vis,
                 "max_cand_area": max(areas) if areas else "",
                 "min_cand_area": min(areas) if areas else "",
                 "max_conf": round(max((c["conf"] for c in cs), default=0), 4),
                 "prov_labels": ";".join(labs),
                 "confidence_grade": ("confirmed" if r["note"].strip() else
                                      "probable" if (labs or r["adj_iou"] not in ("", None))
                                      else "unresolved"),
                 "note": r["note"]})

with open(os.path.join(RES, "wrong_location_taxonomy.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

LB = {"L1": "비슷한 색의 배경", "L2": "비슷한 형태의 다른 객체", "L3": "같은 범주 객체 혼동",
      "L4": "작은 객체 대신 큰 주변 객체", "L5": "인접 영역", "L6": "프롬프트가 잘못된 위치에만 반응",
      "L7": "정답 후보 있으나 오답이 슬롯 차지", "L8": "모든 프롬프트에서 정답 위치 없음",
      "L9": "NMS/후처리가 정답 제거", "L10": "판정 불가"}
c = Counter(r["L_type"] for r in rows)
print(f"\n=== wrong-location 유형 분해 (F5 63 + F12 33 = {len(rows)}건 대상)")
for k in sorted(c, key=lambda x: -c[x]):
    print(f"  {k:4s} {LB[k]:28s} {c[k]:>4}  {c[k]/len(rows):>6.1%}")
print(f"  신뢰도:", Counter(r["confidence_grade"] for r in rows).most_common())

byo = defaultdict(Counter)
for r in rows:
    byo[r["object"]][r["L_type"]] += 1
ks = sorted(c, key=lambda x: -c[x])
print(f"\n{'객체':22s} {'계':>4} " + " ".join(f"{k:>5}" for k in ks))
for o in sorted(byo, key=lambda o: -sum(byo[o].values())):
    print(f"{o:22s} {sum(byo[o].values()):>4} " + " ".join(f"{byo[o][k]:>5}" for k in ks))

print(f"\n=== 오답으로 지목된 객체 (메모 기준)")
w = Counter(x for r in rows for x in r["wrong_objects"].split(";") if x)
for k, v in w.most_common():
    print(f"  {k:22s} {v}")

print(f"\n=== 프레임당 가시 객체 수별 (작은 객체 가설)")
b = defaultdict(int)
for r in rows:
    n = r["n_visible_in_frame"]
    b["1-2" if n <= 2 else "3-4" if n <= 4 else "5-6" if n <= 6 else "7+"] += 1
for k in ("1-2", "3-4", "5-6", "7+"):
    if b[k]:
        print(f"  가시 {k}: {b[k]:>4}  {b[k]/len(rows):>6.1%}")
print(f"\n-> {RES}")
