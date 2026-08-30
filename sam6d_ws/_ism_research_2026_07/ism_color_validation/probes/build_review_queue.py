#!/usr/bin/env python3
"""build_review_queue.py — 최소 사람 검수 큐를 만든다 (READ-ONLY, 신규 대규모 라벨링 없음).

무엇을 묻는가
  프레임 단위 GT 로는 **수락된 박스가 실제로 그 객체 위에 있었는지** 확인할 수 없다.
  이번 결론(HSV 게이트가 FP 139건을 제거)이 진짜인지 확정하려면 그 박스 안에 무엇이
  들어 있었는지만 알면 된다. 그래서 박스 하나당 질문 하나만 던진다.

    "이 박스 안에 실제로 있는 것은?"  →  객체 이름 / hard_negative / wrong_location / unsure

  BBox 를 새로 그리게 하지 않는다.

선정 (지시된 6개 기준, 중복 제거 후 상한 적용)
  P1 baseline 정답 → HSV 오답            (전수, 가장 중요)
  P2 baseline 오답 → HSV 정답            (객체별 층화 표본)
  P3 HSV 방식끼리 결론이 다른 사례        (B vs E vs G 불일치)
  P4 HSV 점수가 게이트 임계 부근          (|hsv - t| 최소)
  P5 주요 confusion pair                 (위에서 안 뽑힌 것)
  P6 provisional 라벨 신뢰도 낮은 사례    (기존 human_review_queue 79건과 교집합)

산출
  human_review/color_review_queue.csv
  human_review/crops/<case_id>.png        선택 박스를 표시한 프레임
  human_review/contact_sheets/*.jpg
  human_review/USER_REVIEW_GUIDE.md 는 별도 작성
"""
import csv, os, random
from collections import defaultdict

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
GT = os.path.join(RSRCH, "gt_input")
RES = os.path.join(ROOT, "results")
HR = os.path.join(ROOT, "human_review")
os.makedirs(os.path.join(HR, "crops"), exist_ok=True)
os.makedirs(os.path.join(HR, "contact_sheets"), exist_ok=True)
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
CAP = {"P2": 40, "P3": 30, "P4": 30, "P5": 20, "P6": 20}
rng = random.Random(0)

OUTC = np.load(os.path.join(RES, "_outcomes.npy"), allow_pickle=True)[0]
coord, box_of = {}, {}
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_boxes.csv"))):
        coord[r["uid"]] = (int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"]))
prov = {r["uid"]: r["true_class"]
        for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}
lowconf = set()
p = os.path.join(OBS, "labels", "human_review_queue.csv")
if os.path.isfile(p):
    for r in csv.DictReader(open(p)):
        lowconf.add(r.get("uid", ""))
gtvis = {}
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] == "yes":
        gtvis[(r["dataset_name"], int(r["frame_id"]))] = set(
            t.strip() for t in r["visible_objects"].split(";") if t.strip())


def key(s):
    a, b, c = s.split("|")
    return (a, int(b), c)


cands = {}
for s, (o, uid, hsv) in OUTC["A"].items():
    cands[s] = {"A": (o, uid, hsv)}
for m in ("B", "C", "D", "E", "F", "G"):
    for s, v in OUTC[m].items():
        cands.setdefault(s, {})[m] = v

sel = {}                       # (ds,frame,uid) -> case dict


def add(s, reason, uid=None, extra=None):
    ds, fr, ob = key(s)
    u = uid or cands[s]["A"][1] or cands[s].get("B", ("", "", ""))[1]
    if not u or u not in coord:
        return
    k = (ds, fr, u)
    if k in sel:
        sel[k]["reasons"].add(reason)
        return
    sel[k] = {"dataset_name": ds, "frame_id": fr, "target_object": ob, "uid": u,
              "bbox": ";".join(map(str, coord[u])), "reasons": {reason},
              "baseline_result": cands[s]["A"][0],
              "hsv_result": cands[s].get("B", ("", "", ""))[0],
              "hsv_score": (round(float(cands[s]["A"][2]), 4)
                            if cands[s]["A"][2] != "" else ""),
              "provisional_label": prov.get(u, ""),
              "gt_visible_in_frame": ";".join(sorted(gtvis.get((ds, fr), set()))) or "none",
              **(extra or {})}


# P1 / P2
byobj_fix = defaultdict(list)
for s, d in cands.items():
    if "B" not in d:
        continue
    a, b = d["A"][0], d["B"][0]
    ok = lambda x: x in ("TP", "TN")
    if ok(a) and not ok(b):
        add(s, "P1_newly_broken")
    elif not ok(a) and ok(b):
        byobj_fix[key(s)[2]].append(s)
n = 0
for o in OBJECTS:
    for s in rng.sample(byobj_fix[o], min(len(byobj_fix[o]), max(2, CAP["P2"] // 10))):
        add(s, "P2_fixed_error"); n += 1

# P3 방식 불일치
dis = [s for s, d in cands.items()
       if len({d[m][0] for m in ("B", "E", "G") if m in d}) > 1]
for s in rng.sample(dis, min(len(dis), CAP["P3"])):
    add(s, "P3_mode_disagree")

# P4 임계 부근 (게이트 임계 중앙값 근처)
THR = 0.123
near = sorted([s for s, d in cands.items() if d["A"][2] != ""],
              key=lambda s: abs(float(cands[s]["A"][2]) - THR))
for s in near[:CAP["P4"]]:
    add(s, "P4_near_threshold")

# P5 주요 혼동쌍
PAIRS = {("saffron", "Febreze_high"), ("Febreze_high", "saffron"), ("Bear", "Dinosaur"),
         ("Bear", "Rabbit"), ("Rabbit", "Dinosaur"), ("Rabbit", "Bear"),
         ("choco_hazelnut_high", "milk"), ("choco_hazelnut_high", "Bear"),
         ("Sauce_high", "Sikhye_high"), ("Sikhye_high", "Sauce_high")}
kp = []
for s, d in cands.items():
    if d["A"][0] != "FP":
        continue
    ds, fr, ob = key(s)
    for b in gtvis.get((ds, fr), set()):
        if (ob, b) in PAIRS:
            kp.append(s); break
for s in rng.sample(kp, min(len(kp), CAP["P5"])):
    add(s, "P5_key_confusion")

# P6 저신뢰 provisional 라벨
lc = [s for s, d in cands.items() if d["A"][1] in lowconf]
for s in rng.sample(lc, min(len(lc), CAP["P6"])):
    add(s, "P6_lowconf_label")

rows = []
for i, (k, v) in enumerate(sorted(sel.items(),
                                  key=lambda t: (0 if "P1_newly_broken" in t[1]["reasons"] else 1,
                                                 t[0])), start=1):
    v["case_id"] = f"C{i:04d}"
    v["reasons"] = "+".join(sorted(v["reasons"]))
    v["priority"] = 1 if "P1" in v["reasons"] else 2
    v["image_path"] = os.path.join(HR, "crops", f"{v['case_id']}.png")
    v["review_label"] = ""; v["reviewed"] = "no"; v["note"] = ""
    rows.append(v)

# ---------------------------------------------------------------- 이미지 생성
byfr = defaultdict(list)
for r in rows:
    byfr[(r["dataset_name"], r["frame_id"])].append(r)
thumbs = defaultdict(list)
for (ds, fr), rs in byfr.items():
    src = os.path.join(GT, "frames", ds, f"frame_{fr:06d}.png")
    if not os.path.isfile(src):
        continue
    base = cv2.imread(src)
    if base is None:
        continue
    for r in rs:
        im = base.copy()
        x1, y1, x2, y2 = map(int, r["bbox"].split(";"))
        cv2.rectangle(im, (x1, y1), (x2, y2), (0, 80, 255), 3)
        cv2.rectangle(im, (0, 0), (im.shape[1], 26), (0, 0, 0), -1)
        cv2.putText(im, f"{r['case_id']}  {ds} f{fr}  target={r['target_object']}",
                    (5, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imwrite(r["image_path"], im)
        t = cv2.resize(im, (420, int(420 * im.shape[0] / im.shape[1])))
        thumbs[ds].append(t)

for ds, ts in thumbs.items():
    per = 9
    for si in range(0, len(ts), per):
        c = ts[si:si + per]
        hh = max(t.shape[0] for t in c); ww = max(t.shape[1] for t in c)
        sheet = np.full((hh * 3, ww * 3, 3), 32, np.uint8)
        for k2, t in enumerate(c):
            rr, cc = divmod(k2, 3)
            sheet[rr*hh:rr*hh+t.shape[0], cc*ww:cc*ww+t.shape[1]] = t
        cv2.imwrite(os.path.join(HR, "contact_sheets", f"{ds}_{si//per:03d}.jpg"), sheet,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])

COLS = ["case_id", "priority", "dataset_name", "frame_id", "target_object", "uid", "bbox",
        "image_path", "baseline_result", "hsv_result", "hsv_score", "reasons",
        "provisional_label", "gt_visible_in_frame", "review_label", "reviewed", "note"]
with open(os.path.join(HR, "color_review_queue.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=COLS, extrasaction="ignore")
    w.writeheader(); w.writerows(rows)

print(f"검수 큐 {len(rows)}건 (중복 제거 후)")
c = defaultdict(int)
for r in rows:
    for x in r["reasons"].split("+"):
        c[x] += 1
for k, v in sorted(c.items()):
    print(f"  {k:22s} {v:>4}")
print(f"  우선순위 1 (새로 깨진 것) {sum(1 for r in rows if r['priority']==1)}")
print(f"  이미지 {len(os.listdir(os.path.join(HR,'crops')))} / "
      f"시트 {len(os.listdir(os.path.join(HR,'contact_sheets')))}")
print(f"-> {os.path.join(HR,'color_review_queue.csv')}")
