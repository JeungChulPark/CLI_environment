#!/usr/bin/env python3
"""eval_honest.py — 순환 논리를 제거한 proposal 평가 (READ-ONLY).

⚠ 첫 평가(eval_proposal_recall.py)의 결함
   위치 GT 로 쓴 provisional 박스 라벨은 **운영 후보 박스에 붙인 라벨**이다.
   따라서 현행(cur) 이 그 위치를 100% 맞추는 것은 순환이며, "정답 위치 recall" 이 아니다.
   이 사실 자체를 결과로 보고한다.

여기서 재는 것 (전부 순환이 아닌 지표)

  M1 재현율 (reproduce)   현행이 찾은 위치를 다른 설정이 재현하는가.
                          cur 은 정의상 1.0 이므로 **비교 대상에서 제외**하고 다른 설정만 본다.
  M2 라우팅 손실          own_prompt recall 과 any_prompt recall 의 격차.
                          박스는 있는데 그 객체의 후보로 라우팅되지 않은 비율. **순환 무관.**
  M3 신규 위치 (novel)    현행 박스 어느 것과도 IoU<0.3 인 박스. "현행이 못 본 위치".
                          정답인지는 알 수 없으므로 **개수만 세고 검수 큐로 넘긴다.**
  M4 FN 프레임 신규 위치  직전 연구에서 '정답 위치 후보 없음' 으로 판정된 (frame,object) 에서
                          각 설정이 만든 신규 위치 박스 수. **개선 가능성의 상한.**
  M5 비용                 프레임당 박스 수, 실행 시간

산출: results/method_comparison.csv(재작성), routing_loss.csv, novel_location.csv,
      recommended_cases.csv
"""
import csv, json, os
from collections import Counter, defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
RESID = os.path.join(RSRCH, "ism_residual_accuracy_research")
GT = os.path.join(RSRCH, "gt_input")
DUMP = os.path.join(ROOT, "candidate_dumps")
RES = os.path.join(ROOT, "results")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
IOU_HIT, IOU_NOVEL, CONF_OP = 0.5, 0.3, 0.02
RUNTIME = {"cur": 38, "cur_1280": 29, "cur_1920": 55, "ml960": 5, "ml960_agn": 3, "yoloe_txt": 5,
           "generic": 29, "ext": 73, "tile2x2": 82}      # 339 프레임 기준 실측(초)


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    u = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / u if u > 0 else 0.0


coord = {}
opbox = defaultdict(list)          # (ds,frame) -> 운영 후보 박스 전부
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_boxes.csv"))):
        b = (int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"]))
        coord[r["uid"]] = b
        opbox[(ds, int(r["frame_id"]))].append(b)
LOC = []
for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv"))):
    if r["true_class"] in OBJECTS and r["uid"] in coord:
        b = coord[r["uid"]]
        LOC.append((r["dataset"], int(r["frame_id"]), r["true_class"], b,
                    (b[2]-b[0]) * (b[3]-b[1])))
gtf = set()
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] == "yes":
        gtf.add((r["dataset_name"], int(r["frame_id"])))
LOC = [x for x in LOC if (x[0], x[1]) in gtf]
areas = sorted(x[4] for x in LOC); SMALL = areas[len(areas) // 3] if areas else 0

# 직전 연구에서 '정답 위치 후보 없음' 으로 판정된 (frame, object)
NOCAND = set()
for r in csv.DictReader(open(os.path.join(RESID, "results", "fn345_stage_breakdown.csv"))):
    if r["stage"] in ("F2", "F3", "F4", "F5", "F6"):
        NOCAND.add((r["dataset"], int(r["frame_id"]), r["object"]))
print(f"위치 GT {len(LOC)} (provisional·순환 주의) / '정답위치 후보 없음' 판정 {len(NOCAND)}건")


def load(cfg):
    d = os.path.join(DUMP, cfg)
    if not os.path.isdir(d):
        return None
    box = defaultdict(list)
    for ds in DATASETS:
        p = os.path.join(d, f"{ds}.csv")
        if not os.path.isfile(p):
            continue
        for r in csv.DictReader(open(p)):
            if float(r["conf"]) < CONF_OP:
                continue
            box[(ds, int(r["frame_id"]))].append(
                (r["target"], r["source"],
                 (int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"])), float(r["conf"])))
    return box


CONFIGS = ["cur", "cur_1280", "cur_1920", "ml960", "ml960_agn", "generic", "ext",
           "tile2x2", "yoloe_txt", "yoloe_pf"]
LABEL = {"cur": "현행 (프롬프트10 × 960)", "cur_1280": "imgsz 1280", "cur_1920": "imgsz 1920",
         "ml960": "multi-label 단일 pass", "ml960_agn": "multi-label + agnostic NMS",
         "generic": "generic 프롬프트 8종", "ext": "확장 프롬프트 (객체당 +2)",
         "tile2x2": "2×2 타일", "yoloe_txt": "YOLOE text prompt", "yoloe_pf": "YOLOE prompt-free"}

BOX, rows, novel_rows = {}, [], []
for cfg in CONFIGS:
    b = load(cfg)
    if b is None:
        print(f"[skip] {cfg}"); continue
    BOX[cfg] = b

# union 조합
for cfgs, name, lb in [(["cur", "ext"], "union_cur_ext", "prompt ensemble (현행+확장)"),
                       (["cur", "generic"], "union_cur_generic", "현행 + generic"),
                       (["cur", "tile2x2"], "union_cur_tile", "현행 + 타일"),
                       (["cur", "yoloe_txt"], "union_cur_yoloe", "현행 + YOLOE")]:
    if all(c in BOX for c in cfgs):
        m = defaultdict(list)
        for c in cfgs:
            for k, v in BOX[c].items():
                m[k].extend(v)
        BOX[name] = m; LABEL[name] = lb

for cfg, box in BOX.items():
    hit_own = hit_any = small_hit = small_n = 0
    per = defaultdict(lambda: [0, 0])
    for ds, fr, ob, b, ar in LOC:
        bs = box.get((ds, fr), [])
        own = [x for x in bs if x[0] == ob]
        o = any(iou(x[2], b) >= IOU_HIT for x in own)
        a = any(iou(x[2], b) >= IOU_HIT for x in bs)
        hit_own += o; hit_any += a
        per[ob][0] += 1; per[ob][1] += o
        if ar < SMALL:
            small_n += 1; small_hit += o
    # M3 신규 위치
    nov = nov_nc = 0
    for (ds, fr), bs in box.items():
        ref = opbox.get((ds, fr), [])
        for tgt, src, b, c in bs:
            if all(iou(b, r) < IOU_NOVEL for r in ref):
                nov += 1
                if (ds, fr, tgt) in NOCAND:
                    nov_nc += 1
                    novel_rows.append({"config": cfg, "dataset": ds, "frame_id": fr,
                                       "object": tgt, "source_prompt": src,
                                       "x1": b[0], "y1": b[1], "x2": b[2], "y2": b[3],
                                       "conf": round(c, 4)})
    n = sum(len(v) for v in box.values())
    rows.append({"config": cfg, "label": LABEL.get(cfg, cfg),
                 "reproduce_own": round(hit_own / max(len(LOC), 1), 4),
                 "reproduce_any": round(hit_any / max(len(LOC), 1), 4),
                 "routing_loss": round((hit_any - hit_own) / max(len(LOC), 1), 4),
                 "reproduce_small": round(small_hit / max(small_n, 1), 4),
                 "novel_boxes": nov, "novel_in_nocand_case": nov_nc,
                 "n_box_total": n, "n_box_per_frame": round(n / max(len(gtf), 1), 1),
                 "runtime_s_339frames": RUNTIME.get(cfg, ""),
                 **{f"own_{o}": round(per[o][1] / max(per[o][0], 1), 3) for o in OBJECTS}})

order = ["cur", "cur_1280", "cur_1920", "ml960", "ml960_agn", "generic", "ext", "tile2x2",
         "yoloe_txt", "yoloe_pf", "union_cur_ext", "union_cur_generic", "union_cur_tile",
         "union_cur_yoloe"]
rows.sort(key=lambda r: order.index(r["config"]) if r["config"] in order else 99)
with open(os.path.join(RES, "method_comparison.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
if novel_rows:
    with open(os.path.join(RES, "novel_location.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(novel_rows[0].keys()))
        w.writeheader(); w.writerows(novel_rows)

print(f"\n=== 설정별 지표 (위치 GT {len(LOC)}건)")
print(f"⚠ reproduce_own 은 '현행이 찾은 위치를 재현하는가' 이지 정답 위치 recall 이 아니다.")
print(f"  cur = 1.0000 은 순환(같은 박스에 붙인 라벨)이므로 비교에서 제외한다.\n")
print(f"{'설정':20s} {'설명':24s} {'재현own':>8} {'재현any':>8} {'라우팅손실':>9} "
      f"{'작은객체':>8} {'신규위치':>8} {'그중 미검출':>10} {'박스/f':>7} {'초':>5}")
for r in rows:
    print(f"{r['config']:20s} {r['label']:24s} {r['reproduce_own']:>8.4f} "
          f"{r['reproduce_any']:>8.4f} {r['routing_loss']:>9.4f} {r['reproduce_small']:>8.4f} "
          f"{r['novel_boxes']:>8} {r['novel_in_nocand_case']:>10} "
          f"{r['n_box_per_frame']:>7.1f} {str(r['runtime_s_339frames']):>5}")

print(f"\n=== 객체별 재현율 (own)")
sel = [r["config"] for r in rows if r["config"] in
       ("cur", "ml960", "ml960_agn", "ext", "yoloe_txt", "cur_1280", "tile2x2")]
print(f"{'객체':22s} " + " ".join(f"{c[:11]:>12s}" for c in sel))
for o in OBJECTS:
    d = {r["config"]: r[f"own_{o}"] for r in rows}
    print(f"{o:22s} " + " ".join(f"{d.get(c, float('nan')):>12.3f}" for c in sel))

with open(os.path.join(RES, "routing_loss.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["config", "reproduce_own", "reproduce_any", "routing_loss", "해석"])
    for r in rows:
        it = ("라우팅 손실 큼 — 박스는 있으나 해당 객체 후보로 안 감"
              if r["routing_loss"] >= 0.05 else "라우팅 손실 없음")
        w.writerow([r["config"], r["reproduce_own"], r["reproduce_any"], r["routing_loss"], it])
print(f"\n-> {RES}")
