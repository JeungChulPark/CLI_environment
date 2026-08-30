#!/usr/bin/env python3
"""eval_proposal_recall.py — 설정별 **정답 위치 proposal recall** 을 측정한다 (READ-ONLY).

후보 개수로 비교하지 않는다. 위치 GT 가 필요하므로 다음을 사용한다.

  위치 GT = provisional 박스 라벨 504건 중 실제 객체로 라벨된 것 (474건, unclear 제외)
           라벨된 박스의 좌표 = 그 객체의 실제 위치로 본다.
  신뢰도  = probable (사람 GT 아님. Claude Vision 컨택트시트 라벨)
  적중    = 어떤 후보 박스가 그 위치와 IoU >= 0.5

측정 지표
  recall_own_prompt   해당 객체의 프롬프트(들)가 만든 박스만으로 적중한 비율
  recall_any_prompt   그 설정의 모든 박스를 합쳐 적중한 비율
  n_box_per_frame     프레임당 박스 수 (비용)
  wrong_loc_rate      해당 객체 프롬프트 박스 중 그 객체 위치와 IoU<0.1 인 비율
  recall_small        작은 객체(면적 하위 1/3)에서의 recall

주의: recall_any_prompt 가 올라도 recall_own_prompt 가 그대로면 **라우팅 문제**이고,
      둘 다 그대로면 **localization 문제**다. 이 구분이 이번 연구의 핵심이다.

산출: results/current_proposal_recall.csv, prompt_experiment.csv,
      prompt_union_experiment.csv, shared_proposal_experiment.csv,
      multi_label_slot_experiment.csv, resolution_tile_experiment.csv, method_comparison.csv
"""
import csv, json, os
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
GT = os.path.join(RSRCH, "gt_input")
DUMP = os.path.join(ROOT, "candidate_dumps")
RES = os.path.join(ROOT, "results")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
IOU_HIT = 0.5
IOU_WRONG = 0.1
CONF_OP = 0.02          # 운영 하한. 설정별로 동일 적용해 공정 비교


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    u = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / u if u > 0 else 0.0


# ---------------------------------------------------------------- 위치 GT
coord = {}
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_boxes.csv"))):
        coord[r["uid"]] = (int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"]))
LOC = []          # (ds, frame, object, bbox, area)
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
areas = sorted(x[4] for x in LOC)
SMALL = areas[len(areas) // 3] if areas else 0
print(f"위치 GT {len(LOC)}건 (provisional, GT 프레임 내) / 작은 객체 기준 면적 < {SMALL:,}")
print(f"  객체별: {dict(sorted(__import__('collections').Counter(x[2] for x in LOC).items()))}")

# ---------------------------------------------------------------- 설정별 박스 적재
def load(cfg):
    if not os.path.isdir(os.path.join(DUMP, cfg)):
        return None
    box = defaultdict(list)          # (ds, frame) -> [(target, source, bbox, conf)]
    for ds in DATASETS:
        p = os.path.join(DUMP, cfg, f"{ds}.csv")
        if not os.path.isfile(p):
            continue
        for r in csv.DictReader(open(p)):
            c = float(r["conf"])
            if c < CONF_OP:
                continue
            box[(ds, int(r["frame_id"]))].append(
                (r["target"], r["source"],
                 (int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"])), c))
    return box


def evaluate(cfg, box, own_map=None, topk=None):
    """own_map: object -> 그 객체의 후보로 인정할 source 집합 (None 이면 target 열 사용)"""
    hit_own = hit_any = 0
    small_n = small_hit = 0
    per = defaultdict(lambda: [0, 0, 0])          # object -> [n, own_hit, any_hit]
    wrong_n = wrong_bad = 0
    for ds, fr, ob, b, ar in LOC:
        bs = box.get((ds, fr), [])
        if own_map is not None:
            own = [x for x in bs if x[1] in own_map.get(ob, set())]
        else:
            own = [x for x in bs if x[0] == ob]
        if topk:
            own = sorted(own, key=lambda x: -x[3])[:topk]
        o_hit = any(iou(x[2], b) >= IOU_HIT for x in own)
        a_hit = any(iou(x[2], b) >= IOU_HIT for x in bs)
        hit_own += o_hit; hit_any += a_hit
        per[ob][0] += 1; per[ob][1] += o_hit; per[ob][2] += a_hit
        if ar < SMALL:
            small_n += 1; small_hit += o_hit
        for x in own:
            wrong_n += 1
            if iou(x[2], b) < IOU_WRONG:
                wrong_bad += 1
    nfr = len(gtf)
    nbox = sum(len(v) for v in box.values())
    return {"config": cfg, "n_loc_gt": len(LOC),
            "recall_own_prompt": round(hit_own / max(len(LOC), 1), 4),
            "recall_any_prompt": round(hit_any / max(len(LOC), 1), 4),
            "recall_small_own": round(small_hit / max(small_n, 1), 4),
            "n_small": small_n,
            "wrong_loc_rate": round(wrong_bad / max(wrong_n, 1), 4),
            "n_box_total": nbox, "n_box_per_frame": round(nbox / max(nfr, 1), 1),
            **{f"own_{o}": round(per[o][1] / max(per[o][0], 1), 3) for o in OBJECTS},
            }, per


CONFIGS = ["cur", "cur_1280", "cur_1920", "ml960", "ml960_agn", "generic", "ext", "tile2x2"]
LABEL = {"cur": "현행 (프롬프트10 × imgsz960)", "cur_1280": "imgsz 1280",
         "cur_1920": "imgsz 1920", "ml960": "multi-label 단일 pass",
         "ml960_agn": "multi-label + agnostic NMS", "generic": "generic 프롬프트 8종",
         "ext": "확장 프롬프트 (객체당 +2)", "tile2x2": "2×2 타일 (겹침 0.2)"}

rows, pers = [], {}
BOX = {}
for cfg in CONFIGS:
    b = load(cfg)
    if b is None:
        print(f"[skip] {cfg} 덤프 없음"); continue
    BOX[cfg] = b
    r, per = evaluate(cfg, b)
    r["label"] = LABEL[cfg]
    rows.append(r); pers[cfg] = per

# --- 조합: ext union (현행 + 확장), generic union (현행 + generic), 전부 union
def merge(cfgs, name):
    m = defaultdict(list)
    for c in cfgs:
        if c not in BOX:
            return None
        for k, v in BOX[c].items():
            m[k].extend(v)
    return m


COMBO = [(["cur", "ext"], "union_cur_ext", "prompt ensemble (현행+확장)"),
         (["cur", "generic"], "union_cur_generic", "현행 + generic proposal"),
         (["cur", "tile2x2"], "union_cur_tile", "현행 + 2×2 타일"),
         (["cur", "ext", "generic", "tile2x2"], "union_all", "전부 union")]
for cfgs, name, lb in COMBO:
    m = merge(cfgs, name)
    if m is None:
        continue
    BOX[name] = m
    # own 판정: 확장 프롬프트는 target 열에 객체명이 들어 있으므로 그대로 사용
    r, per = evaluate(name, m)
    r["label"] = lb
    rows.append(r); pers[name] = per

with open(os.path.join(RES, "method_comparison.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

base = next(r for r in rows if r["config"] == "cur")
print(f"\n=== 정답 위치 proposal recall (위치 GT {len(LOC)}건, IoU≥{IOU_HIT}, conf≥{CONF_OP})")
print(f"{'설정':22s} {'설명':26s} {'own':>7} {'any':>7} {'작은객체':>8} "
      f"{'wrong률':>8} {'박스/프레임':>10}")
for r in rows:
    mk = ""
    if r["config"] != "cur":
        d = r["recall_own_prompt"] - base["recall_own_prompt"]
        mk = f"  ({d:+.3f})"
    print(f"{r['config']:22s} {r['label']:26s} {r['recall_own_prompt']:>7.4f} "
          f"{r['recall_any_prompt']:>7.4f} {r['recall_small_own']:>8.4f} "
          f"{r['wrong_loc_rate']:>8.4f} {r['n_box_per_frame']:>10.1f}{mk}")

print(f"\n=== 객체별 own-prompt recall")
show = [r["config"] for r in rows]
print(f"{'객체':22s} " + " ".join(f"{c[:12]:>13s}" for c in show))
for o in OBJECTS:
    print(f"{o:22s} " + " ".join(
        f"{pers[c][o][1]/max(pers[c][o][0],1):>13.3f}" for c in show))

# 개별 산출물
def dump(name, cfgs):
    R = [r for r in rows if r["config"] in cfgs]
    if R:
        with open(os.path.join(RES, name), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(R)


dump("current_proposal_recall.csv", ["cur"])
dump("prompt_experiment.csv", ["cur", "ext"])
dump("prompt_union_experiment.csv", ["cur", "ext", "union_cur_ext"])
dump("shared_proposal_experiment.csv", ["cur", "generic", "union_cur_generic"])
dump("multi_label_slot_experiment.csv", ["cur", "ml960", "ml960_agn"])
dump("resolution_tile_experiment.csv",
     ["cur", "cur_1280", "cur_1920", "tile2x2", "union_cur_tile"])
json.dump({"loc_gt": len(LOC), "small_area_th": SMALL, "iou_hit": IOU_HIT},
          open(os.path.join(RES, "eval_setup.json"), "w"), indent=2)
print(f"\n-> {RES}")
