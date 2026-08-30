#!/usr/bin/env python3
"""ablation2.py — frame-object 단위 정식 평가 (분모 고정) + ablation.

ablation.py 의 결함 수정: 변형마다 semantic 선택/사전게이트가 달라지면 "게이트까지
도달한 결정"의 수가 달라져 서로 다른 분모를 비교하게 된다. 여기서는 분모를 고정한다.

평가 단위 = (frame, object) 그룹
  - 후보 박스가 전부 라벨된 그룹만 사용 (라벨 미보유/unclear 박스가 있으면 그룹 제외)
  - GT positive  : 후보 중 true_class == object 인 박스가 존재
  - 예측         : 파이프라인이 그 그룹에서 수락한 박스(최대 1개) — 없으면 '거절'
  - TP: 수락 박스가 실제 그 객체 / FP: 수락 박스가 다른 정체
    FN: positive 인데 미수락 또는 잘못된 박스 수락 / TN: negative 이고 미수락
  → E3(선택 실패)와 E7(게이트 실패)이 같은 분모 안에서 자동으로 드러난다.

calib bags 에서만 threshold 결정, eval bags 에서만 보고.
"""
import csv, json, os
from collections import defaultdict, Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE); RES = os.path.join(ROOT, "results")
CALIB = {"sam_105314", "SAM_loop2", "SAM_circle", "SAM_occlusion"}
EVAL = {"sam_110633", "SAM_loop1"}
DOLLS = {"Bear", "Rabbit", "Dinosaur"}

pairs = list(csv.DictReader(open(os.path.join(ROOT, "features", "pairs.csv"))))
labels = {r["uid"]: r["true_class"] for r in csv.DictReader(open(os.path.join(ROOT, "labels", "box_labels.csv")))}
emb = np.load(os.path.join(ROOT, "features", "box_emb.npz"), allow_pickle=True)
EI = {u: i for i, u in enumerate(emb["uid"])}
P11, PHSV = emb["pmean11"], emb["hsv"]

NUM = ("sem_top1 sem_top3 sem_top5 sem_median sem_mean sem_margin12 appe11_clstop1 appe11_max42 "
       "appe11_mean42 appe9_max42 appe2_max42 appe2_mean42 color_hsv_masked yolo_conf").split()
for r in pairs:
    for k in NUM:
        r[k] = float(r[k])
    r["sim_thr"] = float(r["sim_thr"]); r["appe_gate"] = float(r["appe_gate"])
    r["is_candidate"] = int(r["is_candidate"])
OBJS = sorted({r["object"] for r in pairs})

bybox = defaultdict(dict)
for r in pairs:
    bybox[r["uid"]][r["object"]] = r
for r in pairs:
    o = [v["appe11_clstop1"] for k, v in bybox[r["uid"]].items() if k != r["object"]]
    r["margin_appe"] = r["appe11_clstop1"] - max(o) if o else r["appe11_clstop1"]
    o = [v["sem_mean"] for k, v in bybox[r["uid"]].items() if k != r["object"]]
    r["margin_sem"] = r["sem_mean"] - max(o) if o else r["sem_mean"]

# negative bank (calib bag 라벨 박스 중 해당 객체가 아닌 것)
negb = {}
for o in OBJS:
    idx = [EI[u] for u, l in labels.items() if l not in (o, "unclear") and u in EI
           and any(r["uid"] == u and r["source"] in CALIB for r in pairs[:0]) or False]
negs_uid = defaultdict(list)
for r in pairs:
    if r["source"] in CALIB and r["uid"] in labels and labels[r["uid"]] not in (r["object"], "unclear"):
        negs_uid[r["object"]].append(r["uid"])
for o in OBJS:
    ids = [EI[u] for u in set(negs_uid[o]) if u in EI]
    negb[o] = (P11[ids], PHSV[ids]) if ids else (None, None)
for r in pairs:
    B, H = negb.get(r["object"], (None, None))
    i = EI.get(r["uid"])
    if i is None or B is None or len(B) == 0:
        r["neg_patch"] = r["neg_color"] = 0.0
    else:
        c = B @ P11[i]; hh = np.minimum(H, PHSV[i][None]).sum(1)
        m = c < 0.9999
        r["neg_patch"] = float(c[m].max()) if m.any() else float(c.max())
        r["neg_color"] = float(hh[m].max()) if m.any() else float(hh.max())
    r["disc_patch"] = r["appe11_clstop1"] - r["neg_patch"]
    r["disc_color"] = r["color_hsv_masked"] - r["neg_color"]

# ------------------------------------------------- (frame,object) 그룹 구성
groups = defaultdict(list)
for r in pairs:
    if r["is_candidate"]:
        groups[(r["source"], r["frame"], r["object"])].append(r)
G = {}
for k, v in groups.items():
    if all(labels.get(x["uid"]) not in (None, "unclear") for x in v):
        G[k] = v
print(f"전체 (frame,object) 그룹 {len(groups)} → 전 후보 라벨 완비 {len(G)}")
Gc = {k: v for k, v in G.items() if k[0] in CALIB}
Ge = {k: v for k, v in G.items() if k[0] in EVAL}
print(f"  calib {len(Gc)} / eval {len(Ge)}  (positive: calib "
      f"{sum(1 for k,v in Gc.items() if any(labels[x['uid']]==k[2] for x in v))} / eval "
      f"{sum(1 for k,v in Ge.items() if any(labels[x['uid']]==k[2] for x in v))})")


def run(gs, sel, gate, th):
    """sel: 선택 점수 / gate: 게이트 점수 / th: 객체별 게이트 임계."""
    c = Counter(); per = defaultdict(Counter); conf = Counter(); stage = Counter()
    for (src, fr, o), v in gs.items():
        pos_boxes = [x for x in v if labels[x["uid"]] == o]
        b = max(v, key=sel)
        acc = gate(b) >= th[o]
        if acc and labels[b["uid"]] == o:
            k = "TP"
        elif acc:
            k = "FP"; conf[f'{labels[b["uid"]]}->{o}'] += 1
        elif pos_boxes:
            k = "FN"
            stage["E3_selection" if labels[b["uid"]] != o else "E7_gate"] += 1
        else:
            k = "TN"
        c[k] += 1; per[o][k] += 1
    return c, per, conf, stage


def prf(c):
    tp, fp, fn = c["TP"], c["FP"], c["FN"]
    p = tp/(tp+fp) if tp+fp else 0.0; rr = tp/(tp+fn) if tp+fn else 0.0
    return round(p, 4), round(rr, 4), round(2*p*rr/(p+rr), 4) if p+rr else 0.0


def calib(gs, sel, gate):
    th = {}
    for o in OBJS:
        sub = {k: v for k, v in gs.items() if k[2] == o}
        if not sub:
            th[o] = -1e9; continue
        cands = sorted({round(gate(max(v, key=sel)), 4) for v in sub.values()} | {-1e9})
        best, bt = -1, -1e9
        for t in cands:
            c, *_ = run(sub, sel, gate, {o: t})
            p, r, f1 = prf(c)
            if f1 > best:
                best, bt = f1, t
        th[o] = bt
    return th


EXPS = {}
def add(name, sel, gate, desc, fixed_th=None):
    th = calib(Gc, sel, gate)
    c, per, conf, stage = run(Ge, sel, gate, th)
    p, r, f1 = prf(c)
    e = {"desc": desc, "recalibrated": {**{k: c[k] for k in ("TP", "FP", "FN", "TN")},
         "precision": p, "recall": r, "f1": f1, "confusion": dict(conf.most_common()),
         "FN_stage": dict(stage), "thresholds": {k: round(v, 4) for k, v in th.items()},
         "carton_to_choco": conf.get("carton->choco_hazelnut_high", 0),
         "doll_confusion": sum(n for k, n in conf.items()
                               if k.split("->")[0] in DOLLS and k.split("->")[1] in DOLLS),
         "saffron_to_febreze": conf.get("saffron->Febreze_high", 0)},
         "per_object": {o: dict(per[o]) for o in sorted(per)}}
    line = f"{name:5s} {desc[:46]:46s} | recal P{p:.3f} R{r:.3f} F{f1:.3f} FP{c['FP']:3d} FN{c['FN']:3d}"
    if fixed_th:
        c2, per2, conf2, st2 = run(Ge, sel, gate, fixed_th)
        p2, r2, f2 = prf(c2)
        e["fixed_threshold"] = {**{k: c2[k] for k in ("TP", "FP", "FN", "TN")},
                                "precision": p2, "recall": r2, "f1": f2,
                                "confusion": dict(conf2.most_common()), "FN_stage": dict(st2),
                                "carton_to_choco": conf2.get("carton->choco_hazelnut_high", 0),
                                "doll_confusion": sum(n for k, n in conf2.items()
                                                      if k.split("->")[0] in DOLLS and k.split("->")[1] in DOLLS),
                                "saffron_to_febreze": conf2.get("saffron->Febreze_high", 0)}
        line += f" | fixed P{p2:.3f} R{r2:.3f} FP{c2['FP']:3d} FN{c2['FN']:3d}"
    EXPS[name] = e
    print(line)


S5 = lambda r: r["sem_top5"]
SM = lambda r: r["sem_mean"]
GATE0 = lambda r: r["appe11_clstop1"]
prod_th = {o: next(r["appe_gate"] for r in pairs if r["object"] == o) for o in OBJS}

print("\n=== frame-object 단위 ablation (분모 고정, eval bags) ===")
add("B0", S5, GATE0, "운영: sem_top5 선택 + appe11(CLS-top1) 게이트", fixed_th=prod_th)
add("B1", S5, GATE0, "B0 + 객체별 threshold 재보정")
add("B2", S5, lambda r: r["margin_appe"], "sem_top5 선택 + class margin(appe11)")
add("B2b", S5, lambda r: r["margin_sem"], "sem_top5 선택 + class margin(sem_mean)")
add("B3", S5, lambda r: r["disc_patch"], "sem_top5 선택 + negative prototype(patch)")
add("B3b", S5, lambda r: r["disc_color"], "sem_top5 선택 + negative prototype(color)")
add("B4", S5, lambda r: r["color_hsv_masked"], "sem_top5 선택 + masked HSV color 게이트")
add("B4b", S5, lambda r: 0.5*r["appe11_clstop1"]+0.5*r["color_hsv_masked"], "sem_top5 + (appe11+color)/2")
add("B5", S5, lambda r: r["appe2_max42"], "sem_top5 선택 + block2 게이트")
add("B6", S5, lambda r: 0.5*r["appe11_max42"]+0.5*r["appe2_max42"], "sem_top5 + (block11+block2)/2")
add("B7", SM, GATE0, "sem_mean 선택 + appe11 게이트")
add("B7b", SM, SM, "sem_mean 선택 + sem_mean 게이트")
add("B9", S5, lambda r: max(r["appe11_clstop1"], r["margin_appe"]+0.5), "top-2 완화 근사(선택 실패 보정)")
add("B12", SM, lambda r: 0.5*r["sem_mean"]+0.5*r["margin_sem"], "sem_mean 선택 + (sem_mean+margin)/2")
add("B13", SM, lambda r: (r["sem_mean"]+r["color_hsv_masked"]+r["margin_sem"])/3,
    "sem_mean 선택 + (sem_mean+color+margin)/3")
add("B14", SM, lambda r: (r["sem_mean"]+r["appe11_clstop1"])/2, "sem_mean 선택 + (sem_mean+appe11)/2")
add("B15", SM, lambda r: (r["sem_mean"]+r["margin_sem"]+r["appe11_clstop1"])/3,
    "sem_mean 선택 + (sem_mean+margin+appe11)/3")
add("B16", SM, lambda r: 0.5*r["margin_sem"]+0.5*r["disc_color"], "sem_mean 선택 + (margin+neg-color)/2")

print("\n=== 주요안 객체별 (eval) ===")
for nm in ("B0", "B12", "B13", "B15"):
    e = EXPS[nm]
    print(f"-- {nm} {e['desc']}  conf={e['recalibrated']['confusion']} FNstage={e['recalibrated']['FN_stage']}")
    for o, d in e["per_object"].items():
        print(f"     {o:22s} {dict(d)}")

json.dump({"experiments": EXPS,
           "groups": {"all": len(groups), "fully_labeled": len(G),
                      "calib": len(Gc), "eval": len(Ge)}},
          open(os.path.join(RES, "ablation_frameobject.json"), "w"), indent=2)
print(f"\n-> {RES}/ablation_frameobject.json")

# ---- color 를 '클래스 특화 증거' / 'tie-breaker' 로만 쓰는 변형 ----
CHOCO = "choco_hazelnut_high"
def b17(r):
    s = (r["sem_mean"] + r["margin_sem"] + r["appe11_clstop1"]) / 3
    return s + (0.15 * (r["color_hsv_masked"] - 0.45) if r["object"] == CHOCO else 0.0)
def b18(r):
    s = (r["sem_mean"] + r["margin_sem"] + r["appe11_clstop1"]) / 3
    return s + (0.15 * (r["color_hsv_masked"] - 0.45) if abs(r["margin_sem"]) < 0.05 else 0.0)
def b19(r):
    s = (r["sem_mean"] + r["margin_sem"] + r["appe11_clstop1"]) / 3
    return s + (0.15 * (r["color_hsv_masked"] - 0.45) if r["object"] in DOLLS | {CHOCO} else 0.0)
add("B17", SM, b17, "B15 + color(choco 한정 class-specific)")
add("B18", SM, b18, "B15 + color(margin 불확실 구간 tie-breaker)")
add("B19", SM, b19, "B15 + color(choco+인형 한정)")
add("B20", SM, lambda r: (r["sem_mean"]+r["margin_sem"]+r["appe11_max42"])/3,
    "B15 변형: appe11 을 42-view max 로")
print("\n=== B0 고정임계 상세 ===")
print(EXPS["B0"]["fixed_threshold"])
print("\n=== B15 임계 ===", EXPS["B15"]["recalibrated"]["thresholds"])
json.dump({"experiments": EXPS,
           "groups": {"all": len(groups), "fully_labeled": len(G), "calib": len(Gc), "eval": len(Ge)}},
          open(os.path.join(RES, "ablation_frameobject.json"), "w"), indent=2)
