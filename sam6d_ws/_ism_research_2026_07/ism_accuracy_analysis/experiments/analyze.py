#!/usr/bin/env python3
"""analyze.py — 라벨 기반 정확도 분석 + ablation (READ-ONLY, 운영 코드 무관).

입력: features/pairs.csv, features/boxes.csv, labels/box_labels.csv
출력: results/*.json, results/*.csv

정의
  결정 단위 = (object o, box b) 중 b 가 o 의 semantic argmax 로 선택되어 MobileSAM
  까지 도달한 쌍. GT positive = true_class(b) == o.
  라벨 'unclear' 박스는 모든 지표에서 제외한다.

bag 분리 (데이터 누수 방지)
  calib = sam_105314, SAM_loop2, SAM_circle, SAM_occlusion
  eval  = sam_110633, SAM_loop1
"""
import csv, json, os, statistics, sys
from collections import defaultdict, Counter

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RES = os.path.join(ROOT, "results")
os.makedirs(RES, exist_ok=True)

CALIB = {"sam_105314", "SAM_loop2", "SAM_circle", "SAM_occlusion"}
EVAL = {"sam_110633", "SAM_loop1"}

pairs = list(csv.DictReader(open(os.path.join(ROOT, "features", "pairs.csv"))))
boxes = {r["uid"]: r for r in csv.DictReader(open(os.path.join(ROOT, "features", "boxes.csv")))}
labels = {r["uid"]: r["true_class"] for r in csv.DictReader(open(os.path.join(ROOT, "labels", "box_labels.csv")))}

F = ("sem_top1 sem_top3 sem_top5 sem_median sem_softmax sem_mean sem_margin12 "
     "appe11_clstop1 appe11_max42 appe11_mean42 appe11_top5cls appe9_max42 appe9_clstop1 "
     "appe2_max42 appe2_clstop1 appe2_mean42 color_hsv_masked color_hsv_bbox "
     "color_labmom_dist yolo_conf").split()

for r in pairs:
    for k in F:
        r[k] = float(r[k])
    r["sim_thr"] = float(r["sim_thr"]); r["appe_gate"] = float(r["appe_gate"])
    r["is_candidate"] = int(r["is_candidate"])

# ---------------------------------------------------------- 운영 파이프라인 재현
cand = defaultdict(list)
for r in pairs:
    if r["is_candidate"]:
        cand[(r["source"], r["frame"], r["object"])].append(r)

winners, rejected_by_sem = [], []
for k, v in cand.items():
    b = max(v, key=lambda x: x["sem_top5"])
    if b["sem_top5"] < b["sim_thr"]:
        rejected_by_sem.append((b, v))
        continue
    b["_lost"] = [x for x in v if x is not b]
    winners.append(b)

def lab(r):
    return labels.get(r["uid"], None)

def usable(r):
    l = lab(r)
    return l is not None and l != "unclear"

W = [r for r in winners if usable(r)]
print(f"semantic 통과 winner {len(winners)} / 라벨 사용가능 {len(W)}")

# ------------------------------------------------------------ B0 현재 성능
def confusion(rows, decide):
    tp = fp = fn = tn = 0
    per = defaultdict(lambda: [0, 0, 0, 0])
    conf = Counter()
    for r in rows:
        acc = decide(r)
        pos = (lab(r) == r["object"])
        i = 0 if (acc and pos) else 1 if (acc and not pos) else 2 if (not acc and pos) else 3
        per[r["object"]][i] += 1
        if i == 0: tp += 1
        elif i == 1:
            fp += 1
            conf[(lab(r), r["object"])] += 1
        elif i == 2: fn += 1
        else: tn += 1
    return {"TP": tp, "FP": fp, "FN": fn, "TN": tn}, per, conf


def prf(c):
    tp, fp, fn = c["TP"], c["FP"], c["FN"]
    p = tp / (tp + fp) if tp + fp else 0.0
    r = tp / (tp + fn) if tp + fn else 0.0
    return {"precision": round(p, 4), "recall": round(r, 4),
            "f1": round(2*p*r/(p+r), 4) if p+r else 0.0}


B0 = lambda r: r["appe11_clstop1"] >= r["appe_gate"]
c0, per0, conf0 = confusion(W, B0)
print("\n=== B0 현재 운영 (semantic winner 도달분) ===")
print(c0, prf(c0))
print("객체별:", {k: (v, prf({"TP": v[0], "FP": v[1], "FN": v[2], "TN": v[3]}))
                for k, v in sorted(per0.items())})
print("혼동(실제→예측) FP:", conf0.most_common())

# ------------------------------------------------------------ AUROC 유틸
def auroc(pos, neg):
    if not pos or not neg:
        return None
    allv = sorted([(v, 1) for v in pos] + [(v, 0) for v in neg])
    ranks, i, n = {}, 0, len(allv)
    r = [0.0] * n
    while i < n:
        j = i
        while j + 1 < n and allv[j+1][0] == allv[i][0]:
            j += 1
        avg = (i + j) / 2.0 + 1
        for k in range(i, j+1):
            r[k] = avg
        i = j + 1
    rp = sum(r[k] for k in range(n) if allv[k][1] == 1)
    np_, nn = len(pos), len(neg)
    return (rp - np_*(np_+1)/2) / (np_*nn)


# 후보 쌍 전체(=candidate) 기준 점수 변별력
CANDP = [r for r in pairs if r["is_candidate"] and usable(r)]
auc_rows = []
for f in F:
    row = {"feature": f}
    for obj in sorted({r["object"] for r in CANDP}):
        sub = [r for r in CANDP if r["object"] == obj]
        pos = [r[f] for r in sub if lab(r) == obj]
        neg = [r[f] for r in sub if lab(r) != obj]
        a = auroc(pos, neg)
        row[obj] = round(a, 3) if a is not None else None
    vals = [v for k, v in row.items() if k != "feature" and v is not None]
    row["mean"] = round(sum(vals)/len(vals), 3) if vals else None
    row["min"] = min(vals) if vals else None
    auc_rows.append(row)
with open(os.path.join(RES, "auroc_by_feature.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(auc_rows[0].keys())); w.writeheader(); w.writerows(auc_rows)
print("\n=== 후보쌍 AUROC (positive=해당 객체, negative=다른 정체) ===")
for r in auc_rows:
    print(f"{r['feature']:22s} mean={r['mean']} min={r['min']}  " +
          " ".join(f"{k[:6]}={v}" for k, v in r.items() if k not in ("feature", "mean", "min")))

# ------------------------------------------------- carton vs choco 집중 분석
choco = [r for r in CANDP if r["object"] == "choco_hazelnut_high"]
cp = [r for r in choco if lab(r) == "choco_hazelnut_high"]
cn = [r for r in choco if lab(r) == "carton"]
print(f"\n=== choco 후보: 진짜 {len(cp)} vs 갈색택배박스 {len(cn)} ===")
focus = {}
for f in F:
    a = auroc([r[f] for r in cp], [r[f] for r in cn])
    focus[f] = {"auc_choco_vs_carton": round(a, 3) if a else None,
                "choco_med": round(statistics.median([r[f] for r in cp]), 3) if cp else None,
                "carton_med": round(statistics.median([r[f] for r in cn]), 3) if cn else None}
    print(f"{f:22s} AUC={focus[f]['auc_choco_vs_carton']}  choco_med={focus[f]['choco_med']} carton_med={focus[f]['carton_med']}")

# ------------------------------------------------- 인형 3종 상호 혼동 분석
dolls = ["Bear", "Rabbit", "Dinosaur"]
print("\n=== 인형 3종: 각 객체 후보에서 '다른 인형'만 negative 로 ===")
doll_focus = {}
for o in dolls:
    sub = [r for r in CANDP if r["object"] == o and lab(r) in dolls]
    pos = [r for r in sub if lab(r) == o]; neg = [r for r in sub if lab(r) != o]
    doll_focus[o] = {}
    for f in F:
        a = auroc([r[f] for r in pos], [r[f] for r in neg])
        doll_focus[o][f] = round(a, 3) if a else None
    print(f"{o:9s} n_pos={len(pos)} n_neg={len(neg)} " +
          " ".join(f"{f.replace('appe','a')}={doll_focus[o][f]}" for f in
                   ("sem_top5", "appe11_clstop1", "appe11_max42", "appe9_max42", "appe2_max42",
                    "color_hsv_masked")))

json.dump({"B0": {"overall": c0, "prf": prf(c0),
                  "per_object": {k: {"TP": v[0], "FP": v[1], "FN": v[2], "TN": v[3], **prf({"TP": v[0], "FP": v[1], "FN": v[2], "TN": v[3]})}
                                 for k, v in per0.items()},
                  "confusion_true_to_pred": {f"{a}->{b}": n for (a, b), n in conf0.items()}},
           "choco_vs_carton": focus, "dolls": doll_focus},
          open(os.path.join(RES, "baseline_and_focus.json"), "w"), indent=2)
print(f"\n-> {RES}")
