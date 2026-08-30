#!/usr/bin/env python3
"""analyze_changes.py — baseline 대비 판정 변화·혼동쌍 변화·객체별 AUROC 를 낸다 (READ-ONLY).

산출
  results/changed_decision_cases.csv   판정이 바뀐 모든 (frame,object) 사례
  results/fixed_errors.csv             baseline 오답 → HSV 정답
  results/newly_broken_cases.csv       baseline 정답 → HSV 오답
  results/confusion_pair_changes.csv   주요 혼동쌍 FP 변화
  results/score_auroc_by_object.csv    sem/appe/hsv/fused 의 객체별 AUROC·PR-AUC (provisional 라벨)
"""
import csv, json, os
from collections import defaultdict

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
FUS = os.path.join(RSRCH, "ism_fusion_research")
GT = os.path.join(RSRCH, "gt_input")
RES = os.path.join(ROOT, "results")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]

OUT = np.load(os.path.join(RES, "_outcomes.npy"), allow_pickle=True)[0]
MODES = ["A", "B", "C", "D", "E", "F", "G"]

gt = {}
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] == "yes":
        toks = [t.strip() for t in r["visible_objects"].split(";") if t.strip()]
        gt[(r["dataset_name"], int(r["frame_id"]))] = set(t for t in toks if t in OBJECTS)
prov = {r["uid"]: r["true_class"]
        for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}
tri = {(r["dataset"], int(r["frame_id"]), r["object"]): r["verdict"]
       for r in csv.DictReader(open(os.path.join(GT, "triage_answers.csv"), encoding="utf-8"))}


def key(s):
    a, b, c = s.split("|")
    return (a, int(b), c)


# ---------------------------------------------------------------- 판정 변화
BASE, TARGET = "A", os.environ.get("TARGET", "B")
rows, fixed, broken = [], [], []
for s, (o_a, uid_a, hsv_a) in OUT[BASE].items():
    if s not in OUT[TARGET]:
        continue
    o_t, uid_t, hsv_t = OUT[TARGET][s]
    if o_a == o_t:
        continue
    ds, fr, ob = key(s)
    ok = lambda x: x in ("TP", "TN")
    rec = {"dataset": ds, "frame_id": fr, "object": ob,
           "baseline_outcome": o_a, "hsv_outcome": o_t,
           "direction": "fixed" if (not ok(o_a) and ok(o_t)) else
                        "broken" if (ok(o_a) and not ok(o_t)) else "other",
           "selected_uid_baseline": uid_a, "selected_uid_hsv": uid_t,
           "hsv_score_baseline_sel": round(float(hsv_a), 4) if hsv_a != "" else "",
           "provisional_label": prov.get(uid_a, ""),
           "gt_visible": int(ob in gt.get((ds, fr), set())),
           "triage_verdict": tri.get((ds, fr, ob), "")}
    rows.append(rec)
    (fixed if rec["direction"] == "fixed" else broken if rec["direction"] == "broken"
     else rows).append(rec) if rec["direction"] in ("fixed", "broken") else None

for p, R in ((f"changed_decision_cases.csv", rows), ("fixed_errors.csv", fixed),
             ("newly_broken_cases.csv", broken)):
    with open(os.path.join(RES, p), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(R)

print(f"=== baseline(A) → {TARGET} 판정 변화 {len(rows)}건")
print(f"  수정된 오류(fixed)  {len(fixed)}   "
      f"{dict((k, sum(1 for r in fixed if r['baseline_outcome']==k)) for k in ('FP','FN'))}")
print(f"  새로 깨진 것(broken) {len(broken)}   "
      f"{dict((k, sum(1 for r in broken if r['hsv_outcome']==k)) for k in ('FP','FN'))}")
print(f"  객체별 fixed :", dict(sorted(
    ((o, sum(1 for r in fixed if r['object'] == o)) for o in OBJECTS), key=lambda x: -x[1])))
print(f"  객체별 broken:", dict(sorted(
    ((o, sum(1 for r in broken if r['object'] == o)) for o in OBJECTS), key=lambda x: -x[1])))

# ---------------------------------------------------------------- 혼동쌍 변화
PAIRS = [("saffron", "Febreze_high"), ("Febreze_high", "saffron"),
         ("Bear", "Dinosaur"), ("Bear", "Rabbit"), ("Dinosaur", "Bear"),
         ("Dinosaur", "Rabbit"), ("Rabbit", "Bear"), ("Rabbit", "Dinosaur"),
         ("choco_hazelnut_high", "milk"), ("choco_hazelnut_high", "Bear"),
         ("Sauce_high", "Sikhye_high"), ("Sikhye_high", "Sauce_high")]
def fp_pairs(mode):
    c = defaultdict(int)
    for s, (o, uid, _) in OUT[mode].items():
        if o != "FP":
            continue
        ds, fr, ob = key(s)
        for b in gt.get((ds, fr), set()):
            c[(ob, b)] += 1
        if not gt.get((ds, fr)):
            c[(ob, "(아무 객체도 안 보임)")] += 1
    return c


ca, ct = fp_pairs(BASE), fp_pairs(TARGET)
allp = sorted(set(list(ca) + list(ct)), key=lambda p: -(ca[p] + ct[p]))
with open(os.path.join(RES, "confusion_pair_changes.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["accepted_object", "actually_visible", "FP_baseline", f"FP_{TARGET}",
                "delta", "is_key_pair"])
    for p in allp:
        w.writerow([p[0], p[1], ca[p], ct[p], ct[p] - ca[p], int(p in PAIRS)])

print(f"\n=== 주요 혼동쌍 FP 변화 (A → {TARGET})")
print(f"{'수락':22s} {'실제로 보인 것':22s} {'A':>4} {TARGET:>4} {'Δ':>5}")
for p in PAIRS:
    if ca[p] or ct[p]:
        print(f"{p[0]:22s} {p[1]:22s} {ca[p]:>4} {ct[p]:>4} {ct[p]-ca[p]:>+5}")
print(f"\n  그 외 상위 변화")
oth = sorted([p for p in allp if p not in PAIRS], key=lambda p: ct[p] - ca[p])
for p in oth[:5] + oth[-3:]:
    if ca[p] or ct[p]:
        print(f"{p[0]:22s} {p[1]:22s} {ca[p]:>4} {ct[p]:>4} {ct[p]-ca[p]:>+5}")

# ---------------------------------------------------------------- 객체별 AUROC/PR-AUC
def auroc(pos, neg):
    if not len(pos) or not len(neg):
        return None
    p, n = np.asarray(pos, float), np.asarray(neg, float)
    return float(((p[:, None] > n[None, :]).sum() + 0.5 * (p[:, None] == n[None, :]).sum())
                 / (len(p) * len(n)))


def pr_auc(sc, y):
    o = np.argsort(-np.asarray(sc, float)); yy = np.asarray(y)[o]
    tp = np.cumsum(yy); fp = np.cumsum(1 - yy)
    return float(np.sum(np.diff(np.concatenate([[0.0], tp / max(yy.sum(), 1)]))
                        * (tp / np.maximum(tp + fp, 1))))


hist = {}
for ds in DATASETS:
    z = np.load(os.path.join(OBS, "hsv_features", f"{ds}_hsv.npz"))
    for u, h in zip(z["uid"], z["hist"]):
        hist[str(u)] = h[0:224].astype(np.float64)
PC = np.load(os.path.join(FUS, "ply_hsv", "prototype_colors.npz"))


def feat_rgb(rgb):
    img = np.asarray(rgb, np.uint8).reshape(-1, 1, 3)[:, :, ::-1]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.int16)
    hsv[..., 0] = (hsv[..., 0] - 3) % 180
    hsv[..., 1] = np.clip(hsv[..., 1] * 1.3, 0, 255)
    x = cv2.calcHist([hsv.astype(np.uint8)], [0, 1], None, [16, 8], [0, 180, 0, 256]).flatten()
    return x / max(x.sum(), 1e-12)


PROTO = {o: np.stack([feat_rgb(c) for c in PC[f"{o}__render42"]]) for o in OBJECTS}
pairs = []
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) != 1 or r["uid"] not in prov:
            continue
        if prov[r["uid"]] == "unclear":
            continue
        q = hist[r["uid"]][96:224]; q = q / max(q.sum(), 1e-12)
        M = PROTO[r["object"]]
        bc = np.sqrt(np.maximum(q[None, :] * M, 0)).sum(1)
        pairs.append({"object": r["object"], "y": int(prov[r["uid"]] == r["object"]),
                      "sem": float(r["sem_top5"]), "appe": float(r["appe11_clstop1"]),
                      "hsv": float((1 - np.sqrt(np.maximum(0, 1 - bc))).max())})
for p in pairs:
    p["fused"] = (p["sem"] + p["appe"] + p["hsv"]) / 3

with open(os.path.join(RES, "score_auroc_by_object.csv"), "w", newline="") as f:
    ks = ["object", "n_pos", "n_neg"] + [f"{m}_{s}" for s in ("sem", "appe", "hsv", "fused")
                                          for m in ("auroc", "prauc")]
    w = csv.DictWriter(f, fieldnames=ks); w.writeheader()
    print(f"\n=== 객체별 AUROC (provisional 박스라벨 — 사람 GT 아님)")
    print(f"{'객체':22s} {'pos':>4} {'neg':>4} " + " ".join(
        f"{s:>8s}" for s in ("sem", "appe", "hsv", "fused")))
    for o in OBJECTS:
        P = [p for p in pairs if p["object"] == o]
        pos = [p for p in P if p["y"]]; neg = [p for p in P if not p["y"]]
        if len(pos) < 2 or len(neg) < 2:
            continue
        row = {"object": o, "n_pos": len(pos), "n_neg": len(neg)}
        for s in ("sem", "appe", "hsv", "fused"):
            row[f"auroc_{s}"] = round(auroc([p[s] for p in pos], [p[s] for p in neg]), 4)
            row[f"prauc_{s}"] = round(pr_auc([p[s] for p in P], [p["y"] for p in P]), 4)
        w.writerow(row)
        print(f"{o:22s} {len(pos):>4} {len(neg):>4} " + " ".join(
            f"{row['auroc_'+s]:>8.4f}" for s in ("sem", "appe", "hsv", "fused")))
print(f"\n-> {RES}")
