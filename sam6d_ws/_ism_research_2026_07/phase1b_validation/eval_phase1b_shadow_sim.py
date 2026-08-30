#!/usr/bin/env python3
"""eval_phase1b_shadow_sim.py — HSV threshold freeze + B2 shadow simulation. READ-ONLY.

Uses the OPERATIONAL authority (ism_hsv, proto from the operational template caches) so the
frozen threshold and the simulated effect match what the runtime shadow will log.

Pipeline: B1 (threshold 0.02, HSV OFF; Phase 1A winner) candidates -> would_hsv_reject =
(B1 accepted) AND (hsv_score < t). B2-sim accept = B1 accept AND hsv >= t. NO decision is
changed at runtime — this only simulates Phase 1C from the shadow scores.

Threshold freeze (no arbitrary averaging):
  - LOO over the 6 datasets: choose t on the other 5 pooled (F1-max on B2-sim), score held-out.
  - Deployment single value = F1-max on ALL 6 pooled (no held-out exists at deploy time).
  - Report 6-fold stability + FP/TP tradeoff + direction consistency + per-object collapse check.

Outputs (phase1b_validation/): fold_freeze.csv, b1b2_summary.csv, b1b2_by_object.csv,
shadow_decisions.csv, frozen_threshold.txt
"""
import csv, os, sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE)
REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
import ism_hsv                       # noqa: E402
import yolo_ism_object_n as o_n      # noqa: E402

GT = os.path.join(RSRCH, "gt_input")
PIPE = os.path.join(RSRCH, "yolo_localization_research", "pipeline", "cur")
OUT = HERE
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
MASKED = slice(0, 224); HS = slice(96, 224)
TOPK = 3

# ---- operational proto (from the caches we just built) ----
defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
PROTO = {}
for o in objs:
    p = os.path.join(os.path.dirname(o["cls_cache"]), f"{o['name']}_hsv.npz")
    proto, meta = ism_hsv.load_cache(p, o["template_dir"])
    assert proto is not None, f"cache invalid for {o['name']}: {meta}"
    PROTO[o["name"]] = proto

# ---- GT ----
gt = {}
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] == "yes":
        gt[(r["dataset_name"], int(r["frame_id"]))] = set(
            t.strip() for t in r["visible_objects"].split(";") if t.strip())

# ---- B1 candidates + hsv score for the selected (accepted-or-not) best ----
# key=(ds,frame,object) -> dict(accept_b1, hsv, box, sem, appe, conf)
rec = {}
for ds in DATASETS:
    z = np.load(os.path.join(PIPE, f"{ds}_hsv.npz"))
    hq = {str(u): h[MASKED][HS].astype(np.float64) for u, h in zip(z["uid"], z["hist"])}
    rows = defaultdict(list)
    for r in csv.DictReader(open(os.path.join(PIPE, f"{ds}_pairs.csv"))):
        if (ds, int(r["frame_id"])) not in gt:
            continue
        rows[(ds, int(r["frame_id"]), r["object"])].append(r)
    for k, cs in rows.items():
        v = [c for c in cs if int(c["routed"]) == 1 and float(c["yolo_conf"]) >= 0.02]
        v.sort(key=lambda c: -float(c["yolo_conf"]))
        v = v[:TOPK]
        if not v:
            continue
        b = max(v, key=lambda c: float(c["sem_top5"]))
        acc = (float(b["sem_top5"]) >= float(b["sim_thr"])) and \
              (float(b["appe11_clstop1"]) >= float(b["appe_gate"]))
        q = hq.get(b["uid"])
        hsv = ism_hsv.similarity(q, PROTO[k[2]]) if q is not None else 1.0
        x1, y1, x2, y2 = (int(t) for t in b["uid"].split("|")[2].split("_"))
        rec[k] = dict(accept_b1=acc, hsv=hsv, box=(x1, y1, x2, y2),
                      sem=float(b["sem_top5"]), appe=float(b["appe11_clstop1"]),
                      conf=float(b["yolo_conf"]))


def score(frames, t):
    TP = FP = FN = 0
    per = defaultdict(lambda: [0, 0, 0])
    for (ds, f) in frames:
        vis = gt[(ds, f)]
        for o in OBJECTS:
            r = rec.get((ds, f, o))
            acc = bool(r and r["accept_b1"] and r["hsv"] >= t)   # B2-sim
            v = o in vis
            if v and acc: TP += 1; per[o][0] += 1
            elif acc: FP += 1; per[o][1] += 1
            elif v: FN += 1; per[o][2] += 1
    return TP, FP, FN, per


def f1_at(frames, t):
    TP, FP, FN, _ = score(frames, t)
    return 2 * TP / max(2 * TP + FP + FN, 1)


# threshold grid = quantiles of hsv over B1-accepted candidates
acc_hsv = sorted(r["hsv"] for r in rec.values() if r["accept_b1"])
GRID = np.unique(np.quantile(acc_hsv, np.linspace(0.0, 0.6, 60)))
allframes = list(gt)

# ---- LOO freeze ----
fold_rows = []
for held in DATASETS:
    trf = [x for x in allframes if x[0] != held]
    tef = [x for x in allframes if x[0] == held]
    t_star = max(GRID, key=lambda t: f1_at(trf, t))
    TP0, FP0, FN0, _ = score(tef, 0.0)          # B1 (no hsv) on held-out
    TP, FP, FN, _ = score(tef, t_star)          # B2-sim on held-out
    fold_rows.append(dict(held_out=held, t_star=round(float(t_star), 5),
                          held_FP_B1=FP0, held_FP_B2=FP, held_TP_B1=TP0, held_TP_B2=TP,
                          dFP=FP - FP0, dTP=TP - TP0,
                          fp_reduced=int(FP < FP0)))
with open(os.path.join(OUT, "fold_freeze.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(fold_rows[0].keys())); w.writeheader(); w.writerows(fold_rows)

# ---- deployment single value = F1-max on all 6 pooled ----
t_deploy = float(max(GRID, key=lambda t: f1_at(allframes, t)))
lo = min(r["t_star"] for r in fold_rows); hi = max(r["t_star"] for r in fold_rows)
dir_consistent = all(r["fp_reduced"] for r in fold_rows)
with open(os.path.join(OUT, "frozen_threshold.txt"), "w") as f:
    f.write(f"deployment_t_hsv={t_deploy:.5f}\n")
    f.write(f"LOO t* range=[{lo:.5f},{hi:.5f}] mean={np.mean([r['t_star'] for r in fold_rows]):.5f}\n")
    f.write(f"held-out FP reduced in all 6 folds={dir_consistent}\n")

print(f"=== threshold freeze ===")
for r in fold_rows:
    print(f"  held {r['held_out']}: t*={r['t_star']:.4f}  FP {r['held_FP_B1']}->{r['held_FP_B2']} "
          f"({r['dFP']:+d})  TP {r['held_TP_B1']}->{r['held_TP_B2']} ({r['dTP']:+d})")
print(f"  LOO t* range [{lo:.4f},{hi:.4f}]  |  FP reduced all folds = {dir_consistent}")
print(f"  >>> DEPLOYMENT t_hsv = {t_deploy:.5f} (F1-max, all 6 pooled)")

# ---- B1 vs B2-sim summary at deployment t ----
def summary(t):
    TP, FP, FN, per = score(allframes, t)
    P = TP / max(TP + FP, 1); R = TP / max(TP + FN, 1)
    return TP, FP, FN, round(P, 4), round(R, 4), round(2 * P * R / max(P + R, 1e-9), 4), per

b1 = summary(0.0); b2 = summary(t_deploy)
with open(os.path.join(OUT, "b1b2_summary.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["config", "t_hsv", "TP", "FP", "FN", "precision", "recall", "f1"])
    w.writerow(["B1 (0.02, HSV OFF)", 0.0, *b1[:6]])
    w.writerow(["B2-sim (0.02 + would_hsv_reject)", round(t_deploy, 5), *b2[:6]])
print(f"\n=== B1 vs B2-sim (deployment t={t_deploy:.4f}, human GT 935) ===")
print(f"  B1     TP {b1[0]} FP {b1[1]} FN {b1[2]}  F1 {b1[5]}")
print(f"  B2-sim TP {b2[0]} FP {b2[1]} FN {b2[2]}  F1 {b2[5]}  (dTP {b2[0]-b1[0]:+d} dFP {b2[1]-b1[1]:+d})")

# ---- per-object + fix/break ----
per1, per2 = b1[6], b2[6]
with open(os.path.join(OUT, "b1b2_by_object.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["object", "B1_TP", "B1_FP", "B1_FN", "B2_TP", "B2_FP", "B2_FN",
                                   "dTP", "dFP", "dFN"])
    for o in OBJECTS:
        a, b = per1[o], per2[o]
        w.writerow([o, *a, *b, b[0]-a[0], b[1]-a[1], b[2]-a[2]])
print("\n=== per-object (B1 -> B2-sim) ===")
for o in OBJECTS:
    a, b = per1[o], per2[o]
    print(f"  {o:22s} TP {a[0]:>3}->{b[0]:>3}  FP {a[1]:>3}->{b[1]:>3}  FN {a[2]:>3}->{b[2]:>3}"
          f"  (dFP {b[1]-a[1]:+d} dTP {b[0]-a[0]:+d})")

# fix (FP removed) vs break (TP removed) counts at deployment t
fixed = broke = 0
for (ds, f, o), r in rec.items():
    if not (r["accept_b1"] and r["hsv"] < t_deploy):
        continue
    vis = o in gt[(ds, f)]
    if vis: broke += 1
    else: fixed += 1
print(f"\n  HSV would-reject: fixed(FP removed)={fixed}  broke(TP removed)={broke}  "
      f"ratio={fixed}:{broke}")

# ---- shadow decisions dump (for viz + parity of would_reject) ----
with open(os.path.join(OUT, "shadow_decisions.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["dataset", "frame_id", "object", "visible", "b1_accept", "hsv_score",
                "t_hsv", "would_hsv_reject", "b2_accept", "sem", "appe", "conf",
                "x1", "y1", "x2", "y2", "is_crossfire_FP"])
    CROSS = {"Bear", "Rabbit", "Dinosaur"}
    for (ds, f, o) in sorted(rec):
        r = rec[(ds, f, o)]
        vis = o in gt[(ds, f)]
        wr = bool(r["accept_b1"] and r["hsv"] < t_deploy)
        b2 = bool(r["accept_b1"] and r["hsv"] >= t_deploy)
        x1, y1, x2, y2 = r["box"]
        w.writerow([ds, f, o, int(vis), int(r["accept_b1"]), round(r["hsv"], 5),
                    round(t_deploy, 5), int(wr), int(b2), round(r["sem"], 4),
                    round(r["appe"], 4), round(r["conf"], 4), x1, y1, x2, y2,
                    int((o in CROSS) and (not vis) and wr)])
print(f"\n-> {OUT}")
