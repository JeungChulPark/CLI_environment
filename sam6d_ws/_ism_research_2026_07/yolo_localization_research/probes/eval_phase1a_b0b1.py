#!/usr/bin/env python3
"""eval_phase1a_b0b1.py — Phase 1A regression: per-object threshold (B0) vs uniform 0.02 (B1).

READ-ONLY. HSV is OFF in BOTH configs (Phase 1A does not touch HSV). This isolates the
YOLO score_threshold policy effect only.

Faithful to the operational accept logic (recognize_frame):
  - class routing kept (routed==1 only)
  - per-object score_threshold filter -> top_k=3 -> pick best by sem_top5
  - accept iff  sem_top5 >= similarity_threshold  AND  appe11_clstop1 >= appe_gate
  (NO HSV conjunct)

Candidate source = frozen `pipeline/cur/<ds>_pairs.csv` dumps, generated at conf floor 0.02
(= the shared-YOLO-pass output). Each row carries the object's OWN score_threshold at dump
time (Bear 0.35 / Rabbit 0.30 / Dinosaur 0.30 / others 0.02), so B0 and B1 are both fully
representable from one frozen dump — only the threshold policy differs.

  B0 : conf >= row.score_threshold   (today's per-object guards)
  B1 : conf >= 0.02                  (uniform)

GT = human visible-object GT (935 visible instances), frame-level (same limitation as prior work).

Outputs (phase1a_validation/):
  b0b1_summary.csv, b0b1_by_object.csv, b0b1_candidate_counts.csv, b0b1_decisions.csv
"""
import csv, os
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                       # yolo_localization_research
RSRCH = os.path.dirname(ROOT)                      # _ism_research_2026_07
GT = os.path.join(RSRCH, "gt_input")
PIPE = os.path.join(ROOT, "pipeline", "cur")
OUT = os.path.join(RSRCH, "phase1a_validation")
os.makedirs(OUT, exist_ok=True)
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
TOPK = 3
UNIFORM = 0.02

# ---- GT ----
gt = {}
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] == "yes":
        gt[(r["dataset_name"], int(r["frame_id"]))] = set(
            t.strip() for t in r["visible_objects"].split(";") if t.strip())
NVIS = sum(len([t for t in v if t in OBJECTS]) for v in gt.values())
print(f"GT frames {len(gt)} / visible instances {NVIS}")

# ---- candidates ----
rows = defaultdict(list)      # (ds,frame,object) -> [cand,...]
for ds in DATASETS:
    p = os.path.join(PIPE, f"{ds}_pairs.csv")
    for r in csv.DictReader(open(p)):
        key = (ds, int(r["frame_id"]))
        if key not in gt:
            continue
        rows[(ds, int(r["frame_id"]), r["object"])].append({
            "routed": int(r["routed"]), "conf": float(r["yolo_conf"]),
            "sem": float(r["sem_top5"]), "appe": float(r["appe11_clstop1"]),
            "sim_thr": float(r["sim_thr"]), "appe_gate": float(r["appe_gate"]),
            "sth": float(r["score_threshold"])})


def select(cands, policy):
    thr = None if policy == "B0" else UNIFORM
    v = [c for c in cands if c["routed"] and c["conf"] >= (c["sth"] if thr is None else thr)]
    return sorted(v, key=lambda c: -c["conf"])[:TOPK]


def accept(best):
    # operational gate, HSV OFF
    return (best["sem"] >= best["sim_thr"]) and (best["appe"] >= best["appe_gate"])


def run(policy):
    dec = {}
    for k, cands in rows.items():
        v = select(cands, policy)
        if not v:
            dec[k] = (False, "no-candidate", None)
            continue
        b = max(v, key=lambda c: c["sem"])
        if b["sem"] < b["sim_thr"]:
            dec[k] = (False, "below-sim", b)
        elif b["appe"] < b["appe_gate"]:
            dec[k] = (False, "below-appe", b)
        else:
            dec[k] = (True, "detected", b)
    return dec


def score(dec):
    TP = FP = FN = 0
    per = defaultdict(lambda: [0, 0, 0])
    for (ds, f), vis in gt.items():
        for o in OBJECTS:
            acc = dec.get((ds, f, o), (False, "", None))[0]
            v = o in vis
            if v and acc:
                TP += 1; per[o][0] += 1
            elif acc:
                FP += 1; per[o][1] += 1
            elif v:
                FN += 1; per[o][2] += 1
    return TP, FP, FN, per


DEC = {p: run(p) for p in ("B0", "B1")}
SC = {p: score(DEC[p]) for p in ("B0", "B1")}

# ---- summary ----
summ = []
for p, lab in (("B0", "per-object guard (Bear0.35/Rabbit0.30/Dino0.30)"),
               ("B1", "uniform 0.02")):
    TP, FP, FN, _ = SC[p]
    P = TP / max(TP + FP, 1); R = TP / max(TP + FN, 1)
    summ.append({"config": p, "label": lab, "hsv": "OFF", "TP": TP, "FP": FP, "FN": FN,
                 "precision": round(P, 4), "recall": round(R, 4),
                 "f1": round(2 * P * R / max(P + R, 1e-9), 4)})
with open(os.path.join(OUT, "b0b1_summary.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(summ[0].keys())); w.writeheader(); w.writerows(summ)

b0, b1 = summ[0], summ[1]
print("\n=== B0 vs B1 (HSV OFF, human GT {} visible, 6-dataset pooled) ===".format(NVIS))
for s in summ:
    print(f"{s['config']}  {s['label']:48s} TP {s['TP']:>3} FP {s['FP']:>3} FN {s['FN']:>3} "
          f"P {s['precision']:.4f} R {s['recall']:.4f} F1 {s['f1']:.4f}")
print(f"Δ (B1-B0): TP {b1['TP']-b0['TP']:+d}  FP {b1['FP']-b0['FP']:+d}  FN {b1['FN']-b0['FN']:+d}  "
      f"F1 {b1['f1']-b0['f1']:+.4f}")

# ---- per-object ----
with open(os.path.join(OUT, "b0b1_by_object.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["object", "B0_TP", "B0_FP", "B0_FN", "B1_TP", "B1_FP", "B1_FN",
                "dTP", "dFP", "dFN"])
    p0, p1 = SC["B0"][3], SC["B1"][3]
    for o in OBJECTS:
        a, b = p0[o], p1[o]
        w.writerow([o, *a, *b, b[0]-a[0], b[1]-a[1], b[2]-a[2]])
print("\n=== per-object (B0 -> B1) TP/FP/FN ===")
for o in OBJECTS:
    a, b = SC["B0"][3][o], SC["B1"][3][o]
    flag = "  <== recovered" if b[0] > a[0] else ""
    print(f"  {o:22s} {a[0]:>3}/{a[1]:>3}/{a[2]:>3}  ->  {b[0]:>3}/{b[1]:>3}/{b[2]:>3}"
          f"  (dTP {b[0]-a[0]:+d} dFP {b[1]-a[1]:+d} dFN {b[2]-a[2]:+d}){flag}")

# ---- candidate counts before/after threshold (per object, routed only) ----
cc = defaultdict(lambda: {"before": 0, "after_B0": 0, "after_B1": 0})
for (ds, fr, o), cands in rows.items():
    rc = [c for c in cands if c["routed"]]
    cc[o]["before"] += len(rc)
    cc[o]["after_B0"] += len([c for c in rc if c["conf"] >= c["sth"]])
    cc[o]["after_B1"] += len([c for c in rc if c["conf"] >= UNIFORM])
with open(os.path.join(OUT, "b0b1_candidate_counts.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["object", "routed_candidates", "after_B0_threshold", "after_B1_threshold",
                "recovered_candidates_B1_minus_B0"])
    for o in OBJECTS:
        c = cc[o]
        w.writerow([o, c["before"], c["after_B0"], c["after_B1"], c["after_B1"] - c["after_B0"]])
print("\n=== candidate counts (routed) after threshold: B0 -> B1 ===")
for o in ("Bear", "Rabbit", "Dinosaur"):
    c = cc[o]
    print(f"  {o:10s} routed {c['before']:>4} | after B0 {c['after_B0']:>4} | after B1 {c['after_B1']:>4}"
          f"  (+{c['after_B1']-c['after_B0']} candidates survive)")

# ---- per-decision dump (for visualization frame selection) ----
with open(os.path.join(OUT, "b0b1_decisions.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["dataset", "frame_id", "object", "visible",
                "B0_accept", "B0_reason", "B1_accept", "B1_reason",
                "B1_best_conf", "B1_best_sem", "B1_best_appe",
                "B0_best_conf", "recovered_FN_to_TP"])
    for k in sorted(rows):
        ds, fr, o = k
        vis = o in gt[(ds, fr)]
        a0, r0, b0d = DEC["B0"][k]
        a1, r1, b1d = DEC["B1"][k]
        rec = (vis and (not a0) and a1)
        w.writerow([ds, fr, o, int(vis), int(a0), r0, int(a1), r1,
                    round(b1d["conf"], 4) if b1d else "",
                    round(b1d["sem"], 4) if b1d else "",
                    round(b1d["appe"], 4) if b1d else "",
                    round(b0d["conf"], 4) if b0d else "",
                    int(rec)])
print(f"\n-> {OUT}")
