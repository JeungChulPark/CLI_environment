#!/usr/bin/env python3
"""eval_phase1c.py — Phase 1C operational re-eval. READ-ONLY.

Configs (human GT 935 visible, t=0.1214, operational caches):
  A  baseline (sem+appe, HSV OFF)
  B  HSV active, Dinosaur UNcorrected (hc=0)
  C  HSV active, Dinosaur +9 (operational Phase 1C)   <-- the deployed config
Uses the SAME B1 candidate selection as prior phases. Produces the required CSVs + metrics.json.
"""
import csv, json, os, sys
from collections import defaultdict
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RA = os.path.dirname(HERE); ROOT = os.path.dirname(RA)
RSRCH = os.path.dirname(ROOT); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
import ism_hsv, yolo_ism_object_n as o_n   # noqa

GT = os.path.join(RSRCH, "gt_input")
PIPE = os.path.join(RSRCH, "yolo_localization_research", "pipeline", "cur")
OUT = os.path.join(REPO, "outputs", "phase1c_hsv_dinosaur_color_correction")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
MASKED = slice(0, 224); HS = slice(96, 224); TOPK = 3; T = 0.1214

defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
tdir = {o["name"]: o["template_dir"] for o in objs}
PROTO = {}
for o in objs:
    PROTO[o["name"]] = ism_hsv.load_cache(
        os.path.join(os.path.dirname(o["cls_cache"]), f"{o['name']}_hsv.npz"), o["template_dir"],
        )[0] if o["name"] != "Dinosaur" else None
PROTO["Dinosaur_hc9"] = ism_hsv.build_reference(tdir["Dinosaur"], 9)[0]
PROTO["Dinosaur_hc0"] = ism_hsv.build_reference(tdir["Dinosaur"], 0)[0]

gt = {}
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] == "yes":
        gt[(r["dataset_name"], int(r["frame_id"]))] = set(
            t.strip() for t in r["visible_objects"].split(";") if t.strip())

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
        v.sort(key=lambda c: -float(c["yolo_conf"])); v = v[:TOPK]
        if not v:
            continue
        b = max(v, key=lambda c: float(c["sem_top5"]))
        acc = (float(b["sem_top5"]) >= float(b["sim_thr"])) and (float(b["appe11_clstop1"]) >= float(b["appe_gate"]))
        rec[k] = dict(accept_b1=acc, q=hq.get(b["uid"]), sem=float(b["sem_top5"]),
                      appe=float(b["appe11_clstop1"]), conf=float(b["yolo_conf"]), uid=b["uid"])


def proto_for(o, dino_hc):
    if o == "Dinosaur":
        return PROTO[f"Dinosaur_hc{dino_hc}"]
    return PROTO[o]


def decide(o, r, mode, dino_hc):
    """mode: 'A' none, 'B'/'C' HSV active."""
    if not r["accept_b1"]:
        return False, None
    if mode == "A":
        return True, None
    p = proto_for(o, dino_hc)
    hsv = ism_hsv.similarity(r["q"], p) if r["q"] is not None else 1.0
    return (hsv >= T), hsv


def metrics(mode, dino_hc):
    TP = FP = FN = TN = 0; per = defaultdict(lambda: [0, 0, 0])
    for (ds, f), vis in gt.items():
        for o in OBJECTS:
            r = rec.get((ds, f, o))
            acc = decide(o, r, mode, dino_hc)[0] if r else False
            v = o in vis
            if v and acc: TP += 1; per[o][0] += 1
            elif acc: FP += 1; per[o][1] += 1
            elif v: FN += 1; per[o][2] += 1
            else: TN += 1
    P = TP / max(TP + FP, 1); R = TP / max(TP + FN, 1)
    spec = TN / max(TN + FP, 1); f1 = 2 * P * R / max(P + R, 1e-9)
    return dict(TP=TP, FP=FP, FN=FN, TN=TN, precision=round(P, 4), recall=round(R, 4),
                specificity=round(spec, 4), f1=round(f1, 4),
                balanced_accuracy=round((R + spec) / 2, 4), per=per)


CFG = {"A_baseline_noHSV": ("A", 0), "B_HSVactive_dino0": ("B", 0), "C_HSVactive_dino9": ("C", 9)}
M = {k: metrics(*v) for k, v in CFG.items()}

# ---- comparison CSV ----
with open(os.path.join(OUT, "csv", "phase1c_config_comparison.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["configuration", "TP", "FP", "FN", "TN", "precision", "recall", "F1",
                "specificity", "balanced_accuracy"])
    for k, m in M.items():
        w.writerow([k, m["TP"], m["FP"], m["FN"], m["TN"], m["precision"], m["recall"],
                    m["f1"], m["specificity"], m["balanced_accuracy"]])
print("=== Phase 1C config comparison (human GT 935, t=0.1214) ===")
for k, m in M.items():
    print(f"  {k:22s} TP {m['TP']} FP {m['FP']} FN {m['FN']} F1 {m['f1']} "
          f"prec {m['precision']} rec {m['recall']}")
dc = M["C_HSVactive_dino9"]["per"]["Dinosaur"]; db = M["B_HSVactive_dino0"]["per"]["Dinosaur"]
print(f"  Dinosaur: HSV+dino0 TP/FP/FN {db}  ->  HSV+dino9 (Phase1C) {dc}")

# ---- class metrics (config C) ----
with open(os.path.join(OUT, "csv", "phase1c_class_metrics.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["class_name", "GT_visible", "TP", "FP", "FN", "precision", "recall", "F1",
                                   "dinosaur_hue_recovered"])
    for o in OBJECTS:
        a = M["C_HSVactive_dino9"]["per"][o]
        gtv = a[0] + a[2]; P = a[0] / max(a[0] + a[1], 1); R = a[0] / max(a[0] + a[2], 1)
        rec_g = (dc[0] - db[0]) if o == "Dinosaur" else 0
        w.writerow([o, gtv, a[0], a[1], a[2], round(P, 3), round(R, 3),
                    round(2 * P * R / max(P + R, 1e-9), 3), rec_g])

# ---- per-candidate debug (config C) ----
with open(os.path.join(OUT, "csv", "phase1c_per_candidate_debug.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["dataset", "frame_id", "class_name", "bbox_uid", "semantic_score", "appearance_score",
                "hsv_score", "hsv_threshold", "hsv_comparison", "hsv_pass", "hue_correction_ocv",
                "hue_convention", "final_decision", "reject_reason", "gt_visible"])
    for (ds, f_, o), r in sorted(rec.items()):
        hc = 9 if o == "Dinosaur" else 0
        acc, hsv = decide(o, r, "C", hc)
        if not r["accept_b1"]:
            dec, rr = "reject", "semantic_or_appearance"
        elif hsv is not None and hsv < T:
            dec, rr = "reject", "hsv_below_threshold"
        else:
            dec, rr = "accept", ""
        w.writerow([ds, f_, o, r["uid"].split("|")[2], round(r["sem"], 4), round(r["appe"], 4),
                    round(hsv, 5) if hsv is not None else "", T, ">=", int(hsv >= T) if hsv is not None else "",
                    hc, "OpenCV(0-179); +9ocv=+18deg(0-360)", dec, rr, int(o in gt[(ds, f_)])])

# ---- HSV threshold sweep (config C) ----
def sweep(t):
    TP = FP = FN = 0; dino = 0
    for (ds, f), vis in gt.items():
        for o in OBJECTS:
            r = rec.get((ds, f, o))
            if not (r and r["accept_b1"]):
                if o in vis: FN += 1
                continue
            p = proto_for(o, 9 if o == "Dinosaur" else 0)
            hsv = ism_hsv.similarity(r["q"], p) if r["q"] is not None else 1.0
            acc = hsv >= t; v = o in vis
            if v and acc: TP += 1; dino += (o == "Dinosaur")
            elif acc: FP += 1
            elif v: FN += 1
    return TP, FP, FN, dino
with open(os.path.join(OUT, "csv", "phase1c_threshold_sweep_hsv.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["threshold", "TP", "FP", "FN", "F1", "Dinosaur_TP"])
    for t in [0.0, 0.05, 0.08, 0.10, 0.11, 0.12, 0.1214, 0.13, 0.14, 0.15]:
        TP, FP, FN, dino = sweep(t)
        w.writerow([t, TP, FP, FN, round(2 * TP / max(2 * TP + FP + FN, 1), 4), dino])

# ---- metrics.json + reproduction ----
mc = M["C_HSVactive_dino9"]; mb = M["B_HSVactive_dino0"]
json.dump({k: {kk: vv for kk, vv in m.items() if kk != "per"} for k, m in M.items()},
          open(os.path.join(OUT, "metrics", "phase1c_metrics.json"), "w"), indent=2)
repro = {
    "target_F1_baseline": 0.676, "target_F1_hsv": 0.753, "measured_C_F1": mc["f1"],
    "target_FP_hsv": 86, "measured_C_FP": mc["FP"],
    "target_dino_before": 33, "target_dino_after": 39,
    "measured_dino_hsv0": db[0], "measured_dino_hsv9": dc[0],
    "measured_dino_new_fp": dc[1],
    "note": "baseline F1 .676/FP334 is the HSV-OFF pipeline (config A here)."
}
json.dump(repro, open(os.path.join(OUT, "metrics", "reproduction_comparison.json"), "w"), indent=2)
print(f"\nreproduction: C F1={mc['f1']} FP={mc['FP']} | Dino {db[0]}->{dc[0]} (+{dc[0]-db[0]}), new FP {dc[1]}")
print(f"baseline(A) F1={M['A_baseline_noHSV']['f1']} FP={M['A_baseline_noHSV']['FP']}")
print(f"-> {OUT}")
