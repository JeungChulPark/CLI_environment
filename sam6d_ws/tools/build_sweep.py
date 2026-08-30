#!/usr/bin/env python3
"""cls semantic-threshold sweep on top of conf=0.02.

Selection is argmax-semantic (threshold-independent), so a SINGLE sim=0.35 run
draws the selected box for every frame that becomes milk at any T>=0.35. Box
correctness is judged ONCE per frame and reused across all thresholds:
  - best_sem >= 0.50  -> reuse conf02 audit (outputs/yolo_ism_audit_conf02)
  - best_sem in [0.35,0.50) -> fresh flip-audit (this tool's needs_vlm/vlm_raw)
milk_present is frame-intrinsic (baseline reuse / fresh override on flips).

Usage:
  python tools/build_sweep.py extract     # write needs_vlm manifests for flips
  python tools/build_sweep.py aggregate    # compute sweep metrics + FP/FN folders
"""
import csv
import glob
import json
import os
import shutil
import sys
import collections

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
S035 = os.path.join(REPO, "outputs", "yolo_ism_conf02_sim035")
BASE_VLM = os.path.join(REPO, "outputs", "yolo_ism_audit", "vlm_raw")
C02_CASES = os.path.join(REPO, "outputs", "yolo_ism_audit_conf02", "audit_cases_conf02.csv")
OUT = os.path.join(REPO, "outputs", "yolo_ism_audit_sweep")
TASKS = os.path.join(OUT, "needs_vlm")
FLIP_VLM = os.path.join(OUT, "vlm_raw")

BAGS = ["high_texture_around", "high_texture_far_close", "two_table_around",
        "two_table_around_goback", "two_table_diagonal1", "two_table_diagonal2",
        "two_table_goback", "only_milk", "milk_nomilk_bag"]
THRESHOLDS = [0.50, 0.45, 0.40, 0.35]
MILK_BOX = {"correct_milk_box", "partial_milk_box"}
NONMILK_BOX = {"background_box", "other_object_box"}
CHUNK = 40


def load_s035(bag):
    d = {}
    for r in csv.DictReader(open(os.path.join(S035, bag, "yolo_ism_results.csv"))):
        d[int(r["frame_idx"])] = {
            "name": r["timestamp"], "decision": r["decision"],
            "sem": float(r["best_semantic_score"]),
            "yolo": float(r["best_yolo_score"]),
            "appe": float(r["best_appe_score"]),
            "num": int(r["num_yolo_proposals"]),
        }
    return d


def extract():
    os.makedirs(TASKS, exist_ok=True)
    idx = []
    total = 0
    for bag in BAGS:
        s = load_s035(bag)
        items = []
        for fi, r in s.items():
            if r["decision"] == "milk" and 0.35 <= r["sem"] < 0.50:
                items.append({
                    "frame_idx": fi, "name": r["name"],
                    "overlay": os.path.join(S035, bag, "audit_overlay", f"{r['name']}.jpg"),
                    "decision": "milk", "num_proposals": r["num"],
                    "best_yolo_score": r["yolo"], "best_semantic_score": r["sem"],
                    "best_appe_score": r["appe"],
                })
        items.sort(key=lambda x: x["frame_idx"])
        for k in range(0, len(items), CHUNK):
            p = os.path.join(TASKS, f"{bag}__{k//CHUNK:02d}.json")
            json.dump(items[k:k+CHUNK], open(p, "w"), indent=1)
            idx.append({"bag": bag, "chunk": k//CHUNK, "n": len(items[k:k+CHUNK]), "path": p})
        total += len(items)
    json.dump(idx, open(os.path.join(TASKS, "_index.json"), "w"), indent=1)
    print(f"flip frames to audit (best_sem in [0.35,0.50)): {total} in {len(idx)} manifests")
    for m in idx:
        print(f"  {m['bag']}__{m['chunk']:02d}: {m['n']}")


def aggregate():
    # baseline frame-intrinsic milk_present
    base_mp = {}
    for bag in BAGS:
        for r in json.load(open(os.path.join(BASE_VLM, f"{bag}.json"))):
            base_mp[(bag, int(r["frame_idx"]))] = r.get("milk_present", "uncertain")
    # conf02 audit: milk_present + box for best_sem>=0.50 frames
    c02 = {}
    for r in csv.DictReader(open(C02_CASES)):
        c02[(r["bag"], int(r["frame_idx"]))] = (r["milk_present"], r["selected_box_correct"])
    # flip audit
    flip = {}
    for f in glob.glob(os.path.join(FLIP_VLM, "*.json")):
        bag = os.path.basename(f).rsplit("__", 1)[0]
        for r in json.load(open(f)):
            flip[(bag, int(r["frame_idx"]))] = (
                r.get("milk_present", "uncertain"), r.get("selected_box_correct", "na"))

    # unified per-frame: best_sem, milk_present, box_correct
    frames = {}
    for bag in BAGS:
        s = load_s035(bag)
        for fi, r in s.items():
            key = (bag, fi)
            sem = r["sem"]
            mp = base_mp.get(key, "uncertain")
            box = None
            if sem >= 0.50 and key in c02:
                mp_c, box_c = c02[key]
                mp, box = mp_c, box_c
            elif 0.35 <= sem < 0.50 and key in flip:
                mp_f, box_f = flip[key]
                mp, box = mp_f, box_f
            frames[key] = {"bag": bag, "fi": fi, "name": r["name"], "sem": sem,
                           "mp": mp, "box": box, "overlay":
                           os.path.join(S035, bag, "audit_overlay", f"{r['name']}.jpg")}

    def classify(mp, dec_milk, box):
        if mp == "uncertain":
            return "UNCERTAIN"
        if dec_milk and (box in (None, "", "uncertain")):
            return "UNCERTAIN"
        if dec_milk:
            if box in NONMILK_BOX or mp == "no":
                return "FP"
            if box in MILK_BOX:
                return "TP"
            return "UNCERTAIN"
        return "FN" if mp == "yes" else "TN"

    def safe_div(a, b):
        return round(a/b, 4) if b else None

    sweep = {}
    for T in THRESHOLDS:
        cnt = collections.Counter()
        per_bag = collections.defaultdict(collections.Counter)
        # prepare FP/FN folders
        tdir = os.path.join(OUT, f"T{T:.2f}")
        for cat in ["FP", "FN"]:
            d = os.path.join(tdir, cat)
            if os.path.isdir(d):
                shutil.rmtree(d)
            os.makedirs(d, exist_ok=True)
        for key, fr in frames.items():
            dec_milk = fr["sem"] >= T and fr["sem"] > 0
            cat = classify(fr["mp"], dec_milk, fr["box"])
            cnt[cat] += 1
            per_bag[fr["bag"]][cat] += 1
            if cat in ("FP", "FN") and os.path.isfile(fr["overlay"]):
                shutil.copy(fr["overlay"], os.path.join(
                    tdir, cat, f"{fr['bag']}__{fr['name']}_sem{fr['sem']:.3f}.jpg"))
        P = safe_div(cnt["TP"], cnt["TP"]+cnt["FP"])
        R = safe_div(cnt["TP"], cnt["TP"]+cnt["FN"])
        F = safe_div(2*(P or 0)*(R or 0), (P or 0)+(R or 0)) if P and R else None
        sweep[f"{T:.2f}"] = {
            "TP": cnt["TP"], "FP": cnt["FP"], "FN": cnt["FN"], "TN": cnt["TN"],
            "uncertain": cnt["UNCERTAIN"], "precision": P, "recall": R, "f1": F,
            "accuracy": safe_div(cnt["TP"]+cnt["TN"], cnt["TP"]+cnt["FP"]+cnt["FN"]+cnt["TN"]),
        }
    summary = {
        "pseudo_gt_disclaimer": "pseudo-GT based (VLM-assisted). conf fixed 0.02; "
        "cls semantic-threshold swept. Selection is threshold-independent so a single "
        "sim=0.35 run + one box judgment/frame covers all thresholds.",
        "conf": 0.02, "thresholds": sweep,
    }
    json.dump(summary, open(os.path.join(OUT, "sweep_metrics.json"), "w"),
              indent=2, ensure_ascii=False)
    print("=== cls threshold sweep (conf 0.02 fixed) ===")
    print(f"{'T':>6} {'TP':>4} {'FP':>4} {'FN':>4} {'TN':>4} {'prec':>7} {'rec':>7} {'f1':>7}")
    for T in THRESHOLDS:
        s = sweep[f"{T:.2f}"]
        print(f"{T:6.2f} {s['TP']:4d} {s['FP']:4d} {s['FN']:4d} {s['TN']:4d} "
              f"{s['precision']:7} {s['recall']:7} {s['f1']:7}")
    print(f"\nFP/FN images: {OUT}/T<thr>/{{FP,FN}}/")


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "extract"
    (extract if mode == "extract" else aggregate)()
