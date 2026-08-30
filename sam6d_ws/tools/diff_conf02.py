#!/usr/bin/env python3
"""Differential audit prep: conf=0.02 vs conf=0.05 baseline.

milk_present / view_state are frame-intrinsic -> reuse baseline VLM labels.
Only frames whose conf02 decision==milk with a NEW or materially CHANGED
selected box need fresh VLM box-correctness judgment.

Emits:
  outputs/yolo_ism_audit_conf02/reuse_cases.json   frames classifiable now
  outputs/yolo_ism_audit_conf02/needs_vlm/<bag>__<k>.json   manifests for re-audit
  prints a diff summary
"""
import csv
import json
import os
import collections

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = os.path.join(REPO, "outputs", "yolo_ism")
C02 = os.path.join(REPO, "outputs", "yolo_ism_conf02")
VLM = os.path.join(REPO, "outputs", "yolo_ism_audit", "vlm_raw")
OUT = os.path.join(REPO, "outputs", "yolo_ism_audit_conf02")
TASKS = os.path.join(OUT, "needs_vlm")

BAGS = ["high_texture_around", "high_texture_far_close", "two_table_around",
        "two_table_around_goback", "two_table_diagonal1", "two_table_diagonal2",
        "two_table_goback", "only_milk", "milk_nomilk_bag"]
CHUNK = 40


def parse_box(s):
    s = (s or "").strip()
    if not s:
        return None
    try:
        return [int(float(v)) for v in s.split(";")]
    except Exception:
        return None


def iou(a, b):
    if a is None or b is None:
        return 0.0
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def load_results(path):
    d = {}
    for r in csv.DictReader(open(path)):
        d[int(r["frame_idx"])] = {
            "name": r["timestamp"], "decision": r["decision"],
            "box": parse_box(r["selected_bbox_xyxy"]),
            "best_yolo": r["best_yolo_score"], "best_sem": r["best_semantic_score"],
            "best_appe": r["best_appe_score"], "num": int(r["num_yolo_proposals"]),
        }
    return d


def main():
    os.makedirs(TASKS, exist_ok=True)
    reuse = []
    needs = collections.defaultdict(list)
    diff = collections.Counter()

    for bag in BAGS:
        base = load_results(os.path.join(BASE, bag, "yolo_ism_results.csv"))
        c02 = load_results(os.path.join(C02, bag, "yolo_ism_results.csv"))
        vlm = {int(r["frame_idx"]): r for r in
               json.load(open(os.path.join(VLM, f"{bag}.json")))}
        # baseline category needs baseline box label
        for fi, c in c02.items():
            b = base.get(fi, {})
            v = vlm.get(fi, {})
            mp = v.get("milk_present", "uncertain")
            view = v.get("view_state", "")
            new_milk = c["decision"] == "milk"
            old_milk = b.get("decision") == "milk"

            if not new_milk:
                # decision negative -> classifiable from milk_present only
                reuse.append({"bag": bag, "frame_idx": fi, "name": c["name"],
                              "milk_present": mp, "view_state": view,
                              "decision": c["decision"],
                              "selected_box_correct": "na",
                              "selected_box_quality": "na",
                              "best_semantic_score": c["best_sem"],
                              "source": "reuse"})
                diff["c02_negative"] += 1
                continue

            # conf02 decided milk
            same_box = old_milk and iou(b.get("box"), c["box"]) > 0.8
            if same_box and v.get("selected_box_correct", "na") not in ("na", ""):
                reuse.append({"bag": bag, "frame_idx": fi, "name": c["name"],
                              "milk_present": mp, "view_state": view,
                              "decision": "milk",
                              "selected_box_correct": v["selected_box_correct"],
                              "selected_box_quality": v.get("selected_box_quality", "na"),
                              "best_semantic_score": c["best_sem"],
                              "source": "reuse_box"})
                diff["c02_milk_unchanged_box"] += 1
            else:
                needs[bag].append({
                    "frame_idx": fi, "name": c["name"],
                    "overlay": os.path.join(C02, bag, "audit_overlay", f"{c['name']}.jpg"),
                    "decision": "milk", "num_proposals": c["num"],
                    "best_yolo_score": float(c["best_yolo"]),
                    "best_semantic_score": float(c["best_sem"]),
                    "best_appe_score": float(c["best_appe"]),
                    "baseline_milk_present": mp, "baseline_view_state": view,
                })
                diff["c02_milk_needs_vlm"] += 1

    json.dump(reuse, open(os.path.join(OUT, "reuse_cases.json"), "w"), indent=1)
    idx = []
    for bag, items in needs.items():
        for k in range(0, len(items), CHUNK):
            ch = items[k:k + CHUNK]
            p = os.path.join(TASKS, f"{bag}__{k//CHUNK:02d}.json")
            json.dump(ch, open(p, "w"), indent=1)
            idx.append({"bag": bag, "chunk": k // CHUNK, "n": len(ch), "path": p})
    json.dump(idx, open(os.path.join(TASKS, "_index.json"), "w"), indent=1)

    print("diff summary:", dict(diff))
    print(f"reuse-classifiable frames : {len(reuse)}")
    print(f"need fresh VLM (new/changed milk box): {sum(m['n'] for m in idx)} "
          f"in {len(idx)} manifests")
    for m in idx:
        print(f"  {m['bag']}__{m['chunk']:02d}: {m['n']}")


if __name__ == "__main__":
    main()
