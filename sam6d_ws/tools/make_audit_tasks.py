#!/usr/bin/env python3
"""Build chunked VLM-audit task manifests from yolo_ism outputs.

Each manifest is a JSON list of per-frame items the VLM auditor inspects:
  {frame_idx, name, overlay, decision, num_proposals,
   best_yolo_score, best_semantic_score, best_appe_score}
Manifests: outputs/yolo_ism_audit/tasks/<bag>__<chunk>.json
"""
import csv
import json
import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ISM_ROOT = os.path.join(REPO, "outputs", "yolo_ism")
TASKS = os.path.join(REPO, "outputs", "yolo_ism_audit", "tasks")

BAGS = ["high_texture_around", "high_texture_far_close", "two_table_around",
        "two_table_around_goback", "two_table_diagonal1", "two_table_diagonal2",
        "two_table_goback", "only_milk", "milk_nomilk_bag"]
CHUNK = 40


def main():
    os.makedirs(TASKS, exist_ok=True)
    manifest_index = []
    for bag in BAGS:
        csv_path = os.path.join(ISM_ROOT, bag, "yolo_ism_results.csv")
        if not os.path.isfile(csv_path):
            continue
        items = []
        with open(csv_path) as f:
            for r in csv.DictReader(f):
                name = r["timestamp"]
                items.append({
                    "frame_idx": int(r["frame_idx"]),
                    "name": name,
                    "overlay": os.path.join(ISM_ROOT, bag, "audit_overlay", f"{name}.jpg"),
                    "decision": r["decision"],
                    "num_proposals": int(r["num_yolo_proposals"]),
                    "best_yolo_score": float(r["best_yolo_score"]),
                    "best_semantic_score": float(r["best_semantic_score"]),
                    "best_appe_score": float(r["best_appe_score"]),
                })
        for k in range(0, len(items), CHUNK):
            chunk = items[k:k + CHUNK]
            cid = k // CHUNK
            path = os.path.join(TASKS, f"{bag}__{cid:02d}.json")
            json.dump(chunk, open(path, "w"), indent=1)
            manifest_index.append({"bag": bag, "chunk": cid, "n": len(chunk),
                                   "path": path})
    json.dump(manifest_index, open(os.path.join(TASKS, "_index.json"), "w"), indent=1)
    print(f"wrote {len(manifest_index)} manifests covering "
          f"{sum(m['n'] for m in manifest_index)} frames")
    for m in manifest_index:
        print(f"  {m['bag']}__{m['chunk']:02d}: {m['n']}")


if __name__ == "__main__":
    main()
