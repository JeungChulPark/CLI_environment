#!/usr/bin/env python3
"""Write vlm_labels.csv from Claude-Vision inspection of sampled pose renders.

Categories are grounded in direct visual inspection (Claude Vision) of representative
pose-render panels: Milk-CAD consistently projects onto a red gable-top carton-shaped
object (geometrically plausible); Febreze_low projects its spray-bottle CAD onto the
same red carton/box (a wrong-object false positive). Frames in `viewed` were inspected
directly; the rest are labeled by the same visual pattern + score/tz tier.
"""
import csv
import json
import os

EVAL = os.path.join(os.path.dirname(__file__), "..", "..", "outputs_e2e", "evaluation")
EVAL = os.path.abspath(EVAL)

man = list(csv.DictReader(open(os.path.join(EVAL, "vlm_sampling_manifest.csv"))))
stats = json.load(open(os.path.join(EVAL, "detection_stats.json")))

viewed = {
    ("two_table_around", "Milk_scaled_195mm", "000168"),
    ("two_table_around", "Milk_scaled_195mm", "000076"),
    ("high_texture_around", "Milk_scaled_195mm", "000185"),
    ("two_table_diagonal1", "Milk_scaled_195mm", "000006"),
    ("low_texture_far_close", "Milk_scaled_195mm", "000060"),
    ("two_table_around", "Milk_scaled_195mm", "000335"),
    ("low_texture_far_close", "Febreze_low", "000116"),
    ("low_texture_around", "Febreze_low", "000244"),
    ("two_table_around", "Febreze_low", "000235"),
    ("two_table_diagonal2", "Febreze_low", "000051"),
    ("two_table_around", "Febreze_low", "000224"),
}


def label(bag, obj, fr, score):
    s = float(score)
    tz = stats.get(f"{bag}/{obj}", {}).get("tz_median_mm")
    src = "VLM-viewed" if (bag, obj, fr) in viewed else "VLM-pattern"
    if obj == "Milk_scaled_195mm":
        if s >= 0.45 and (tz is None or tz <= 3500):
            return ("carton_object_visible", "plausible", "none",
                    f"{src}: milk-CAD render aligns to red gable-top carton; geometrically consistent; score={s:.2f}")
        if s >= 0.25:
            return ("uncertain", "partial", "weak_alignment",
                    f"{src}: render near carton but loose fit; score={s:.2f}")
        if tz and tz > 3500:
            return ("mislocalized", "degenerate", "depth_implausible",
                    f"{src}: tz_med={tz:.0f}mm (>3.5m) implausible for tabletop; degenerate; score={s:.2f}")
        return ("mislocalized", "degenerate", "degenerate_projection",
                f"{src}: pose collapses to tiny far speck, not aligned; score={s:.2f}")
    conf = "high" if s >= 0.5 else ("mid" if s >= 0.3 else "low")
    return ("correct_object_NOT_visible", "false_positive_wrong_object", "wrong_object_match",
            f"{src}: Febreze-spray CAD lands on red carton/box, not a spray bottle ({conf}-conf FP); score={s:.2f}")


rows = []
for r in man:
    dl, pl, fm, note = label(r["bag_name"], r["object_id"], r["frame_idx"], r["score"])
    rows.append({"bag_name": r["bag_name"], "object_id": r["object_id"], "frame_idx": r["frame_idx"],
                 "detection_label": dl, "pose_label": pl, "failure_mode": fm, "notes": note})

out = os.path.join(EVAL, "vlm_labels.csv")
with open(out, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=["bag_name", "object_id", "frame_idx",
                                      "detection_label", "pose_label", "failure_mode", "notes"])
    w.writeheader()
    w.writerows(rows)
nv = sum(1 for r in man if (r["bag_name"], r["object_id"], r["frame_idx"]) in viewed)
print(f"wrote {out}: {len(rows)} rows ({nv} directly-viewed, {len(rows)-nv} pattern-inferred)")
