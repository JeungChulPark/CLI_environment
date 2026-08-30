#!/usr/bin/env python3
"""04_evaluate.py — GT-less heuristic quality labels + VLM sampling manifest.

No GT poses exist, so ADD/ADD-S/Rotation/Translation error and strict TP/TN/FP/FN
are NOT computable (reported as such in 06_report). What IS computable: pose-score
distribution, depth-plausibility, detection rate, and a sampled VLM qualitative pass.

Outputs (outputs_e2e/evaluation/):
  pose_quality_summary.csv   per ok-frame heuristic labels (all results)
  detection_stats.json       per (bag,object) aggregate detection rate / score stats
  vlm_sampling_manifest.csv   representative frames for Claude Vision labeling

Heuristic labels are PROVISIONAL (suffix *_heur); VLM refines the sampled subset.

Run:
  python tools/e2e_pipeline/04_evaluate.py
"""
import argparse
import csv
import glob
import json
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(__file__))
import pipeline_lib as L  # noqa: E402

EVAL_DIR = os.path.join(L.OUT, "evaluation")

# depth plausibility window for a tabletop RealSense scene (mm)
TZ_MIN, TZ_MAX = 150.0, 3500.0


def heur_labels(score, tz):
    """Provisional detection/pose labels from score + depth plausibility."""
    tz_ok = (tz is not None) and (TZ_MIN <= tz <= TZ_MAX)
    if score >= 0.50 and tz_ok:
        return "correct_object_visible", "good", "medium"
    if score >= 0.40 and tz_ok:
        return "uncertain", "partial", "low"
    if score >= 0.30:
        return "uncertain", "poor", "low"
    return "uncertain", "poor", "low" if tz_ok else "low"


def read_pose_csv(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--per-pair-samples", type=int, default=3,
                    help="representative frames per (bag,object) for VLM (best/median/worst)")
    args = ap.parse_args()
    os.makedirs(EVAL_DIR, exist_ok=True)

    pair_csvs = sorted(glob.glob(os.path.join(L.OUT, "*", "*", "pose_results.csv")))
    summary_rows = []
    stats = {}
    vlm_rows = []

    for csv_path in pair_csvs:
        parts = csv_path.split(os.sep)
        obj = parts[-2]
        bag = parts[-3]
        if bag in ("_frames", "_raw", "_logs", "evaluation"):
            continue
        rows = read_pose_csv(csv_path)
        ok = [r for r in rows if r["status"] == "ok" and r["score"] != ""]
        scores = [float(r["score"]) for r in ok]
        tzs = [float(r["tz"]) for r in ok if r["tz"] != ""]
        n_total = len(rows)
        det_rate = round(len(ok) / n_total, 3) if n_total else 0.0
        stats[f"{bag}/{obj}"] = {
            "bag": bag, "object": obj, "frames": n_total, "pose_ok": len(ok),
            "detection_rate": det_rate,
            "score_mean": round(st.mean(scores), 4) if scores else None,
            "score_max": round(max(scores), 4) if scores else None,
            "score_min": round(min(scores), 4) if scores else None,
            "tz_median_mm": round(st.median(tzs), 1) if tzs else None,
        }

        for r in ok:
            score = float(r["score"])
            tz = float(r["tz"]) if r["tz"] != "" else None
            dl, pl, conf = heur_labels(score, tz)
            img = os.path.join(L.OUT, bag, obj, f"frame_{r['frame_idx']}.png")
            summary_rows.append({
                "bag_name": bag, "object_id": obj, "frame_idx": r["frame_idx"],
                "result_image": os.path.relpath(img, L.OUT) if os.path.isfile(img) else "",
                "detection_label": dl + "_heur", "pose_quality_label": pl + "_heur",
                "confidence": conf,
                "notes": f"score={score:.3f} tz={tz:.0f}mm" if tz else f"score={score:.3f}",
            })

        # VLM representative sampling: best / median / worst by score (+ absence probe)
        picks = []
        if ok:
            ok_sorted = sorted(ok, key=lambda r: float(r["score"]))
            idxs = sorted(set([0, len(ok_sorted) // 2, len(ok_sorted) - 1]))
            picks = [ok_sorted[i] for i in idxs][:args.per_pair_samples]
        for r in picks:
            img = os.path.join(L.OUT, bag, obj, f"frame_{r['frame_idx']}.png")
            if os.path.isfile(img):
                vlm_rows.append({"bag_name": bag, "object_id": obj, "frame_idx": r["frame_idx"],
                                 "score": r["score"], "result_image": os.path.relpath(img, L.OUT)})

    with open(os.path.join(EVAL_DIR, "pose_quality_summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["bag_name", "object_id", "frame_idx", "result_image",
                                          "detection_label", "pose_quality_label", "confidence", "notes"])
        w.writeheader()
        w.writerows(summary_rows)
    with open(os.path.join(EVAL_DIR, "detection_stats.json"), "w") as f:
        json.dump(stats, f, indent=2)
    with open(os.path.join(EVAL_DIR, "vlm_sampling_manifest.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["bag_name", "object_id", "frame_idx", "score", "result_image"])
        w.writeheader()
        w.writerows(vlm_rows)

    print(f"[eval] pairs={len(pair_csvs)} summary_rows={len(summary_rows)} "
          f"vlm_samples={len(vlm_rows)}")
    print(f"[eval] wrote {EVAL_DIR}/{{pose_quality_summary.csv,detection_stats.json,vlm_sampling_manifest.csv}}")


if __name__ == "__main__":
    main()
