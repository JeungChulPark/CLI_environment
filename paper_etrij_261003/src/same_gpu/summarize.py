#!/usr/bin/env python3
"""Collect raw_*.json into results.json (run 1 = primary; run 2 = repeat)."""
import json
import statistics

import common as C


def load(n):
    return json.load(open(C.OUT / n))


def per_obj(rs, pem_key, n_key):
    xs = [p[pem_key] / p[n_key] for r in rs for p in r["per_frame"] if p[n_key] > 0]
    return round(statistics.median(xs), 1)


def strip(r):
    return {k: v for k, v in r.items() if k != "per_frame"}


A1, A2 = load("raw_A_original_dinov2_vitl14_fastsamx_run1.json"), load("raw_A_original_dinov2_vitl14_fastsamx_run2.json")
AS = load("raw_A_original_dinov2_vits14_fastsamx.json")
B1, B2 = load("raw_B_ours_run1.json"), load("raw_B_ours_run2.json")
for r in (A1, A2, AS):
    for p in r["per_frame"]:
        p["_pem"] = p["ms"]["pem"]
for r in (B1, B2):
    for p in r["per_frame"]:
        p["_pem"] = p["core_ms"]["pem"]

res = {
    "gpu": A1["gpu"], "torch": A1["torch"],
    "dataset": str(C.DATASET), "frames": C.FRAMES, "warmup_frames": C.WARMUP,
    "objects": C.OBJECTS,
    "protocol": "3 warm-up frames, then each of the 31 frames once; torch.cuda.synchronize() at every "
                "stage boundary; frames pre-loaded in RAM (no disk I/O, no visualization, no JSON writing "
                "inside the timed region); peak memory = torch max_memory_allocated/reserved over the timed "
                "loop (model weights included); A and B run in separate processes, nothing else on the GPU.",
    "A_original_sam6d": {
        "config": "FastSAM-x segment-everything (iou 0.9, conf 0.25, max_det 200, 640) -> small-box/mask filter "
                  "-> DINOv2 ViT-L/14 cls+patch (chunk 16) -> semantic/appearance/geometric weighted-sum score, "
                  "one pass over 8 objects x 42 templates -> NMS per object (0.25) -> top-1 per object -> "
                  "PEM (sam-6d-pem-base, upstream code) per object whose top-1 ISM score > 0.2, one forward per object",
        "total_ms": {"median": A1["total_ms"]["median"], "p90": A1["total_ms"]["p90"]},
        "stage_ms_median": {"proposals_fastsam": A1["stage_ms"]["proposals"]["median"],
                            "descriptors_dinov2": A1["stage_ms"]["descriptors"]["median"],
                            "ism_scoring_nms_top1": A1["stage_ms"]["matching"]["median"],
                            "ism_total": round(A1["stage_ms"]["proposals"]["median"] + A1["stage_ms"]["descriptors"]["median"] + A1["stage_ms"]["matching"]["median"], 1),
                            "pem": A1["stage_ms"]["pem"]["median"]},
        "ism_ms_per_frame_median": round(statistics.median(p["ms"]["proposals"] + p["ms"]["descriptors"] + p["ms"]["matching"] for p in A1["per_frame"]), 1),
        "pem_ms_per_object_median": per_obj([A1], "_pem", "n_pose_output"),
        "objects_with_ism_top1_per_frame": {"median": A1["n_objects_with_ism_detection_per_frame"]["median"],
                                            "mean": A1["n_objects_with_ism_detection_per_frame"]["mean"]},
        "pose_outputs_per_frame": {"median": A1["n_pose_output_per_frame"]["median"],
                                   "mean": A1["n_pose_output_per_frame"]["mean"],
                                   "note": "no acceptance test exists in the original; every top-1 above 0.2 is output"},
        "fastsam_proposals_per_frame_median": A1["n_fastsam_proposals"]["median"],
        "gpu_mem_peak_MiB": A1["gpu_mem_peak_MiB"],
        "repeat_run_total_ms": {"median": A2["total_ms"]["median"], "p90": A2["total_ms"]["p90"]},
    },
    "A_variant_dinov2_vits14": {
        "note": "same as A but DINOv2 ViT-S/14 (the local sam6d_ws config value; our recognizer also uses ViT-S/14)",
        "total_ms": {"median": AS["total_ms"]["median"], "p90": AS["total_ms"]["p90"]},
        "stage_ms_median": {k: v["median"] for k, v in AS["stage_ms"].items()},
        "pose_outputs_per_frame_median": AS["n_pose_output_per_frame"]["median"],
        "gpu_mem_peak_MiB": AS["gpu_mem_peak_MiB"],
    },
    "B_ours": {
        "config": "production Sam6DCore.process: YOLO-World (8 text prompts) -> DINOv2 ViT-S/14 + MobileSAM + "
                  "sequential gates (semantic, appearance, HSV, assignment, cross-object NMS) -> PEM batch of "
                  "accepted objects -> pose verification; run config live_260915_eightcircle_orbslam3, objects = 8",
        "total_ms": {"median": B1["total_ms"]["median"], "p90": B1["total_ms"]["p90"]},
        "stage_ms_median": {"yolo_world": B1["stage_ms"]["yolo"]["median"],
                            "ism_dinov2_mobilesam_gates": B1["stage_ms"]["ism"]["median"],
                            "pem_plus_verification": B1["stage_ms"]["pem"]["median"]},
        "ism_ms_per_frame_median": round(statistics.median(p["core_ms"]["yolo"] + p["core_ms"]["ism"] for p in B1["per_frame"]), 1),
        "pem_ms_per_object_median": per_obj([B1], "_pem", "n_ism_accepted"),
        "ism_accepted_per_frame": {"median": B1["n_ism_accepted_per_frame"]["median"], "mean": B1["n_ism_accepted_per_frame"]["mean"]},
        "accepted_poses_per_frame": {"median": B1["n_output_per_frame"]["median"], "mean": B1["n_output_per_frame"]["mean"],
                                     "frames_with_zero": sum(p["n_output"] == 0 for p in B1["per_frame"])},
        "gpu_mem_peak_MiB": B1["gpu_mem_peak_MiB"],
        "repeat_run_total_ms": {"median": B2["total_ms"]["median"], "p90": B2["total_ms"]["p90"]},
    },
    "raw_files": ["raw_A_original_dinov2_vitl14_fastsamx_run1.json", "raw_A_original_dinov2_vitl14_fastsamx_run2.json",
                  "raw_A_original_dinov2_vits14_fastsamx.json", "raw_B_ours_run1.json", "raw_B_ours_run2.json"],
}
json.dump(res, open(C.OUT / "results.json", "w"), indent=1)
print(json.dumps(res, indent=1))
