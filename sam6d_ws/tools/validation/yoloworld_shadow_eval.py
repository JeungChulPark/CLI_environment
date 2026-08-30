#!/usr/bin/env python3
"""
YOLO-World Shadow Validation for Milk Presence Gating (offline, read-only).

Shadow experiment ONLY — does NOT touch ROS / FastSAM / ISM / PEM / scores.
Runs YOLO-World on the 311 validation frames in the isolated `sam_yolo` env and
answers: does a YOLO-World front-gate keep milk recall, cut SAM-6D's FP=120, and
let us skip ISM/PEM on absent frames to maintain/improve latency?

GT source: outputs/validation/SLAM_with_milk_nomilk_baseline_bak/frame_results.csv
  decision_type: TP/FN -> milk present, FP/TN -> milk absent (0 mismatch vs visibility_labels).
SAM-6D baseline on these 311 frames: TP=112, FN=6, FP=120, TN=73
  -> present=118, absent=193, SAM-6D recall=112/118=0.949, SAM-6D FP=120.

Outputs raw per-frame per-prompt max-confidence to a JSON cache and prints the full
benchmark (prompt comparison, presence P/R/F1, FP reduction, recall risk, runtime,
ISM/PEM skip simulation, GO/NO-GO inputs).

Run: conda run -n sam_yolo python tools/validation/yoloworld_shadow_eval.py
"""
import os, sys, csv, json, time, argparse

ROOT = "/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"
FRAMES_DIR = os.path.join(ROOT, "outputs/validation/SLAM_with_milk_nomilk/frames")
GT_CSV = os.path.join(ROOT, "outputs/validation/SLAM_with_milk_nomilk_baseline_bak/frame_results.csv")
CACHE = os.path.join(ROOT, "outputs/validation/yoloworld_shadow_cache.json")

# SAM-6D per-frame pipeline runtime baseline (from /tmp/rerun_logs/launch2.log)
SAM6D_TOTAL_S = 0.878      # mean wall time per frame
SAM6D_GPU_S = 0.694        # GPU inference part (ISM-DINOv2 + PEM) that a gate can skip

PROMPTS = {
    "milk carton": ["milk carton"],
    "milk box": ["milk box"],
    "milk package": ["milk package"],
    "milk bottle": ["milk bottle"],
    "carton": ["carton"],
    "milk": ["milk"],
    # combo: union of detections across several phrasings (max conf over the set)
    "combo(carton+box+package)": ["milk carton", "milk box", "milk package", "carton"],
}


def load_gt():
    gt = {}
    with open(GT_CSV) as f:
        for r in csv.DictReader(f):
            gt[r["frame_id"]] = r["decision_type"]
    frames = sorted(fn[:-4] for fn in os.listdir(FRAMES_DIR) if fn.endswith(".png"))
    frames = [f for f in frames if f in gt]
    return frames, gt


def present(dt):  # GT milk present?
    return dt in ("TP", "FN")


def run_inference(weights, frames, prompt_sets, warmup=5):
    """Return {frame: {prompt_label: max_conf}} and measured ms/frame (single-class).

    NOTE: ultralytics YOLO-World moves the CLIP text model to CPU; calling set_classes()
    AFTER a GPU predict triggers a device mismatch. We therefore reload a fresh model per
    phrase and call set_classes() BEFORE any predict (the only order that works).
    """
    from ultralytics import YOLOWorld
    import numpy as np  # noqa

    phrases = sorted({p for ps in prompt_sets.values() for p in ps})

    raw = {f: {} for f in frames}  # frame -> phrase -> max_conf
    for phrase in phrases:
        model = YOLOWorld(weights)           # fresh model per phrase (avoids device bug)
        model.set_classes([phrase])
        for f in frames:
            img = os.path.join(FRAMES_DIR, f + ".png")
            res = model.predict(img, conf=0.001, imgsz=640, verbose=False, device=0)
            confs = res[0].boxes.conf.tolist() if res[0].boxes is not None else []
            raw[f][phrase] = max(confs) if confs else 0.0
        print(f"  [phrase done] '{phrase}'", flush=True)

    # Aggregate phrase confidences into prompt-set labels (max over phrases in the set)
    perframe = {}
    for f in frames:
        perframe[f] = {}
        for label, ps in prompt_sets.items():
            perframe[f][label] = max((raw[f][p] for p in ps), default=0.0)

    # ---- Runtime measurement: single representative class, warm GPU ----
    model = YOLOWorld(weights)
    model.set_classes(["milk carton"])
    sample = frames[: min(len(frames), 120)]
    for f in sample[:warmup]:
        model.predict(os.path.join(FRAMES_DIR, f + ".png"), conf=0.25, imgsz=640, verbose=False, device=0)
    import torch
    torch.cuda.synchronize()
    t0 = time.time()
    for f in sample:
        model.predict(os.path.join(FRAMES_DIR, f + ".png"), conf=0.25, imgsz=640, verbose=False, device=0)
    torch.cuda.synchronize()
    ms_per_frame = (time.time() - t0) / len(sample) * 1000.0
    return perframe, ms_per_frame


def confusion(frames, gt, conf_by_frame, thr):
    """Presence detection vs GT at a confidence threshold."""
    TP = FP = TN = FN = 0
    for f in frames:
        det = conf_by_frame[f] >= thr
        gp = present(gt[f])
        if det and gp: TP += 1
        elif det and not gp: FP += 1
        elif (not det) and gp: FN += 1
        else: TN += 1
    P = TP / (TP + FP) if (TP + FP) else 0.0
    R = TP / (TP + FN) if (TP + FN) else 0.0
    F1 = 2 * P * R / (P + R) if (P + R) else 0.0
    return dict(TP=TP, FP=FP, TN=TN, FN=FN, P=P, R=R, F1=F1)


def gate_analysis(frames, gt, conf_by_frame, thr):
    """Front-gate effect on SAM-6D outcomes.
    Gate keeps a frame iff YOLO-World detects (conf>=thr). SAM-6D output is ANDed with gate.
    SAM-6D published on TP (correct) and FP (wrong). Gate removes those where YOLO says absent.
    """
    sam_fp = [f for f in frames if gt[f] == "FP"]   # 120: SAM-6D wrongly published
    sam_tp = [f for f in frames if gt[f] == "TP"]   # 112: SAM-6D correctly published
    fn_frames = [f for f in frames if gt[f] == "FN"]  # 6: SAM-6D already missed
    fp_kept = sum(1 for f in sam_fp if conf_by_frame[f] >= thr)   # FP that survive gate
    fp_removed = len(sam_fp) - fp_kept
    tp_kept = sum(1 for f in sam_tp if conf_by_frame[f] >= thr)   # TP that survive gate
    tp_lost = len(sam_tp) - tp_kept                               # recall loss
    present_n = len([f for f in frames if present(gt[f])])        # 118
    new_recall = tp_kept / present_n
    return dict(
        sam_fp=len(sam_fp), fp_removed=fp_removed, fp_kept=fp_kept,
        sam_tp=len(sam_tp), tp_kept=tp_kept, tp_lost=tp_lost,
        present_n=present_n, sam_recall=len(sam_tp) / present_n, new_recall=new_recall,
        fn_added=tp_lost,
    )


def skip_sim(frames, gt, conf_by_frame, thr):
    """Latency if YOLO-World gate skips ISM/PEM on frames it calls 'absent'."""
    n = len(frames)
    gate_negative = sum(1 for f in frames if conf_by_frame[f] < thr)  # frames we skip SAM-6D GPU on
    # On skipped frames: only YOLO runtime. On kept frames: YOLO + full SAM-6D.
    # ms_per_frame filled by caller via closure; here return counts only.
    return dict(n=n, skipped=gate_negative, ran=n - gate_negative,
                skip_ratio=gate_negative / n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default="yolov8s-worldv2.pt")
    ap.add_argument("--from-cache", action="store_true", help="reuse cached per-frame confidences")
    ap.add_argument("--prompt", default=None, help="force the gate prompt for sweep (else best F1)")
    args = ap.parse_args()

    frames, gt = load_gt()
    print(f"[data] frames={len(frames)} present={sum(present(gt[f]) for f in frames)} "
          f"absent={sum(not present(gt[f]) for f in frames)}  weights={args.weights}")

    cache_key = args.weights
    ms_per_frame = None
    if args.from_cache and os.path.exists(CACHE):
        blob = json.load(open(CACHE))
        if cache_key in blob:
            perframe = blob[cache_key]["perframe"]
            ms_per_frame = blob[cache_key].get("ms_per_frame")
            print(f"[cache] loaded {cache_key}")
    if ms_per_frame is None:
        perframe, ms_per_frame = run_inference(args.weights, frames, PROMPTS)
        blob = json.load(open(CACHE)) if os.path.exists(CACHE) else {}
        blob[cache_key] = dict(perframe=perframe, ms_per_frame=ms_per_frame)
        json.dump(blob, open(CACHE, "w"))

    print(f"\n[runtime] YOLO-World({args.weights}) = {ms_per_frame:.2f} ms/frame "
          f"= {1000.0/ms_per_frame:.1f} FPS")

    # ---- Task 2: prompt comparison (recall-oriented, thr=0.05) ----
    print("\n=== PROMPT COMPARISON (presence detection @ several thresholds) ===")
    print(f"{'prompt':<28} {'thr':>5} {'R':>6} {'P':>6} {'F1':>6} {'TP':>4} {'FP':>4} {'FN':>4} {'TN':>4}")
    for label in PROMPTS:
        cbf = {f: perframe[f][label] for f in frames}
        for thr in (0.05, 0.10):
            c = confusion(frames, gt, cbf, thr)
            print(f"{label:<28} {thr:>5.2f} {c['R']:>6.3f} {c['P']:>6.3f} {c['F1']:>6.3f} "
                  f"{c['TP']:>4} {c['FP']:>4} {c['FN']:>4} {c['TN']:>4}")

    # ---- Per-prompt GATE summary at a recall-preserving point ----
    # For each prompt, find the threshold that keeps gated-recall closest to >=0.94
    # while maximizing FP removed. This is the real gate objective (not raw recall).
    print("\n=== PER-PROMPT FRONT-GATE @ recall-preserving threshold (target newRec>=0.94) ===")
    print(f"{'prompt':<28} {'thr':>5} {'newRec':>7} {'FPrem':>6} {'FPkeep':>6} {'TPlost':>6} {'skip%':>6}")
    THRS = [round(0.01 * k, 2) for k in range(1, 41)]
    best_prompt, best_score = None, -1
    for label in PROMPTS:
        cbf = {f: perframe[f][label] for f in frames}
        pick = None
        for thr in THRS:
            g = gate_analysis(frames, gt, cbf, thr)
            if g["new_recall"] >= 0.94:
                pick = (thr, g)        # keep raising thr while recall holds
        if pick is None:
            thr0 = THRS[0]; pick = (thr0, gate_analysis(frames, gt, cbf, thr0))
        thr, g = pick
        s = skip_sim(frames, gt, cbf, thr)
        print(f"{label:<28} {thr:>5.2f} {g['new_recall']:>7.3f} {g['fp_removed']:>6} "
              f"{g['fp_kept']:>6} {g['tp_lost']:>6} {s['skip_ratio']*100:>5.1f}%")
        # score: FP removed, tie-break higher recall
        sc = g["fp_removed"] + g["new_recall"]
        if sc > best_score:
            best_score, best_prompt = sc, label
    if args.prompt:
        best_prompt = args.prompt
    print(f"\n[gate prompt] = '{best_prompt}'  (override={args.prompt})")

    # ---- Tasks 3-7 on best prompt across threshold sweep ----
    cbf = {f: perframe[f][best_prompt] for f in frames}
    print(f"\n=== THRESHOLD SWEEP on '{best_prompt}' "
          f"(presence + front-gate effect + latency) ===")
    hdr = (f"{'thr':>5} | {'R':>6} {'P':>6} {'F1':>6} | "
           f"{'FPrem':>6} {'FPkeep':>6} {'TPlost':>6} {'newRec':>7} | "
           f"{'skip%':>6} {'avg_ms':>8} {'FPS':>6}")
    print(hdr); print("-" * len(hdr))
    sweep = []
    for thr in (0.02, 0.05, 0.08, 0.10, 0.15, 0.20, 0.25, 0.30):
        c = confusion(frames, gt, cbf, thr)
        g = gate_analysis(frames, gt, cbf, thr)
        s = skip_sim(frames, gt, cbf, thr)
        # avg latency: every frame pays YOLO; only non-skipped frames pay SAM-6D GPU part.
        # (SAM-6D non-GPU overhead 0.184s assumed to remain on ran frames; skipped frames pay only YOLO)
        sam_overhead = SAM6D_TOTAL_S - SAM6D_GPU_S
        ran_frac = s["ran"] / s["n"]
        avg_s = (ms_per_frame / 1000.0) + ran_frac * SAM6D_GPU_S + ran_frac * sam_overhead
        avg_fps = 1.0 / avg_s
        sweep.append((thr, c, g, s, avg_s, avg_fps))
        print(f"{thr:>5.2f} | {c['R']:>6.3f} {c['P']:>6.3f} {c['F1']:>6.3f} | "
              f"{g['fp_removed']:>6} {g['fp_kept']:>6} {g['tp_lost']:>6} {g['new_recall']:>7.3f} | "
              f"{s['skip_ratio']*100:>5.1f}% {avg_s*1000:>7.1f}ms {avg_fps:>6.2f}")

    # ---- Summary at recall-preserving operating point ----
    # pick the highest thr that keeps new_recall >= 0.93 (within ~0.02 of SAM-6D 0.949)
    keep = [row for row in sweep if row[2]["new_recall"] >= 0.93]
    op = keep[-1] if keep else sweep[0]
    thr, c, g, s, avg_s, avg_fps = op
    print("\n=== RECOMMENDED OPERATING POINT (recall-preserving) ===")
    print(f"threshold = {thr:.2f}")
    print(f"  SAM-6D recall {g['sam_recall']:.3f} -> gated recall {g['new_recall']:.3f} "
          f"(TP lost={g['tp_lost']}, FN added={g['fn_added']})")
    print(f"  FP: SAM-6D {g['sam_fp']} -> after gate {g['fp_kept']}  (removed {g['fp_removed']})")
    print(f"  ISM/PEM skipped on {s['skipped']}/{s['n']} frames ({s['skip_ratio']*100:.1f}%)")
    print(f"  latency {SAM6D_TOTAL_S*1000:.0f}ms -> {avg_s*1000:.1f}ms  | "
          f"FPS {1.0/SAM6D_TOTAL_S:.2f} -> {avg_fps:.2f}")
    print(f"\n[cache] {CACHE}")


if __name__ == "__main__":
    main()
