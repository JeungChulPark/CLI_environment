#!/usr/bin/env python3
"""yolo_ism_object_n.py — config-driven MULTI-object YOLO-World × SAM-6D ISM.

Generalizes the single-object ``yolo_ism.py`` to N objects WITHOUT modifying it:
every DINOv2 / template / scoring / frame-source helper is imported and reused
from ``yolo_ism`` (read-only). Per RGB frame:

  1. YOLO-World runs ONCE over the set of UNIQUE object prompts (set_classes).
  2. Detected boxes are routed by their predicted prompt to every enabled
     object sharing that prompt (high/low variants share a prompt; they differ
     only in their ISM *template* features, not in YOLO text).
  3. For each object: its proposals -> DINOv2 cls (SEMANTIC, 1st gate) selects
     the best proposal; if --use_mask, MobileSAM masks it and a masked DINOv2
     patch score (APPEarance, 2nd gate) decides acceptance.

Object prompts / template dirs / feature-cache paths / per-object threshold
overrides all come from a YAML config (configs/yolo_ism_objects.yaml) — nothing
object-specific is hard-coded here.

SCOPE: ISM recognition only. PEM / 6D pose / geometric(depth) are OUT OF SCOPE.
"""

import argparse
import csv
import os
import statistics
import sys
import time

import cv2
import torch

# PERF-PROBE COPY: force THIS folder's yolo_ism.py copy to win the import, and
# make the probe's perf package importable. All data paths resolve to the real
# sam6d_ws root (grand-grand-parent of this file).
_COPIED_DIR = os.path.dirname(os.path.abspath(__file__))
_PROBE_ROOT = os.path.dirname(_COPIED_DIR)
_SAM6D_ROOT = os.path.dirname(_PROBE_ROOT)
if _COPIED_DIR not in sys.path:
    sys.path.insert(0, _COPIED_DIR)
if _PROBE_ROOT not in sys.path:
    sys.path.insert(1, _PROBE_ROOT)

import yolo_ism as yi  # read-only reuse of the single-object pipeline (COPY)
from perf.stage_timer import PerfCollector

# HARD guarantee: the copy must win the import (contract 3). A pre-imported
# original yolo_ism in sys.modules would silently produce wrong measurements.
assert os.path.dirname(os.path.abspath(yi.__file__)) == _COPIED_DIR, (
    f"perf-probe imported the WRONG yolo_ism: {yi.__file__}")
print(f"[perf-probe] imported yolo_ism from: {yi.__file__}", file=sys.stderr)

PERF = PerfCollector()  # disabled unless --perf-out is given

REPO_ROOT = _SAM6D_ROOT
DEFAULT_CONFIG = os.path.join(REPO_ROOT, "configs", "yolo_ism_objects.yaml")
DEFAULT_FEATURE_DIR = os.path.join(
    REPO_ROOT, "outputs", "yolo_ism_object_n", "template_features")

# Keys an object entry may override from `defaults`.
_OVERRIDABLE = (
    "weights", "device", "top_k", "score_threshold", "similarity_threshold",
    "use_mask", "appe_gate", "match_topk", "seg_weights", "stride",
    "dinov2_checkpoint",
)

CSV_FIELDS = ["frame_idx", "timestamp", "num_proposals", "best_yolo_score",
              "best_semantic_score", "best_appe_score", "selected_bbox_xyxy",
              "decision", "output_image_path", "masked_appe_score", "mask_area_px"]


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
def _abspath(p):
    return p if os.path.isabs(p) else os.path.join(REPO_ROOT, p)


def load_config(config_path):
    import yaml
    with open(config_path) as f:
        cfg = yaml.safe_load(f) or {}
    defaults = dict(cfg.get("defaults", {}))
    raw_objs = cfg.get("objects", [])
    if not isinstance(raw_objs, list):
        raise SystemExit(f"[config] `objects` must be a list in {config_path}")
    objs, seen = [], set()
    for i, raw in enumerate(raw_objs):
        if not isinstance(raw, dict):
            raise SystemExit(f"[config] objects[{i}] must be a mapping")
        if not raw.get("enabled", True):
            continue
        o = dict(defaults)            # start from defaults
        o.update(raw)                 # object keys override defaults
        for req in ("name", "yolo_prompt", "template_dir"):
            if not o.get(req):
                raise SystemExit(
                    f"[config] objects[{i}] missing required key '{req}'")
        name = o["name"]
        if name in seen:
            raise SystemExit(f"[config] duplicate object name '{name}' "
                             "(names must be unique — they key CSVs/dirs)")
        seen.add(name)
        o["template_dir"] = _abspath(o["template_dir"])
        o["cls_cache"] = _abspath(o.get(
            "cls_cache", os.path.join(DEFAULT_FEATURE_DIR, f"{name}_cls.pt")))
        o["appe_cache"] = _abspath(o.get(
            "appe_cache", os.path.join(DEFAULT_FEATURE_DIR, f"{name}_appe.pt")))
        objs.append(o)
    if not objs:
        raise SystemExit(f"[config] no enabled objects in {config_path}")
    return defaults, objs


# ---------------------------------------------------------------------------
# Per-object template features (cls + masked-appe), built once via yolo_ism
# ---------------------------------------------------------------------------
def prepare_objects(objs, model, device, rebuild):
    """Attach tcls / tappe template features; drop objects with bad templates."""
    ready = []
    for o in objs:
        tdir = o["template_dir"]
        rgbs = []
        if os.path.isdir(tdir):
            import glob
            rgbs = glob.glob(os.path.join(tdir, "rgb_*.png"))
        if not rgbs:
            print(f"[skip] {o['name']}: no rgb_*.png under {tdir}")
            continue
        try:
            tcls, _ = yi.build_template_cls(tdir, model, device, o["cls_cache"], rebuild)
            tappe, _ = yi.build_template_appe(tdir, model, device, o["appe_cache"], rebuild)
        except Exception as e:
            print(f"[skip] {o['name']}: template build failed ({e})")
            continue
        o["tcls"], o["tappe"] = tcls, tappe
        ready.append(o)
        print(f"[templates] {o['name']:20s} cls={tuple(tcls.shape)} "
              f"appe={len(tappe)} tmpl  prompt='{o['yolo_prompt']}'")
    if not ready:
        raise SystemExit("[templates] no usable objects after template loading")
    return ready


def build_prompt_groups(objs):
    """Unique prompts (for set_classes) + prompt-index -> [objects] routing."""
    unique, groups = [], {}
    for o in objs:
        p = o["yolo_prompt"]
        if p not in unique:
            unique.append(p)
        groups.setdefault(unique.index(p), []).append(o)
    return unique, groups


# ---------------------------------------------------------------------------
# Per-object ISM gate on a set of proposals (mirrors yolo_ism --use_mask path)
# ---------------------------------------------------------------------------
def recognize_frame(groups, prompt_boxes, bgr, rgb, norm_full,
                    model, device, segmentor, pool):
    """OPT2 (PERF-PROBE): 2-pass batched frame processing.

    Same gates/thresholds/decision logic as recognize(), restructured so the
    expensive models run ONCE PER FRAME instead of once per object/proposal:
      pass 1  collect every object's proposals -> ONE batched DINOv2 forward
              (frame stage `dinov2_batch`) -> per-object semantic gate
      mask    ONE MobileSAM call with ALL passing bboxes (frame stage
              `mask_batch`; batched masks pre-verified IoU=1.0 vs per-box)
      pass 2  per-object masked-appearance gate (winner patch transferred
              to CPU exactly once)
    Per-object CSV rows keep counts/scores/decisions; their per-stage timing
    fields are 0 with note=opt2_batched — batched costs live in the frame
    stages dinov2_batch / mask_batch. Returns {object_name: res}.
    """
    states = []   # one per object, in groups iteration order
    entries = []  # (state, box, conf, rank, crop) across ALL objects
    for pi, objs_here in groups.items():
        cand = sorted(prompt_boxes.get(pi, []), key=lambda t: t[1], reverse=True)
        for o in objs_here:
            sel = [(b, s) for (b, s) in cand
                   if s >= float(o.get("score_threshold", 0.0))]
            sel = sel[:int(o.get("top_k", 3))]
            res = {"num_proposals": len(sel),
                   "best_yolo": max((s for _, s in sel), default=0.0),
                   "best_sem": 0.0, "appe": 0.0, "masked_appe": 0.0,
                   "mask_area": 0, "box": None, "mask": None, "rank": 0,
                   "decision": "no-object(no-proposal)", "accepted": False}
            st = {"o": o, "res": res, "cand_n": len(cand), "best": None,
                  "ncrop": 0}
            states.append(st)
            for rank, (box, sc) in enumerate(sel, start=1):
                crop = yi.crop_resize_pad(norm_full, box)
                if crop is None:
                    continue
                st["ncrop"] += 1
                entries.append((st, box, sc, rank, crop))

    # ---- ONE DINOv2 forward for the whole frame's crops ----
    patch_all = None
    if entries:
        with PERF.frame_stage("dinov2_batch"):
            cls_all, patch_all = yi.dinov2_forward_batch(
                model, [e[4] for e in entries], device)
        for i, (st, box, sc, rank, crop) in enumerate(entries):
            sem = yi.semantic_score(cls_all[i], st["o"]["tcls"],
                                    st["o"]["match_topk"])
            if st["best"] is None or sem > st["best"][0]:
                st["best"] = (sem, box, sc, rank, crop, cls_all[i], i)

    # ---- semantic gate (identical thresholds/ordering) ----
    passing = []
    for st in states:
        o, res, best = st["o"], st["res"], st["best"]
        if res["num_proposals"] == 0:
            continue
        res["best_sem"] = best[0] if best else 0.0
        if not (best and res["best_sem"] >= o["similarity_threshold"]):
            res["decision"] = "no-object(below-sim)"
            continue
        res["box"], res["rank"] = best[1], best[3]
        passing.append(st)

    # ---- ONE MobileSAM call for ALL passing bboxes ----
    mask_states = [st for st in passing if st["o"].get("use_mask", True)]
    masks = [None] * len(mask_states)
    if mask_states:
        with PERF.frame_stage("mask_batch"):
            masks = yi.segment_boxes(
                segmentor, bgr, [st["best"][1] for st in mask_states], device)
    mask_of = {id(st): m for st, m in zip(mask_states, masks)}

    # ---- pass 2: appearance gate per object ----
    for st in passing:
        o, res, best = st["o"], st["res"], st["best"]
        qpatch = patch_all[best[6]].cpu()   # winner-only device->host copy
        if not o.get("use_mask", True):
            res["appe"] = yi.appearance_score(qpatch, o["tappe"],
                                              o["match_topk"])
            res["decision"], res["accepted"] = "detected", True
            continue
        mask = mask_of[id(st)]
        x1, y1, x2, y2 = best[1]
        res["mask"] = mask
        res["mask_area"] = int(mask[y1:y2, x1:x2].sum()) if mask is not None else 0
        q_fg = yi.masked_query_patches(qpatch, mask, best[1], pool)[0] \
            if mask is not None else qpatch
        st["best_t"] = int(torch.argmax(o["tcls"] @ best[5]))
        res["masked_appe"] = yi.masked_appe_score(q_fg, o["tappe"][st["best_t"]])
        if res["masked_appe"] >= o["appe_gate"]:
            res["decision"], res["accepted"] = "detected", True
        else:
            res["decision"] = "no-object(below-appe)"

    # ---- per-object CSV rows (counts/scores/decisions; timing is batched) ----
    for st in states:
        o, res = st["o"], st["res"]
        PERF.obj_begin(o.get("_perf_index", -1), o["name"])
        PERF.obj_set(detection_count_for_object=st["cand_n"],
                     proposal_count_for_object=res["num_proposals"],
                     template_count_for_object=int(o["tcls"].shape[0]),
                     dinov2_forward_count=st["ncrop"],
                     selected_template_id=st.get("best_t", -1))
        PERF.obj_note("opt2_batched")
        if (o.get("use_mask", True) and res["mask"] is None
                and res["decision"] in ("detected", "no-object(below-appe)")):
            PERF.obj_note("mask_none")
        PERF.obj_end(res)

    return {st["o"]["name"]: st["res"] for st in states}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(
        description="Config-driven multi-object YOLO-World x SAM-6D ISM recognition.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--config", default=DEFAULT_CONFIG)
    p.add_argument("--bag", default="data/ros2_bag/two_table_around")
    p.add_argument("--frames-dir", default="")
    # PERF-PROBE COPY: overlays/CSVs go INSIDE the probe folder so the original
    # outputs/yolo_ism_object_n results are never touched or overwritten.
    p.add_argument("--output-root",
                   default=os.path.join(_PROBE_ROOT, "outputs", "algo_out"))
    p.add_argument("--topic", default="/camera/camera/color/image_raw")
    p.add_argument("--stride", type=int, default=0,
                   help="override config stride (0 = use config defaults.stride)")
    p.add_argument("--max-frames", type=int, default=0, help="0 = no cap")
    p.add_argument("--rebuild-features", action="store_true")
    p.add_argument("--device", default="")
    # ---- perf probe options (default OFF: identical behavior to original) ----
    p.add_argument("--perf-out", default="",
                   help="enable timing instrumentation; CSVs are written to "
                        "<perf-out>/<run_id>/ (recommended: "
                        "_perf_bottleneck_probe_2026_07_06/outputs)")
    p.add_argument("--perf-warmup-frames", type=int, default=3,
                   help="first N frames flagged is_warmup=1 and excluded from "
                        "stage_summary statistics")
    p.add_argument("--perf-run-id", default="",
                   help="run id (default: <bag>_<YYYYmmdd-HHMMSS>)")
    p.add_argument("--no-save-images", action="store_true",
                   help="skip overlay/combined imwrite (Validation V5: isolate "
                        "disk-IO cost; CSV rows are still written)")
    return p.parse_args()


def _timed_frames(gen):
    """Yield (frame_input_ms, item) measuring time spent INSIDE the frame
    generator (disk read / bag decode / stride skipping)."""
    it = iter(gen)
    while True:
        t0 = time.perf_counter()
        try:
            item = next(it)
        except StopIteration:
            return
        yield (time.perf_counter() - t0) * 1e3, item


def main():
    args = parse_args()
    defaults, objs = load_config(args.config)

    # PERF-PROBE: never WRITE template-feature caches outside the probe folder.
    # Existing original caches are still read (warm reuse); a cache miss or
    # --rebuild-features writes into the probe's own outputs/ instead.
    _probe_feat = os.path.join(_PROBE_ROOT, "outputs", "template_features")
    for o in objs:
        for k in ("cls_cache", "appe_cache"):
            if args.rebuild_features or not os.path.isfile(o[k]):
                o[k] = os.path.join(_probe_feat, os.path.basename(o[k]))

    device = args.device or defaults.get("device", "cuda:0")
    device = device if torch.cuda.is_available() else "cpu"
    stride = args.stride or int(defaults.get("stride", 10))
    bag_name = os.path.basename(os.path.normpath(args.bag))
    out_dir = os.path.join(args.output_root, bag_name)
    os.makedirs(out_dir, exist_ok=True)
    combined_dir = os.path.join(out_dir, "combined")
    os.makedirs(combined_dir, exist_ok=True)

    if args.perf_out:
        run_id = args.perf_run_id or f"{bag_name}_{time.strftime('%Y%m%d-%H%M%S')}"
        PERF.configure(args.perf_out, run_id, args.perf_warmup_frames, device,
                       extra_frame_stages=["dinov2_batch", "mask_batch"])
        print(f"[perf] enabled -> {PERF.out_dir}")
        # minimal meta first — identifies the run even if model loading crashes;
        # overwritten with the full meta below once everything is loaded
        PERF.write_run_meta({"run_id": run_id, "status": "loading",
                             "argv": sys.argv, "device": device})

    ckpt = defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT
    print(f"[dinov2] loading {ckpt}")
    model = yi.build_dinov2(ckpt, device)
    objs = prepare_objects(objs, model, device, args.rebuild_features)
    unique_prompts, groups = build_prompt_groups(objs)
    for _i, _o in enumerate(objs):
        _o["_perf_index"] = _i

    any_mask = any(o.get("use_mask", True) for o in objs)
    segmentor = pool = None
    if any_mask:
        seg_w = _abspath(defaults.get("seg_weights", "mobile_sam.pt"))
        print(f"[seg] loading {seg_w}")
        segmentor = yi.build_segmentor(seg_w, device)
        pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)

    from ultralytics import YOLOWorld
    weights = defaults.get("weights", "yolov8m-worldv2.pt")
    try:
        yolo = YOLOWorld(weights)
    except Exception as e:
        print(f"[yolo] {weights} failed ({e}); falling back to yolov8s-worldv2.pt")
        yolo = YOLOWorld("yolov8s-worldv2.pt")
    yolo.set_classes(unique_prompts)
    print(f"[yolo] {len(unique_prompts)} unique prompts -> {len(objs)} objects: "
          f"{unique_prompts}")

    # single shared YOLO pass uses the LOWEST per-object threshold so no object
    # is starved; each object re-applies its own score_threshold/top_k in recognize().
    min_score = min(float(o.get("score_threshold", 0.02)) for o in objs)
    img_sz = int(defaults.get("imgsz", 640))   # YOLO-World inference resolution (config-driven)

    if PERF.enabled:
        import datetime
        import yaml as _yaml
        with open(args.config) as _f:
            _cfg_snapshot = _yaml.safe_load(_f)
        try:
            import ultralytics
            _ul_ver = ultralytics.__version__
        except Exception:
            _ul_ver = "unknown"
        PERF.write_run_meta({
            "run_id": PERF.run_id,
            "status": "running",
            "created_at": datetime.datetime.now().isoformat(),
            "python_version": sys.version,
            "torch_version": torch.__version__,
            "ultralytics_version": _ul_ver,
            "gpu_name": (torch.cuda.get_device_name(0)
                         if torch.cuda.is_available() else "none"),
            "cuda_version": getattr(torch.version, "cuda", None),
            "argv": sys.argv,
            "working_directory": os.getcwd(),
            "original_project_root": REPO_ROOT,
            "copied_script_path": os.path.abspath(__file__),
            "copied_yolo_ism_path": os.path.join(_COPIED_DIR, "yolo_ism.py"),
            "actual_imported_yolo_ism_file": yi.__file__,
            "device": device,
            "cuda_available": torch.cuda.is_available(),
            "perf_enabled": True,
            "perf_warmup_frames": args.perf_warmup_frames,
            "config_path": os.path.abspath(args.config),
            "frame_source": {"bag": args.bag, "frames_dir": args.frames_dir,
                             "topic": args.topic, "stride": stride,
                             "max_frames": args.max_frames},
            "object_prompt_count": len(unique_prompts),
            "enabled_object_names": [o["name"] for o in objs],
            "imgsz": img_sz,
            "top_k": defaults.get("top_k"),
            "score_threshold": defaults.get("score_threshold"),
            "similarity_threshold": defaults.get("similarity_threshold"),
            "appe_gate": defaults.get("appe_gate"),
            "notes": [
                "This profiling run uses copied files only. Original project "
                "files were not modified.",
                "CUDA timing uses torch.cuda.synchronize when CUDA is enabled; "
                "measured runtime may include instrumentation overhead.",
            ],
            "config_snapshot": _cfg_snapshot,
        })

    per_obj_rows = {o["name"]: [] for o in objs}
    per_obj_count = {o["name"]: 0 for o in objs}
    summary_rows = []
    n_frames = 0
    template_count_total = sum(int(o["tcls"].shape[0]) for o in objs)  # constant

    for input_ms, (frame_idx, name, bgr) in _timed_frames(yi.resolve_frames(
            args.bag, args.frames_dir, max(1, stride), args.max_frames, args.topic)):
        n_frames += 1
        h, w = bgr.shape[:2]
        PERF.frame_begin(frame_idx, name)
        PERF.frame_add_input_ms(input_ms)
        with PERF.frame_stage("preprocess"):
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            # improvement (4): hoist — normalize the full frame ONCE and share
            # it across all objects (was recomputed inside every recognize()).
            norm_full = yi.normalize_rgb(rgb)

        with PERF.frame_stage("yolo_world"):
            res = yolo.predict(bgr, conf=min_score, imgsz=img_sz, verbose=False,
                               device=device)
        # boxes grouped by predicted prompt index
        detection_count = 0
        with PERF.frame_stage("box_routing"):
            prompt_boxes = {i: [] for i in range(len(unique_prompts))}
            if len(res) and res[0].boxes is not None and len(res[0].boxes) > 0:
                b = res[0].boxes
                detection_count = len(b)
                for j in range(len(b)):
                    xy = b.xyxy[j].tolist()
                    x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                    x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                    cls_i = int(b.cls[j]) if b.cls is not None else 0
                    if cls_i in prompt_boxes:   # ignore any out-of-range class id
                        prompt_boxes[cls_i].append(([x1, y1, x2, y2], float(b.conf[j])))

        with PERF.frame_stage("output_packaging"):
            combined = None if args.no_save_images else bgr.copy()
        accepted_names = []
        proposal_count_total = 0
        # OPT2: one batched 2-pass call replaces the per-object recognize loop
        with PERF.frame_stage("recognize_total"):
            results = recognize_frame(groups, prompt_boxes, bgr, rgb,
                                      norm_full, model, device, segmentor, pool)
        for pi, objs_here in groups.items():
            for o in objs_here:
                r = results[o["name"]]
                proposal_count_total += r["num_proposals"]
                with PERF.frame_stage("output_packaging"):
                    out_path = ""
                    if r["accepted"]:
                        per_obj_count[o["name"]] += 1
                        accepted_names.append(o["name"])
                        if not args.no_save_images:
                            obj_dir = os.path.join(out_dir, o["name"])
                            os.makedirs(obj_dir, exist_ok=True)
                            out_path = os.path.join(obj_dir, f"frame_{frame_idx:06d}.jpg")
                            ov = yi.draw_result(bgr, r["box"], r["rank"], r["best_yolo"],
                                                r["best_sem"], r["masked_appe"] or r["appe"])
                            if r["mask"] is not None:
                                cnts, _ = cv2.findContours(r["mask"].astype("uint8"),
                                                           cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                                cv2.drawContours(ov, cnts, -1, (0, 200, 0), 2)
                            cv2.imwrite(out_path, ov)
                            # annotate object name onto the combined overlay
                            bx = r["box"]
                            cv2.rectangle(combined, (bx[0], bx[1]), (bx[2], bx[3]), (0, 200, 0), 2)
                            cv2.putText(combined, o["name"], (bx[0] + 2, max(12, bx[1] - 4)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1, cv2.LINE_AA)
                    per_obj_rows[o["name"]].append(
                        [frame_idx, name, r["num_proposals"], round(r["best_yolo"], 4),
                         round(r["best_sem"], 4), round(r["appe"], 4),
                         ";".join(str(v) for v in r["box"]) if r["box"] else "",
                         r["decision"], out_path, round(r["masked_appe"], 4), r["mask_area"]])
        with PERF.frame_stage("output_packaging"):
            if not args.no_save_images:
                cv2.imwrite(os.path.join(combined_dir, f"frame_{frame_idx:06d}.jpg"), combined)
            summary_rows.append([frame_idx, name, len(accepted_names),
                                 ";".join(accepted_names)])
        PERF.frame_end(image_width=w, image_height=h,
                       object_prompt_count=len(unique_prompts),
                       detection_count=detection_count,
                       proposal_count_total=proposal_count_total,
                       template_count_total=template_count_total)

    # write per-object CSVs
    for o in objs:
        with open(os.path.join(out_dir, f"{o['name']}_results.csv"), "w", newline="") as f:
            wcsv = csv.writer(f); wcsv.writerow(CSV_FIELDS)
            wcsv.writerows(per_obj_rows[o["name"]])
    # combined summary
    with open(os.path.join(out_dir, "multiobject_summary.csv"), "w", newline="") as f:
        wcsv = csv.writer(f)
        wcsv.writerow(["frame_idx", "timestamp", "num_accepted", "accepted_objects"])
        wcsv.writerows(summary_rows)

    print("\n===== SUMMARY =====")
    print(f"bag                 : {bag_name}")
    print(f"frames processed    : {n_frames}")
    print(f"objects (enabled)   : {len(objs)}")
    print(f"{'object':22s} {'accepted':>8s} / frames")
    for o in objs:
        print(f"{o['name']:22s} {per_obj_count[o['name']]:>8d} / {n_frames}")
    total_acc = sum(per_obj_count.values())
    print(f"total accepted (obj-frames): {total_acc}")
    print(f"out : {out_dir}")
    PERF.finalize()


if __name__ == "__main__":
    main()
