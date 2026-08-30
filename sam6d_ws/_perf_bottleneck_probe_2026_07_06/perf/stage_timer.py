"""stage_timer.py — minimal-invasive timing collector for the perf probe.

Default state is DISABLED: every method is a cheap no-op, so the copied
pipeline behaves identically to the original when --perf-out is not given.

When enabled (configure()):
  * frame stages   -> per_frame_timing.csv   (one row per frame)
  * object stages  -> per_object_timing.csv  (one row per recognize() call)
  * finalize()     -> stage_summary.csv (warm-up excluded) + run_meta.json

CUDA timing rule: when a CUDA device is used, torch.cuda.synchronize() is
called on stage enter AND exit so time.perf_counter() brackets actual GPU
work (async kernel launches would otherwise be missed).  This adds overhead:
measured totals may exceed an un-instrumented run.
"""

import csv
import json
import os
import statistics
import time
from contextlib import contextmanager

PER_FRAME_FIELDS = [
    "run_id", "frame_index", "stamp", "is_warmup", "image_width",
    "image_height", "object_prompt_count", "detection_count",
    "proposal_count_total", "template_count_total",
    "stage_frame_input_ms", "stage_preprocess_ms", "stage_yolo_world_ms",
    "stage_box_routing_ms", "stage_recognize_total_ms",
    "stage_output_packaging_ms", "total_until_template_selection_ms",
    "total_frame_ms", "gpu_memory_allocated_mb", "gpu_memory_reserved_mb",
    "notes",
]

PER_OBJECT_FIELDS = [
    "run_id", "frame_index", "stamp", "is_warmup", "object_index",
    "object_name", "detection_count_for_object", "proposal_count_for_object",
    "template_count_for_object", "normalize_rgb_ms", "crop_ms",
    "dinov2_cls_ms", "dinov2_forward_count", "semantic_score_ms",
    "dinov2_patch_ms", "mask_ms", "appearance_score_ms", "recognize_total_ms",
    "selected_template_id", "best_sem", "masked_appe", "decision", "accepted",
    "notes",
]

SUMMARY_FIELDS = [
    "run_id", "stage_name", "count", "mean_ms", "median_ms", "p90_ms",
    "p95_ms", "max_ms", "min_ms", "std_ms", "percent_of_total_mean",
    "bottleneck_rank",
]

FRAME_STAGES = ["frame_input", "preprocess", "yolo_world", "box_routing",
                "recognize_total", "output_packaging"]
OBJ_STAGES = ["normalize_rgb", "crop", "dinov2_cls", "semantic_score",
              "dinov2_patch", "mask", "appearance_score"]

# recognize() decision string -> CSV decision vocabulary
DECISION_MAP = {
    "no-object(no-proposal)": "no_proposal",
    "no-object(below-sim)": "below_semantic",
    "no-object(below-appe)": "below_appearance",
    "detected": "detected",
}


def _pctl(sorted_vals, q):
    """Percentile via rounded rank on a (n-1) scale, pre-sorted input.
    With small n this is coarse (n=3 -> p90 == p95 == max)."""
    if not sorted_vals:
        return 0.0
    k = max(0, min(len(sorted_vals) - 1,
                   int(round(q / 100.0 * (len(sorted_vals) - 1)))))
    return sorted_vals[k]


class PerfCollector:
    """Disabled by default; configure() turns it on."""

    def __init__(self):
        self.enabled = False
        self._sync = lambda: None

    # ------------------------------------------------------------------
    def configure(self, out_root, run_id, warmup_frames, device,
                  extra_frame_stages=()):
        """extra_frame_stages: additional frame-level stage names (e.g. the
        opt2 batched pipeline adds 'dinov2_batch'/'mask_batch'). They are
        appended to the per-frame CSV before 'notes' and summarized like any
        other stage; they do NOT enter total_until_template_selection (they
        are sub-splits nested inside recognize_total)."""
        import atexit
        import torch
        self.enabled = True
        self._finalized = False
        atexit.register(self.finalize)  # summary survives a mid-run crash
        self.run_id = run_id
        self.warmup_frames = int(warmup_frames)
        self.device = str(device)
        self._cuda = self.device.startswith("cuda") and torch.cuda.is_available()
        self._torch = torch
        if self._cuda:
            self._sync = torch.cuda.synchronize
        self.frame_stages = FRAME_STAGES + [s for s in extra_frame_stages
                                            if s not in FRAME_STAGES]
        self.frame_fields = (PER_FRAME_FIELDS[:-1]
                             + [f"stage_{s}_ms" for s in self.frame_stages
                                if s not in FRAME_STAGES]
                             + PER_FRAME_FIELDS[-1:])
        self.out_dir = os.path.join(out_root, run_id)
        os.makedirs(self.out_dir, exist_ok=True)

        self._frame_rows = []      # kept for summary
        self._obj_rows = []
        self._frame = None         # current frame record
        self._obj = None           # current object record
        self._n_frames = 0

        self._ff = open(os.path.join(self.out_dir, "per_frame_timing.csv"),
                        "w", newline="")
        self._fw = csv.DictWriter(self._ff, fieldnames=self.frame_fields)
        self._fw.writeheader()
        self._of = open(os.path.join(self.out_dir, "per_object_timing.csv"),
                        "w", newline="")
        self._ow = csv.DictWriter(self._of, fieldnames=PER_OBJECT_FIELDS)
        self._ow.writeheader()

    def write_run_meta(self, meta):
        if not self.enabled:
            return
        path = os.path.join(self.out_dir, "run_meta.json")
        with open(path, "w") as f:
            json.dump(meta, f, indent=2, default=str)

    # ------------------------------------------------------------------ frame
    def frame_begin(self, frame_index, stamp):
        if not self.enabled:
            return
        self._n_frames += 1
        self._frame = {
            "run_id": self.run_id, "frame_index": frame_index, "stamp": stamp,
            "is_warmup": 1 if self._n_frames <= self.warmup_frames else 0,
            "notes": "",
        }
        for s in self.frame_stages:
            self._frame[f"stage_{s}_ms"] = 0.0
        self._frame["_objsum"] = {s: 0.0 for s in OBJ_STAGES}
        self._sync()
        self._frame["_t0"] = time.perf_counter()

    @contextmanager
    def frame_stage(self, name):
        """Accumulating timer: may be entered several times per frame."""
        if not self.enabled or self._frame is None:
            yield
            return
        self._sync()
        t0 = time.perf_counter()
        try:
            yield
        finally:
            self._sync()
            self._frame[f"stage_{name}_ms"] += (time.perf_counter() - t0) * 1e3

    def frame_add_input_ms(self, ms):
        """frame_input is timed OUTSIDE frame_begin (generator wrapper)."""
        if self.enabled and self._frame is not None:
            self._frame["stage_frame_input_ms"] += ms

    def frame_end(self, **fields):
        if not self.enabled or self._frame is None:
            return
        self._sync()
        body_ms = (time.perf_counter() - self._frame.pop("_t0")) * 1e3
        f = self._frame
        f.update(fields)
        f["total_frame_ms"] = round(body_ms + f["stage_frame_input_ms"], 3)
        f["total_until_template_selection_ms"] = round(
            sum(f[f"stage_{s}_ms"] for s in
                ("frame_input", "preprocess", "yolo_world", "box_routing",
                 "recognize_total")), 3)
        if self._cuda:
            f["gpu_memory_allocated_mb"] = round(
                self._torch.cuda.memory_allocated() / 1e6, 1)
            f["gpu_memory_reserved_mb"] = round(
                self._torch.cuda.memory_reserved() / 1e6, 1)
        else:
            f["gpu_memory_allocated_mb"] = f["gpu_memory_reserved_mb"] = 0
        for s in self.frame_stages:
            f[f"stage_{s}_ms"] = round(f[f"stage_{s}_ms"], 3)
        self._frame_rows.append(dict(f))
        self._fw.writerow({k: f.get(k, "") for k in self.frame_fields})
        self._ff.flush()   # partial results survive a crash
        self._frame = None

    # ----------------------------------------------------------------- object
    def obj_begin(self, object_index, object_name):
        if not self.enabled or self._frame is None:
            return
        self._obj = {
            "run_id": self.run_id,
            "frame_index": self._frame["frame_index"],
            "stamp": self._frame["stamp"],
            "is_warmup": self._frame["is_warmup"],
            "object_index": object_index, "object_name": object_name,
            "dinov2_forward_count": 0, "selected_template_id": -1,
            "notes": "",
        }
        for s in OBJ_STAGES:
            self._obj[f"{s}_ms"] = 0.0
        self._sync()
        self._obj["_t0"] = time.perf_counter()

    @contextmanager
    def obj_stage(self, name):
        if not self.enabled or self._obj is None:
            yield
            return
        self._sync()
        t0 = time.perf_counter()
        try:
            yield
        finally:
            self._sync()
            self._obj[f"{name}_ms"] += (time.perf_counter() - t0) * 1e3

    def obj_count(self, key, n=1):
        if self.enabled and self._obj is not None:
            self._obj[key] = self._obj.get(key, 0) + n

    def obj_set(self, **kv):
        if self.enabled and self._obj is not None:
            self._obj.update(kv)

    def obj_note(self, note):
        if self.enabled and self._obj is not None:
            self._obj["notes"] = (self._obj["notes"] + ";" + note).strip(";")

    def obj_end(self, res):
        """res = the recognize() result dict (unchanged original semantics)."""
        if not self.enabled or self._obj is None:
            return
        self._sync()
        o = self._obj
        o["recognize_total_ms"] = round(
            (time.perf_counter() - o.pop("_t0")) * 1e3, 3)
        raw_decision = res.get("decision")
        o["decision"] = DECISION_MAP.get(raw_decision, "error")
        if raw_decision not in DECISION_MAP:
            self.obj_note(f"raw_decision={raw_decision}")
        o["accepted"] = 1 if res.get("accepted") else 0
        o["best_sem"] = round(float(res.get("best_sem", 0.0)), 4)
        o["masked_appe"] = round(float(res.get("masked_appe", 0.0)), 4)
        for s in OBJ_STAGES:
            o[f"{s}_ms"] = round(o[f"{s}_ms"], 3)
            self._frame["_objsum"][s] += o[f"{s}_ms"]
        self._obj_rows.append(dict(o))
        self._ow.writerow({k: o.get(k, "") for k in PER_OBJECT_FIELDS})
        self._of.flush()
        self._obj = None

    # --------------------------------------------------------------- summary
    def finalize(self):
        if not self.enabled or self._finalized:
            return
        self._finalized = True
        rows = [r for r in self._frame_rows if not r["is_warmup"]]
        summary = []
        total_mean = (statistics.fmean([r["total_frame_ms"] for r in rows])
                      if rows else 0.0)

        def add(stage, vals):
            if not vals:
                return
            sv = sorted(vals)
            mean = statistics.fmean(vals)
            summary.append({
                "run_id": self.run_id, "stage_name": stage,
                "count": len(vals), "mean_ms": round(mean, 3),
                "median_ms": round(statistics.median(vals), 3),
                "p90_ms": round(_pctl(sv, 90), 3),
                "p95_ms": round(_pctl(sv, 95), 3),
                "max_ms": round(sv[-1], 3), "min_ms": round(sv[0], 3),
                "std_ms": round(statistics.pstdev(vals), 3)
                if len(vals) > 1 else 0.0,
                "percent_of_total_mean": round(100.0 * mean / total_mean, 2)
                if total_mean else 0.0,
            })

        for s in self.frame_stages:
            add(s, [r[f"stage_{s}_ms"] for r in rows])
        # object sub-stages aggregated per frame (sum over objects in a frame)
        for s in OBJ_STAGES:
            add(f"obj:{s}", [r["_objsum"][s] for r in rows])
        add("total_frame", [r["total_frame_ms"] for r in rows])

        # recognize_total is the umbrella over all obj:* sub-stages — ranking it
        # alongside them double-counts and always wins; exclude both umbrellas.
        rankable = [x for x in summary
                    if x["stage_name"] not in ("total_frame", "recognize_total")]
        for rank, x in enumerate(
                sorted(rankable, key=lambda x: -x["percent_of_total_mean"]),
                start=1):
            x["bottleneck_rank"] = rank
        for x in summary:
            x.setdefault("bottleneck_rank", "")

        with open(os.path.join(self.out_dir, "stage_summary.csv"),
                  "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS)
            w.writeheader()
            w.writerows(summary)
        self._ff.close()
        self._of.close()
        print(f"[perf] wrote {self.out_dir} "
              f"({len(self._frame_rows)} frames, {len(self._obj_rows)} object rows, "
              f"warm-up excluded from summary: {self.warmup_frames})")
