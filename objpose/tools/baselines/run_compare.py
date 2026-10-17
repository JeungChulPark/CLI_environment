#!/usr/bin/env python3
"""run_compare.py — proposed pipeline vs ORIGINAL SAM-6D on YCB-V / LM-O, same frames, same PC.

  --method   proposed   improved SAM-6D of this thesis (YOLO-World + MobileSAM + verification),
                        the exact core used by objpose/tools/ycbv_sam6d/run_ycbv_sam6d.py
             orig       untouched SAM-6D (FastSAM ISM + PEM), objpose/tools/baselines/orig_sam6d.py
  --mode     bop19      only the BOP-2019 test images of the scene (test_targets_bop19.json),
                        one by one, no clock; also writes a BOP submission CSV
             all        every frame in order (offline upper bound)
             still      every frame of the scene as an independent still image (no temporal mask reuse)
             realtime   30 fps wall clock, newest arrived frame only (as run_ycbv_sam6d.py)

Both methods get the same object list per scene: the objects annotated in the scene (the BOP
6D-localisation task gives the targets). Output per scene (same schema as run_ycbv_sam6d.py):
  <out>/<scene>/sam6d_estimates.jsonl, sam6d_frames.jsonl, run.json   and, in bop19 mode,
  <out>/bop_<method>_<dataset>.csv  (scene_id,im_id,obj_id,score,R,t,time)

    conda run -n sam6d python objpose/tools/baselines/run_compare.py --method orig --dataset ycbv \
        --mode bop19 --out objpose/output/cmp_orig_ycbv_bop19
"""
from __future__ import annotations

import argparse
import copy
import json, os
import sys
import time
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[3]
SAM6D = REPO / "sam6d_realtime"
sys.path.insert(0, str(REPO / "objpose" / "pc"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from bop_session import BopSession            # noqa: E402

DATA = {
    "ycbv": {"table": REPO / "objpose/rt/bop/ycbv_objects.json", "ism": "configs/yolo_ism_ycbv.yaml",
             "targets": Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video/test_targets_bop19.json")},
    "hope": {"table": REPO / "objpose/rt/bop/hope_objects.json", "ism": "configs/yolo_ism_hope.yaml", "targets": None},
    "ycbm": {"table": REPO / "objpose/rt/bop/ycbm_objects.json", "ism": "configs/yolo_ism_ycbv.yaml", "targets": None},
    "real": {"table": REPO / "objpose/rt/bop/real_objects.json", "ism": "configs/yolo_ism_objects.yaml", "targets": None},
    "obchange": {"table": REPO / "objpose/rt/bop/obchange_objects.json", "ism": "configs/yolo_ism_ycbv.yaml", "targets": None},
    "lmo": {"table": REPO / "objpose/rt/bop/lmo_objects.json", "ism": "configs/yolo_ism_lmo.yaml",
            "targets": Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/bop/lmo/test_targets_bop19.json")},
}
FAST = {"fine_npoint": 1024, "mask_cache": {"enabled": True, "iou_thresh": 0.90, "max_age": 5, "interval": 30}}


def scene_targets(scene: Path, table: dict) -> list[str]:
    gt = json.loads((scene / "scene_gt.json").read_text())
    name = {o["obj_id"]: o["name"] for o in table["objects"]}
    return [name[i] for i in sorted({r["obj_id"] for v in gt.values() for r in v}) if i in name]


class _Boxes:
    def __init__(self, xyxy, cls):
        import torch
        self.xyxy = torch.tensor(xyxy, dtype=torch.float32).reshape(-1, 4)
        self.cls = torch.tensor(cls, dtype=torch.float32)
        self.conf = torch.ones(len(cls))
    def __len__(self):
        return len(self.cls)


class _Res:
    def __init__(self, boxes): self.boxes = boxes


class GTBoxDetector:
    """Stands in for YOLO-World: returns the frame's ground-truth boxes, each under the prompt
    index of its own object (conf 1.0). Everything after the detector is untouched."""
    def __init__(self, core):
        self.core = core
        self.pidx = {o["name"]: core.unique_prompts.index(o["yolo_prompt"]) for o in core.objs}
    def predict(self, *a, **k):
        bx = [(self.pidx[n], b) for n, b in getattr(self.core, "gt_boxes", []) if n in self.pidx]
        return [_Res(_Boxes([b for _, b in bx], [c for c, _ in bx]))]
    def set_classes(self, *a, **k):
        pass


def gt_boxes_of(scene: Path, table: dict):
    """frame position -> [(asset name, [x1,y1,x2,y2])] for instances with visib_fract >= 0.1"""
    gt = json.loads((scene / "scene_gt.json").read_text()); info = json.loads((scene / "scene_gt_info.json").read_text())
    name = {o["obj_id"]: o["name"] for o in table["objects"]}
    out = {}
    for fid in gt:
        rows = []
        for r, i in zip(gt[fid], info[fid]):
            x, y, w, h = i.get("bbox_visib", [0, 0, 0, 0])
            if i.get("visib_fract", 1.0) >= 0.1 and w > 0 and h > 0 and r["obj_id"] in name:
                rows.append((name[r["obj_id"]], [x, y, x + w, y + h]))
        out[int(fid)] = rows
    return out


def build(method: str, dataset: str, objects: list[str], table: dict, bop19: bool, gt_boxes=False):
    if method.startswith("orig"):
        from orig_sam6d import OrigSam6D
        bb = "vits14" if method == "orig_vits" else "vitl14"
        core = OrigSam6D(objects, table, backbone=bb, gt_boxes=gt_boxes)
        return core, {"method": method, "segmentor": "mobilesam(gt box)" if gt_boxes else "fastsam", "dinov2": bb, "objects": objects}
    sys.path.insert(0, str(SAM6D / "realtime"))
    import verify_config as VC
    from sam6d_core import Sam6DCore
    cfg = yaml.safe_load((SAM6D / "realtime" / "run_split_example.yaml").read_text(encoding="utf-8"))
    rt = cfg["runtime"]
    if method != "proposed_noverify_fullpts":
        rt.update(copy.deepcopy(FAST))
    if method in ("proposed_noverify", "proposed_noverify_fullpts"):
        rt["verify"] = None            # geometry-only PEM output (no pose verification)
    if method == "proposed_score":     # BOP-style: every answer kept, verification only ranks it
        v = rt.get("verify", VC.UNSET)
        rt["verify"] = dict(v if isinstance(v, dict) else {}, as_score=True)
    if bop19:   # single still images: no temporal mask reuse
        rt["mask_cache"] = {"enabled": False}
    ism = DATA[dataset]["ism"]
    if os.environ.get("VERIFY_OVR") and rt.get("verify", VC.UNSET) is not None:     # tuning on validation frames
        v = rt.get("verify", VC.UNSET)
        rt["verify"] = dict(v if isinstance(v, dict) else {}, **json.loads(os.environ["VERIFY_OVR"]))
    if os.environ.get("ISM_OVR") or os.environ.get("PROMPT_OVR"):
        c0 = yaml.safe_load((SAM6D / ism).read_text(encoding="utf-8"))
        c0["defaults"].update(json.loads(os.environ.get("ISM_OVR", "{}")))
        po = json.loads(os.environ.get("PROMPT_OVR", "{}"))
        for o in c0["objects"]:
            if o["name"] in po:
                o["yolo_prompt"] = po[o["name"]]
        ism = f"configs/_tune_{os.getpid()}.yaml"
        (SAM6D / ism).write_text(yaml.safe_dump(c0, allow_unicode=True), encoding="utf-8")
    if method in ("proposed_nogate", "proposed_noverify", "proposed_noverify_fullpts", "proposed_score"):   # ablation: candidate-verification gates off, pose verification on
        ism = str(Path(__file__).resolve().parent / "configs" / f"yolo_ism_{dataset}_nogate.yaml")
    core = Sam6DCore(ism, objects, rt.get("device", "cuda:0"), rt.get("det_score_thresh", 0.2),
                     appe_rerank=rt.get("appe_rerank"), verify=rt.get("verify", VC.UNSET),
                     precision=rt.get("precision"), mask_cache=rt.get("mask_cache"),
                     pem_schedule=rt.get("pem_schedule"), coarse_npoint_override=rt.get("coarse_npoint"),
                     fine_npoint_override=rt.get("fine_npoint"))
    if gt_boxes:
        core.yolo = GTBoxDetector(core)
    if method == "proposed_score":
        # candidate-verification thresholds of the ORIGINAL config, applied after the fact
        import yolo_ism_object_n as o_n
        _, objs0 = o_n.load_config(str(SAM6D / DATA[dataset]["ism"]))
        core.gate_thr = {o["name"]: (float(o["similarity_threshold"]), o_n._appe_gate_of(o)) for o in objs0}
    return core, {"method": method, "ism_config": ism, "objects": objects, "runtime": rt, "gt_boxes": gt_boxes}


def run_scene(method, dataset, scene: Path, mode, out: Path, csv_rows: list, max_frames=0, all_objects=False, gt_boxes=False, stride=1, offset=0, exclude_targets=False, resume=False, stop_at=0.0):
    table = json.loads(DATA[dataset]["table"].read_text())
    oid = {o["name"]: o["obj_id"] for o in table["objects"]}
    out.mkdir(parents=True, exist_ok=True)
    sess = BopSession(scene, fps=float(table.get("fps", 30.0)))
    K = sess.K
    objects = [o['name'] for o in table['objects']] if all_objects else scene_targets(scene, table)
    core, used = build(method, dataset, objects, table, mode in ("bop19", "still"), gt_boxes)
    gtb = gt_boxes_of(scene, table) if gt_boxes else None
    w = sess.read(0, with_depth=True)
    for _ in range(2):
        core.process(w.color_bgr, w.depth_raw, K, want_mask=False)
    if getattr(core, "mask_cache", None) is not None:
        core.mask_cache.reset()
    pos = {int(i): k for k, i in enumerate(sess.ids)}
    if mode == "bop19":
        tg = json.loads(DATA[dataset]["targets"].read_text())
        todo = sorted({pos[t["im_id"]] for t in tg if t["scene_id"] == int(scene.name)})
    else:
        todo = list(range(len(sess)))
    if exclude_targets and DATA[dataset]["targets"] is not None:
        tg = json.loads(DATA[dataset]["targets"].read_text())
        bad = {pos[t["im_id"]] for t in tg if t["scene_id"] == int(scene.name) and t["im_id"] in pos}
        todo = [k for k in todo if k not in bad]
    if stride > 1 or offset:
        todo = todo[offset::max(stride, 1)]
    if max_frames > 0:
        todo = todo[:max_frames]
    n_total = len(todo); stopped = False
    if resume and (out / "sam6d_frames.jsonl").exists():      # continue a scene that was stopped part-way (still / all / bop19)
        keep = []
        for l in (out / "sam6d_frames.jsonl").read_text().splitlines():
            try:
                keep.append(json.loads(l))
            except ValueError:
                pass
        seen = {r["idx"] for r in keep}
        (out / "sam6d_frames.jsonl").write_text("".join(json.dumps(r, ensure_ascii=False) + "\n" for r in keep))
        est = []
        for l in (out / "sam6d_estimates.jsonl").read_text().splitlines():
            try:
                if json.loads(l)["idx"] in seen:
                    est.append(l + "\n")
            except ValueError:
                pass
        (out / "sam6d_estimates.jsonl").write_text("".join(est))
        todo = [k for k in todo if k not in seen]
        print(f"[{method} {scene.name} {mode}] resume: {len(seen)} frames already done, {len(todo)} left", flush=True)
    f_est = open(out / "sam6d_estimates.jsonl", "a" if resume else "w"); f_frm = open(out / "sam6d_frames.jsonl", "a" if resume else "w")
    n_proc = n_det = 0
    fps = sess.fps; n = len(sess)
    wall0 = time.monotonic()
    j = 0; i = todo[0] if todo else 0
    while j < len(todo):
        if stop_at and time.time() >= stop_at:
            stopped = True
            break
        if mode == "realtime":
            i = max(todo[j], int((time.monotonic() - wall0) * fps))
            if i >= n:
                break
            wait = i / fps - (time.monotonic() - wall0)
            if wait > 0:
                time.sleep(wait)
        else:
            i = todo[j]
        t_start = time.monotonic() - wall0
        fr = sess.read(i, with_depth=True)
        if gtb is not None:
            core.gt_boxes = gtb.get(int(sess.ids[i]), [])
        rows, ms, n_boxes, _ = core.process(fr.color_bgr, fr.depth_raw, K, want_mask=False)
        if hasattr(core, "gate_thr"):
            for r in rows:
                sim, appe = core.gate_thr[r["object"]]
                ism = r.get("ism") or {}
                r["gate_pass"] = bool(ism.get("best_sem", 0) >= sim and ism.get("masked_appe", 0) >= appe)
                r["verified"] = bool(r.get("verified", True))
                r["pem_score"] = r["score"]
                r["score"] = 2.0 * r["gate_pass"] + 1.0 * r["verified"] + 0.5 * float(r["pem_score"])
        t_done = time.monotonic() - wall0
        n_proc += 1; n_det += len(rows)
        arrive = i / fps
        rec = {"t_ns": fr.t_ns, "idx": i, "im_id": int(sess.ids[i]), "arrive_s": round(arrive, 4),
               "start_s": round(t_start, 4), "done_s": round(t_done, 4), "latency_s": round(t_done - arrive, 4),
               "ms": ms, "n_boxes": n_boxes,
               "dets": [{k: r.get(k) for k in ("object", "score", "R", "t_mm", "gate_pass", "verified") if k in r} for r in rows]}
        f_frm.write(json.dumps(rec, ensure_ascii=False) + "\n")
        for r in rows:
            f_est.write(json.dumps({"t_ns": fr.t_ns, "idx": i, "im_id": int(sess.ids[i]), "object": r["object"],
                                    "score": r["score"], "R": r["R"], "t_mm": r["t_mm"],
                                    "latency_s": rec["latency_s"], "ms": ms}, ensure_ascii=False) + "\n")
            if mode == "bop19":
                R = np.asarray(r["R"]).reshape(-1); t = np.asarray(r["t_mm"]).reshape(-1)
                csv_rows.append(f"{int(scene.name)},{int(sess.ids[i])},{oid[r['object']]},{float(r['score']):.6f},"
                                f"{' '.join(f'{v:.6f}' for v in R)},{' '.join(f'{v:.4f}' for v in t)},{(t_done - t_start):.4f}")
        if n_proc % 50 == 0:
            f_est.flush(); f_frm.flush()
            print(f"[{method} {scene.name} {mode}] {j + 1}/{len(todo)} processed {n_proc} dets {n_det} "
                  f"last {ms['total']} ms elapsed {t_done:.0f}s", flush=True)
        j += 1
        if mode == "realtime":
            while j < len(todo) and todo[j] <= i:
                j += 1
    el = time.monotonic() - wall0
    f_est.close(); f_frm.close()
    if stopped:                 # no run.json: the scene stays unfinished and --resume picks it up
        print(f"[{method} {scene.name} {mode}] stopped at the time limit: {n_proc} frames this run, {len(todo) - j} left", flush=True)
        return False
    if n_total != len(todo):    # resumed: the summary counts the whole scene, elapsed / hz only this run
        n_det = sum(1 for _ in open(out / "sam6d_estimates.jsonl")); used = dict(used, resumed=True)
    summ = {"scene": scene.name, "mode": mode, "frames": n_total, "processed": n_total if n_total != len(todo) else n_proc, "detections": n_det,
            "elapsed_s": round(el, 1), "hz": round(n_proc / max(el, 1e-9), 3), "config": used}
    (out / "run.json").write_text(json.dumps(summ, indent=1, ensure_ascii=False))
    print(f"[{method} {scene.name} {mode}] done: {n_proc} frames, {n_det} dets, {el:.0f}s", flush=True)
    del core
    import torch
    torch.cuda.empty_cache()
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", choices=["proposed", "proposed_nogate", "proposed_noverify", "proposed_noverify_fullpts", "proposed_score", "orig", "orig_vits"], required=True)
    ap.add_argument("--dataset", choices=sorted(DATA), required=True)
    ap.add_argument("--mode", choices=["bop19", "all", "still", "realtime"], required=True)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--scenes", nargs="*", default=None)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--gt-boxes", action="store_true", help="ground-truth boxes replace the detector (YOLO-World / FastSAM ISM)")
    ap.add_argument("--stride", type=int, default=1, help="every n-th frame of the scene (still / all modes)")
    ap.add_argument("--offset", type=int, default=0, help="first frame of the stride (validation frames)")
    ap.add_argument("--exclude-targets", action="store_true", help="drop the BOP-19 test images (validation frames)")
    ap.add_argument("--root", type=Path, default=None, help="scenes folder (default: the table's scenes_dir)")
    ap.add_argument("--all-objects", action="store_true", help="search every dataset object, not only the scene's")
    ap.add_argument("--resume", action="store_true", help="continue a scene stopped part-way instead of starting it again")
    ap.add_argument("--stop-at", type=float, default=0.0, help="unix time: stop cleanly before the next frame once it has passed")
    a = ap.parse_args()
    table = json.loads(DATA[a.dataset]["table"].read_text())
    root = a.root or Path(table["scenes_dir"])
    scenes = a.scenes or sorted(p.name for p in root.iterdir() if p.is_dir())
    csv_path = a.out / f"bop_{a.method}_{a.dataset}.csv"
    a.out.mkdir(parents=True, exist_ok=True)
    for s in scenes:
        rows: list[str] = []
        if (a.out / s / "run.json").exists():
            print(f"[{s}] already done, skip", flush=True)
            continue
        if not run_scene(a.method, a.dataset, root / s, a.mode, a.out / s, rows, a.max_frames, a.all_objects, a.gt_boxes, a.stride, a.offset, a.exclude_targets, a.resume, a.stop_at):
            break
        if a.mode == "bop19":
            new = not csv_path.exists()
            with open(csv_path, "a") as f:
                if new:
                    f.write("scene_id,im_id,obj_id,score,R,t,time\n")
                f.write("".join(r + "\n" for r in rows))


if __name__ == "__main__":
    main()
