#!/usr/bin/env python3
"""score_estcmp.py — per-object scores of the estimator comparison (original SAM-6D / proposed / proposed with GT boxes).

Every processed frame, each method keeps ONE answer per object (its best score; the original SAM-6D returns several
candidates per object, the proposed pipeline at most one). Against the ground truth of that frame
(BOP scene_gt / scene_gt_info; on the recorded session the map-based pseudo GT of build_pgt.py):

  inst     ground-truth instances with visib_fract >= 0.1
  found    of those, the ones whose object has an answer with ADD-S < 10 % of the diameter
  answers  answers given
  correct  answers with ADD-S < 10 % of the diameter to that object's ground truth in the frame
  wrong    answers - correct  (object not there, or pose off)
  ms       median time per frame

    python objpose/tools/pgt/score_estcmp.py            -> objpose/output/estcmp_260927/scores.json
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[3]
OUT = REPO / "objpose/output/estcmp_260927"
PGT = REPO / "objpose/output/pgt_260915_inlong_middle"
import os
SUF = os.environ.get("EST_SUFFIX", "")   # "T": proposed / gtbox runs with the validation-tuned settings
YCBV = Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video")
YCBM = Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_M/xtion_bop/test")
RUNS = {
    "real": {"table": "real_objects.json", "scenes": PGT / "eval", "info": PGT / "models/models_info.json",
             "runs": {"orig": PGT / "runs/eval_orig", "proposed": PGT / "runs/eval_proposed", "gtbox": PGT / "runs/eval_gtbox"}},
    "ycbv": {"table": "ycbv_objects.json", "scenes": YCBV / "test", "info": YCBV / "models_eval/models_info.json",
             "runs": {m: OUT / f"ycbv_{m}{SUF if m != 'orig' else ''}" for m in ("orig", "proposed", "gtbox")}},
    "ycbm": {"table": "ycbm_objects.json", "scenes": YCBM, "info": YCBV / "models_eval/models_info.json",
             "runs": {m: OUT / f"ycbm_{m}{SUF if m != 'orig' else ''}" for m in ("orig", "proposed", "gtbox")}},
}
_PTS = {}


def model_pts(table, oid):
    key = (table["models_dir"], oid)
    if key not in _PTS:
        m = trimesh.load(Path(table["models_dir"]) / f"obj_{oid:06d}.ply", process=False)
        v = np.asarray(m.vertices)
        _PTS[key] = v[np.random.default_rng(0).choice(len(v), min(2000, len(v)), replace=False)]
    return _PTS[key]


def adds(P, Re, te, Rg, tg):
    a = P @ Re.T + te; b = P @ Rg.T + tg
    return float(cKDTree(b).query(a)[0].mean())


def score(ds, cfg):
    table = json.loads((REPO / "objpose/rt/bop" / cfg["table"]).read_text())
    name2id = {o["name"]: o["obj_id"] for o in table["objects"]}
    diam = {int(k): v["diameter"] for k, v in json.loads(Path(cfg["info"]).read_text()).items()}
    res = {}
    for m, root in cfg["runs"].items():
        if not root.exists():
            continue
        per = {}
        ms = []
        errs = []          # ADD-S (mm) of the answer for every visible GT instance, inf when unanswered
        n_frames = 0
        for sc in sorted(p for p in root.iterdir() if (p / "sam6d_frames.jsonl").exists()):
            gt = json.loads((cfg["scenes"] / sc.name / "scene_gt.json").read_text())
            gi = json.loads((cfg["scenes"] / sc.name / "scene_gt_info.json").read_text())
            for l in (sc / "sam6d_frames.jsonl").read_text().splitlines():
                f = json.loads(l)
                fid = str(f["im_id"])
                n_frames += 1; ms.append(f["ms"]["total"])
                best = {}
                for d in f["dets"]:
                    if d["object"] not in best or d["score"] > best[d["object"]]["score"]:
                        best[d["object"]] = d
                gts = {}
                for r, i in zip(gt.get(fid, []), gi.get(fid, [])):
                    gts.setdefault(r["obj_id"], []).append((np.asarray(r["cam_R_m2c"]).reshape(3, 3), np.asarray(r["cam_t_m2c"]).reshape(3), i["visib_fract"]))
                ok_obj = set()
                err_obj = {}
                for n, d in best.items():
                    oid = name2id[n]
                    s = per.setdefault(oid, dict(inst=0, found=0, answers=0, correct=0))
                    s["answers"] += 1
                    Re, te = np.asarray(d["R"]).reshape(3, 3), np.asarray(d["t_mm"]).reshape(3)
                    es = [adds(model_pts(table, oid), Re, te, Rg, tg) for Rg, tg, _ in gts.get(oid, [])]
                    if es:
                        err_obj[oid] = min(es)
                    if any(e < 0.1 * diam[oid] for e in es):
                        s["correct"] += 1; ok_obj.add(oid)
                for oid, lst in gts.items():
                    if oid not in diam:
                        continue
                    s = per.setdefault(oid, dict(inst=0, found=0, answers=0, correct=0))
                    for _, _, vf in lst:
                        if vf >= 0.1:
                            s["inst"] += 1
                            s["found"] += oid in ok_obj
                            errs.append(err_obj.get(oid, np.inf))
        tot = {k: sum(v[k] for v in per.values()) for k in ("inst", "found", "answers", "correct")}
        e = np.asarray(errs); th = np.linspace(0, 100, 1001)
        tot["adds_auc"] = float(np.mean([(e < t).mean() for t in th])) if len(e) else None
        tot.update(frames=n_frames, ms_median=float(np.median(ms)) if ms else None,
                   recall=tot["found"] / max(tot["inst"], 1), precision=tot["correct"] / max(tot["answers"], 1),
                   wrong=tot["answers"] - tot["correct"], wrong_per_frame=(tot["answers"] - tot["correct"]) / max(n_frames, 1))
        res[m] = {"total": tot, "objects": {str(k): v for k, v in sorted(per.items())}}
        print(ds, m, {k: (round(v, 3) if isinstance(v, float) else v) for k, v in tot.items()}, flush=True)
    return res


if __name__ == "__main__":
    import sys
    only = sys.argv[1:] or list(RUNS)
    p = OUT / f"scores{SUF}.json"
    allr = json.loads(p.read_text()) if p.exists() else {}
    for ds in only:
        allr[ds] = score(ds, RUNS[ds])
    p.write_text(json.dumps(allr, indent=1))
