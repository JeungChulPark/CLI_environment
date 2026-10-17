#!/usr/bin/env python3
"""score_allframes.py — speed, per-module time and accuracy of the all-frame runs (run_allframes.sh).

Reads <root>/<dataset>_<method>/<scene>/sam6d_frames.jsonl (only finished scenes: run.json present) and writes
<root>/metrics.json. Scenes are scored only when every method of the dataset has finished them, so the
numbers of the two methods always cover the same frames.

Time (ms per frame, mean over frames; `sub` timers of run_profiled.py):
  proposed  detect | seg | dino | cand_other | pem_prep | pem_feat | pem_coarse | verify | pem_fine | pem_other
  orig               seg | dino | cand_other |            pem_feat | pem_coarse |          pem_fine | pem_other
  read = image + depth loading (outside the estimator, not in `total`)

Accuracy, as objpose/tools/pgt/score_estcmp.py: one answer per object and frame (its best score), against
scene_gt of that frame, instances with visib_fract >= 0.1.
  recall      instances whose object has an answer with ADD-S < 10 % of the diameter
  precision   answers with ADD-S < 10 % of the diameter
  recall_add  same with ADD(-S): ADD for objects without symmetry, ADD-S for symmetric ones
  auc_adds / auc_add   area under the accuracy-threshold curve, 0..10 cm (unanswered instance = miss)

    python objpose/tools/baselines/score_allframes.py [objpose/output/allframes_261006]
"""
from __future__ import annotations

import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[3]
YCBV = Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video")
INFO = YCBV / "models_eval/models_info.json"
DATASETS = {"ycbv": "ycbv_objects.json", "ycbm": "ycbm_objects.json"}
METHODS = ("orig", "proposed")
PARTS = {"proposed": ("detect", "seg", "dino", "cand_other", "pem_prep", "pem_feat", "pem_coarse", "verify", "pem_fine", "pem_other"),
         "orig": ("seg", "dino", "cand_other", "pem_feat", "pem_coarse", "pem_fine", "pem_other")}
_PTS = {}


def model_pts(models_dir, oid):
    if oid not in _PTS:
        v = np.asarray(trimesh.load(Path(models_dir) / f"obj_{oid:06d}.ply", process=False).vertices)
        _PTS[oid] = v[np.random.default_rng(0).choice(len(v), min(2000, len(v)), replace=False)]
    return _PTS[oid]


def parts_of(method, ms):
    s = ms.get("sub", {})
    g = lambda k: s.get(k, 0.0)
    known_pem = g("pem_prep") + g("pem_feat") + g("pem_coarse") + g("pem_fine") + g("verify_cand") + g("verify_fine")
    p = {"detect": ms.get("yolo", 0), "seg": g("seg"), "dino": g("dino"), "cand_other": ms["ism"] - g("seg") - g("dino"),
         "pem_prep": g("pem_prep"), "pem_feat": g("pem_feat"), "pem_coarse": g("pem_coarse"),
         "verify": g("verify_cand") + g("verify_fine"), "pem_fine": g("pem_fine"), "pem_other": ms["pem"] - known_pem}
    return [p[k] for k in PARTS[method]]


def score_scene(job):
    ds, method, run, scene = job
    table = json.loads((REPO / "objpose/rt/bop" / DATASETS[ds]).read_text())
    name2id = {o["name"]: o["obj_id"] for o in table["objects"]}
    info = {int(k): v for k, v in json.loads(INFO.read_text()).items()}
    sym = {k for k, v in info.items() if "symmetries_discrete" in v or "symmetries_continuous" in v}
    sdir = Path(table["scenes_dir"]) / scene
    gt = json.loads((sdir / "scene_gt.json").read_text()); gi = json.loads((sdir / "scene_gt_info.json").read_text())
    T, P, NOBJ, READ = [], [], [], []
    cnt = dict(inst=0, found=0, found_add=0, answers=0, correct=0)
    e_s, e_a = [], []
    for l in (Path(run) / scene / "sam6d_frames.jsonl").read_text().splitlines():
        f = json.loads(l); fid = str(f["im_id"])
        T.append(f["ms"]["total"]); P.append(parts_of(method, f["ms"]))
        READ.append((f["done_s"] - f["start_s"]) * 1e3 - f["ms"]["total"])
        best = {}
        for d in f["dets"]:
            if d["object"] not in best or d["score"] > best[d["object"]]["score"]:
                best[d["object"]] = d
        gts = {}
        for r, i in zip(gt.get(fid, []), gi.get(fid, [])):
            gts.setdefault(r["obj_id"], []).append((np.asarray(r["cam_R_m2c"]).reshape(3, 3), np.asarray(r["cam_t_m2c"]).reshape(3), i["visib_fract"]))
        NOBJ.append(sum(v >= 0.1 for lst in gts.values() for *_, v in lst))
        es, ea = {}, {}
        for n, d in best.items():
            oid = name2id[n]; cnt["answers"] += 1
            if oid not in gts:
                continue
            M = model_pts(table["models_dir"], oid)
            a = M @ np.asarray(d["R"]).reshape(3, 3).T + np.asarray(d["t_mm"]).reshape(3)
            s_, a_ = [], []
            for Rg, tg, _ in gts[oid]:
                b = M @ Rg.T + tg
                s_.append(float(cKDTree(b).query(a)[0].mean()))
                a_.append(s_[-1] if oid in sym else float(np.linalg.norm(a - b, axis=1).mean()))
            es[oid], ea[oid] = min(s_), min(a_)
            cnt["correct"] += es[oid] < 0.1 * info[oid]["diameter"]
        for oid, lst in gts.items():
            for *_, vf in lst:
                if vf >= 0.1:
                    cnt["inst"] += 1
                    cnt["found"] += es.get(oid, np.inf) < 0.1 * info[oid]["diameter"]
                    cnt["found_add"] += ea.get(oid, np.inf) < 0.1 * info[oid]["diameter"]
                    e_s.append(es.get(oid, np.inf)); e_a.append(ea.get(oid, np.inf))
    return dict(ds=ds, method=method, scene=scene, T=T, P=P, NOBJ=NOBJ, READ=READ, cnt=cnt, e_s=e_s, e_a=e_a)


def auc(e):
    e = np.asarray(e); th = np.linspace(0, 100, 1001)
    return float(np.mean([(e < t).mean() for t in th])) if len(e) else None


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else REPO / "objpose/output/allframes_261006")
    jobs, cover = [], {}
    for ds in DATASETS:
        done = {m: {p.name for p in (root / f"{ds}_{m}").glob("*") if (p / "run.json").exists()} for m in METHODS}
        ran = [m for m in METHODS if done[m]]          # a method that was not run for this dataset is left out
        common = sorted(set.intersection(*(done[m] for m in ran))) if ran else []
        table = json.loads((REPO / "objpose/rt/bop" / DATASETS[ds]).read_text())
        n_all = sum(1 for p in Path(table["scenes_dir"]).iterdir() if p.is_dir())
        cover[ds] = {"scenes_scored": len(common), "scenes_total": n_all, **{f"done_{m}": len(done[m]) for m in METHODS}}
        jobs += [(ds, m, str(root / f"{ds}_{m}"), s) for m in ran for s in common]
    with ProcessPoolExecutor(8) as ex:
        res = list(ex.map(score_scene, jobs))
    out = {"coverage": cover}
    for ds in DATASETS:
        for m in METHODS:
            rr = [r for r in res if r["ds"] == ds and r["method"] == m]
            if not rr:
                continue
            T = np.concatenate([r["T"] for r in rr]); P = np.concatenate([np.asarray(r["P"]).reshape(-1, len(PARTS[m])) for r in rr])
            N = np.concatenate([r["NOBJ"] for r in rr]); RD = np.concatenate([r["READ"] for r in rr])
            c = {k: int(sum(r["cnt"][k] for r in rr)) for k in rr[0]["cnt"]}
            e_s = sum((r["e_s"] for r in rr), []); e_a = sum((r["e_a"] for r in rr), [])
            by_n = {int(n): {"frames": int((N == n).sum()), "ms_mean": float(T[N == n].mean())} for n in np.unique(N)}
            out[f"{ds}_{m}"] = {
                "frames": int(len(T)),
                "time": {"ms_mean": float(T.mean()), "ms_median": float(np.median(T)), "ms_p95": float(np.percentile(T, 95)),
                         "ms_max": float(T.max()), "fps": float(1e3 / T.mean()), "read_ms_mean": float(RD.mean()),
                         "by_gt_objects": by_n},
                "modules_ms_mean": dict(zip(PARTS[m], P.mean(0).round(1).tolist())),
                "modules_share": dict(zip(PARTS[m], (P.mean(0) / T.mean()).round(4).tolist())),
                "accuracy": {**c, "wrong": c["answers"] - c["correct"], "recall": c["found"] / max(c["inst"], 1),
                             "recall_add": c["found_add"] / max(c["inst"], 1), "precision": c["correct"] / max(c["answers"], 1),
                             "wrong_per_frame": (c["answers"] - c["correct"]) / len(T), "auc_adds": auc(e_s), "auc_add": auc(e_a)}}
            o = out[f"{ds}_{m}"]; a = o["accuracy"]
            print(f"{ds} {m}: frames {o['frames']} | {o['time']['ms_mean']:.0f} ms ({o['time']['fps']:.2f} fps), median {o['time']['ms_median']:.0f}, p95 {o['time']['ms_p95']:.0f}")
            print("   modules", " ".join(f"{k}={v:.0f}({100 * o['modules_share'][k]:.0f}%)" for k, v in o["modules_ms_mean"].items()))
            print(f"   inst {a['inst']} recall {100 * a['recall']:.1f} recall_add {100 * a['recall_add']:.1f} answers {a['answers']} precision {100 * a['precision']:.1f} "
                  f"wrong/frame {a['wrong_per_frame']:.2f} AUC ADD-S {100 * a['auc_adds']:.1f} ADD(-S) {100 * a['auc_add']:.1f}")
    print("coverage", cover)
    (root / "metrics.json").write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
