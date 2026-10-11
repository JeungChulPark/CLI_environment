#!/usr/bin/env python3
"""Metrics for the YCB-V single-image comparison (paper Table 1 protocol).

Per method (prediction JSON from run_orig.py / run_ours.py):
  correct      ADD-S < 10 % of the object diameter (models_eval/models_info.json),
               ADD-S over the models_eval vertices
  Found        visible GT targets with a correct answer / visible GT targets
               (visible = visib_fract > 0; all 4123 BOP19 targets are)
  Answers correct   correct answers / all answers
  Wrong / image     wrong answers / images
  ADD-S AUC    area under the accuracy-threshold curve, 0-10 cm (PoseCNN VOCap), pooled over
               all GT targets (missing answer = infinite error); per-object mean also reported
  BOP AR       official bop_toolkit scripts/eval_bop19_pose.py (VSD, MSSD, MSPD) on BOP CSV
  Time         median per-image time of the timed region
BOP CSV files and bop_toolkit output go to OUT/bop; summary -> results.json.

    python evaluate.py [--skip_bop]
Env: BOP_TOOLKIT (clone of github.com/thodan/bop_toolkit), BOP_PYTHON (python with its deps).
"""
import argparse
import json
import os
import subprocess
import time
from pathlib import Path
from collections import defaultdict

import numpy as np
import trimesh
from scipy.spatial import cKDTree

import data as D
import paths as P

METHODS = [("sam6d", "SAM-6D (original)", "sam6dorig", "pred_sam6d.json"),
           ("ours_text", "Ours (text)", "ourstext", "pred_ours_text.json"),
           ("ours_gtbox", "Ours (GT box)", "oursgtbox", "pred_ours_gtbox.json"),
           ("ours_gtbox_locked", "Ours (GT box, label-locked; supplementary, not the protocol)",
            "oursgtboxlocked", "pred_ours_gtbox_locked.json")]
BOP_TOOLKIT = P._p("BOP_TOOLKIT", P.WORK / "bop_toolkit")
BOP_PYTHON = P._p("BOP_PYTHON", P.WORK / "bop_venv/bin/python")


def vocap(errs, max_d=0.1):
    d = np.sort(np.asarray(errs, float))
    n = len(d)
    d[d > max_d] = np.inf
    acc = np.arange(1, n + 1) / n
    ok = np.isfinite(d)
    rec, prec = d[ok], acc[ok]
    if len(rec) == 0:
        return 0.0
    mrec = np.concatenate([[0.0], rec, [max_d]])
    mpre = np.concatenate([[0.0], prec, [prec[-1]]])
    for i in range(1, len(mpre)):
        mpre[i] = max(mpre[i], mpre[i - 1])
    i = np.where(mrec[1:] != mrec[:-1])[0] + 1
    return float(np.sum((mrec[i] - mrec[i - 1]) * mpre[i]) / max_d * 100.0)


def adds_mm(pts, Rg, tg, Rp, tp):
    g = pts @ Rg.T + tg
    p = pts @ np.asarray(Rp).T + np.asarray(tp)
    d, _ = cKDTree(p).query(g, k=1)
    return float(d.mean())


def evaluate(pred, models, info):
    per_img = {(x["scene_id"], x["im_id"]): x for x in pred["images"]}
    items = [it for it in D.load_targets() if (it[0], it[1]) in per_img]   # all 900 in a full run
    n_vis = n_found = n_ans = n_cor = 0
    errs, per_obj = [], defaultdict(lambda: {"targets": 0, "found": 0, "answers": 0, "correct": 0, "errs": []})
    for sid, iid, oids in items:
        x = per_img[(sid, iid)]
        best = {}
        for p in x["preds"]:
            if p["obj_id"] not in best or p["score"] > best[p["obj_id"]]["score"]:
                best[p["obj_id"]] = p
        gts = D.gt_instances(sid, iid)
        for oid in oids:
            insts = [g for g in gts if g["obj_id"] == oid]
            diam = info[str(oid)]["diameter"]
            vis = max(g["visib_fract"] for g in insts) > 0
            st = per_obj[oid]
            e = np.inf
            if oid in best:
                e = min(adds_mm(models[oid], g["R"], g["t_mm"], best[oid]["R"], best[oid]["t_mm"]) for g in insts)
                ok = e < 0.1 * diam
                n_ans += 1; st["answers"] += 1
                n_cor += ok; st["correct"] += ok
                if ok and vis:
                    n_found += 1; st["found"] += 1
            if vis:
                n_vis += 1; st["targets"] += 1
            errs.append(e / 1000.0); st["errs"].append(e / 1000.0)
        for oid in best:
            assert oid in oids, "answer for a non-target object"
    auc_obj = {oid: vocap(s["errs"]) for oid, s in per_obj.items()}
    return {
        "images": len(items), "visible_gt_targets": n_vis, "answers": n_ans, "correct_answers": int(n_cor),
        "found_pct": round(100.0 * n_found / n_vis, 1),
        "answers_correct_pct": round(100.0 * n_cor / max(n_ans, 1), 1),
        "wrong_per_image": round((n_ans - n_cor) / len(items), 3),
        "adds_auc": round(vocap(errs), 1),
        "adds_auc_mean_over_objects": round(float(np.mean(list(auc_obj.values()))), 1),
        "time_ms": pred["time_ms"],
        "per_object": {P.YCB_NAMES[oid - 1]: {"targets": s["targets"], "found": s["found"],
                                              "found_pct": round(100.0 * s["found"] / max(s["targets"], 1), 1),
                                              "answers": s["answers"], "correct": int(s["correct"]),
                                              "adds_auc": round(auc_obj[oid], 1)}
                       for oid, s in sorted(per_obj.items())},
    }


def write_csv(pred, path):
    with open(path, "w") as f:
        f.write("scene_id,im_id,obj_id,score,R,t,time\n")
        for x in pred["images"]:
            for p in x["preds"]:
                R = " ".join(f"{v:.8f}" for v in np.asarray(p["R"]).reshape(-1))
                t = " ".join(f"{v:.5f}" for v in np.asarray(p["t_mm"]).reshape(-1))
                f.write(f"{x['scene_id']},{x['im_id']},{p['obj_id']},{p['score']:.6f},{R},{t},"
                        f"{x['time_ms'] / 1000.0:.6f}\n")


def bop_ar(csv_names, bop_dir):
    root = P.WORK / "bop_root"
    root.mkdir(parents=True, exist_ok=True)
    if not (root / "ycbv").exists():
        os.symlink(P.YCBV, root / "ycbv")
    env = dict(os.environ, BOP_PATH=str(root), BOP_RESULTS_PATH=str(bop_dir),
               BOP_EVAL_PATH=str(bop_dir / "eval"), PYTHONPATH=str(BOP_TOOLKIT),
               PATH=str(BOP_PYTHON.parent) + os.pathsep + os.environ.get("PATH", ""),   # sub-scripts call "python"
               LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libstdc++.so.6"
               if os.path.exists("/usr/lib/x86_64-linux-gnu/libstdc++.so.6") else "")
    cmd = [str(BOP_PYTHON), str(BOP_TOOLKIT / "scripts/eval_bop19_pose.py"),
           "--renderer_type=vispy", "--result_filenames=" + ",".join(csv_names),
           f"--num_workers={os.cpu_count() or 4}"]
    print(" ".join(cmd), flush=True)
    log = open(bop_dir / "eval_log.txt", "a")
    subprocess.run(cmd, check=True, env=env, cwd=str(BOP_TOOLKIT), stdout=log, stderr=subprocess.STDOUT)
    out = {}
    for n in csv_names:
        s = json.load(open(bop_dir / "eval" / n[:-4] / "scores_bop19.json"))
        out[n] = {k: round(100.0 * v, 1) for k, v in s.items() if k.startswith("bop19_average_recall")}
        out[n]["raw"] = s
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip_bop", action="store_true")
    ap.add_argument("--methods", default="sam6d,ours_text,ours_gtbox")
    ap.add_argument("--results", default=str(P.HERE / "results.json"))
    a = ap.parse_args()
    want = a.methods.split(",")
    # detector study runs: run_ours.py --tag X writes pred_ours_text_X.json; evaluated as key ours_text_X
    known = {m[0] for m in METHODS}
    extra = [(k, k, k.replace("_", "").replace("-", ""), f"pred_{k}.json") for k in want
             if k not in known and k.startswith(("ours_", "ladder_"))]
    info = json.load(open(P.YCBV / "models_eval" / "models_info.json"))
    models = {oid: np.asarray(trimesh.load(str(P.YCBV / "models_eval" / f"obj_{oid:06d}.ply"),
                                           process=False).vertices, float) for oid in range(1, 22)}
    bop_dir = P.OUT / "bop"
    bop_dir.mkdir(parents=True, exist_ok=True)
    res_path = Path(a.results)
    res = json.load(open(res_path)) if res_path.exists() else {}
    res.update({"label": P.LABEL, "protocol": __doc__.strip().split("\n\n")[1], "evaluated": time.strftime("%Y-%m-%d %H:%M:%S")})
    res.setdefault("methods", {})
    csvs = {}
    for key, title, short, fn in METHODS + extra:
        if key not in want or not (P.OUT / fn).exists():
            continue
        pred = json.load(open(P.OUT / fn))
        m = evaluate(pred, models, info)
        m.update({"title": title, "gpu": pred["gpu"], "run_label": pred["label"],
                  "prediction_file": str(P.OUT / fn), "stage_ms": pred.get("stage_ms")})
        res["methods"][key] = {**res["methods"].get(key, {}), **m}
        csvs[key] = f"{short}_ycbv-test.csv"
        write_csv(pred, bop_dir / csvs[key])
        print(key, {k: v for k, v in m.items() if k not in ("per_object", "stage_ms")}, flush=True)
    if not a.skip_bop and csvs:
        ar = bop_ar(list(csvs.values()), bop_dir)
        for key, n in csvs.items():
            res["methods"][key]["bop_ar"] = ar[n]
            print(key, "BOP AR", {k: v for k, v in ar[n].items() if k != "raw"})
    json.dump(res, open(res_path, "w"), indent=1, default=float)
    print("->", res_path)


if __name__ == "__main__":
    main()
