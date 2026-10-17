#!/usr/bin/env python3
"""eval_cmp.py — score the runs of run_all.sh.

  bop     BOP-19 AR (VSD / MSSD / MSPD) of every bop_<method>_<dataset>.csv with the official
          bop_toolkit (conda env `bop`), written to <out>/bop_scores.json
  rt      YCB-V real-time comparison, proposed (objpose/output/ycbv_sam6d_realtime_260921, same core)
          vs original SAM-6D (<out>/orig_<dataset>_realtime), scored with compare_modes.score_scene.
          The original emits several candidates per object; each processed frame keeps the
          best-scoring candidate per object (YCB-V holds one instance per object), the same
          number of answers per object as the proposed pipeline returns.

    python3 objpose/tools/baselines/eval_cmp.py bop
    python3 objpose/tools/baselines/eval_cmp.py rt
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
OUT = Path(os.environ.get("CMP_OUT", REPO / "objpose/output/cmp_260924"))
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))


def bop(only=None):
    env = dict(os.environ)
    env["PATH"] = f"{Path.home()}/anaconda3/envs/bop/bin:" + env["PATH"]
    env.update(BOP_PATH=str(HERE / "bop_root"), BOP_RESULTS_PATH=str(HERE / "bop_results"), BOP_EVAL_PATH=str(HERE / "bop_eval"))
    scores = json.loads((OUT / "bop_scores.json").read_text()) if (OUT / "bop_scores.json").exists() else {}
    for csv in sorted(OUT.glob("*/bop_*.csv")):
        if only and csv.parent.name not in only:
            continue
        method, ds = re.match(r"bop_(.+)_(ycbv|lmo)\.csv", csv.name).groups()
        name = f"{method.replace('_', '')}-{csv.parent.name.replace('_', '')}_{ds}-test.csv"
        if name in scores and scores[name].get("ar") is not None:
            continue
        shutil.copy(csv, HERE / "bop_results" / name)
        log = subprocess.run([f"{Path.home()}/anaconda3/envs/bop/bin/python", "scripts/eval_bop19_pose.py",
                              f"--result_filenames={name}", "--renderer_type=vispy", "--num_workers=8"],
                             cwd=HERE / "bop_toolkit", env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True).stdout
        r = {}
        for k in ("vsd", "mssd", "mspd"):
            m = re.findall(rf"bop19_average_recall_{k}: ([0-9.]+)", log)
            r[k] = float(m[-1]) if m else None
        m = re.findall(r"bop19_average_recall: ([0-9.]+)", log); r["ar"] = float(m[-1]) if m else None
        m = re.findall(r"bop19_average_time_per_image: ([0-9.]+)", log); r["time"] = float(m[-1]) if m else None
        scores[name] = r
        print(name, r, flush=True)
        (OUT / "bop_scores.json").write_text(json.dumps(scores, indent=1))
    return scores


def top1(run: Path, dst: Path):
    dst.mkdir(parents=True, exist_ok=True)
    rows = []
    for l in (run / "sam6d_frames.jsonl").read_text().splitlines():
        if not l.strip():
            continue
        f = json.loads(l)
        best = {}
        for d in f["dets"]:
            if d["object"] not in best or d["score"] > best[d["object"]]["score"]:
                best[d["object"]] = d
        f["dets"] = list(best.values())
        rows.append(json.dumps(f))
    (dst / "sam6d_frames.jsonl").write_text("\n".join(rows) + "\n")


def rt():
    DS = os.environ.get("BENCH_TAG", "ycbv")          # run folders are <method>_<DS>_realtime
    import compare_modes as CM
    table = json.loads(CM.TABLE.read_text())
    by_id = {o["obj_id"]: o["name"] for o in table["objects"]}
    info = json.loads((CM.YCBV / "models_eval" / "models_info.json").read_text())
    objs = {by_id[int(k)]: CM.Obj(by_id[int(k)], v["diameter"]) for k, v in info.items() if int(k) in by_id}
    slam_dir = Path(os.environ.get("BENCH_SLAM", REPO / "objpose/output/ycbv_slam_orb_260921"))
    if "CMP_OUT" in os.environ:
        runs = {m: OUT / f"{m}_{DS}_realtime" for m in ("proposed", "orig", "proposednogate", "proposedscore", "proposedscoretier3", "origtop") if (OUT / f"{m}_{DS}_realtime").exists()}
    else:
        runs = {"proposed": REPO / "objpose/output/ycbv_sam6d_realtime_260921", "orig": OUT / f"orig_{DS}_realtime"}
    res = {}
    for m, root in runs.items():
        agg = {"frames": 0, "processed": 0, "scored": 0, "adds_ok": 0, "add_ok": 0, "est": 0, "inst_all": 0,
               "inst_proc": 0, "hit": 0, "screen_hold_hit": 0, "screen_slam_hit": 0, "latency": [], "ms": [], "scenes": {}}
        for sc in sorted(p.name for p in (CM.YCBV / "test").iterdir()):
            run = root / sc
            if not (run / "sam6d_frames.jsonl").exists():
                continue
            if m not in ("proposed", "proposedscoretier3", "origtop"):
                t1 = OUT / f"{m}_{DS}_realtime_top1" / sc
                top1(run, t1); run = t1
            n = len(json.loads((CM.YCBV / "test" / sc / "scene_camera.json").read_text()))
            slam = CM.load_slam(slam_dir / f"{sc}_frames.txt", n)
            r = CM.score_scene(CM.YCBV / "test" / sc, run, objs, by_id, slam_T=slam, realtime=True)
            for k in ("frames", "processed", "scored", "adds_ok", "add_ok", "est", "inst_all", "inst_proc", "hit",
                      "screen_hold_hit", "screen_slam_hit"):
                agg[k] += r[k]
            agg["latency"] += [x for x in r["latency"] if x is not None]; agg["ms"] += r["ms"]
            agg["scenes"][sc] = {k: r[k] for k in ("frames", "processed", "inst_all", "screen_hold_hit", "screen_slam_hit", "hit", "inst_proc")}
        a = agg
        res[m] = {"frames": a["frames"], "processed": a["processed"], "proc_rate": a["processed"] / max(a["frames"], 1),
                  "est": a["est"], "add_cond": a["add_ok"] / max(a["scored"], 1), "adds_cond": a["adds_ok"] / max(a["scored"], 1),
                  "recall_proc": a["hit"] / max(a["inst_proc"], 1),
                  "screen_hold": a["screen_hold_hit"] / max(a["inst_all"], 1), "screen_slam": a["screen_slam_hit"] / max(a["inst_all"], 1),
                  "ms_median": float(np.median(a["ms"])) if a["ms"] else None,
                  "latency_median": float(np.median(a["latency"])) if a["latency"] else None,
                  "scenes": a["scenes"]}
        print(m, {k: v for k, v in res[m].items() if k != "scenes"}, flush=True)
    (OUT / "rt_scores.json").write_text(json.dumps(res, indent=1))
    return res


if __name__ == "__main__":
    if sys.argv[1] == "bop":
        bop(sys.argv[2:] or None)
    else:
        rt()
