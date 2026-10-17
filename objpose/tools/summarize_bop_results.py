#!/usr/bin/env python3
"""summarize_bop_results.py — the paper table from a batch of scored runs.

Reads the `eval.json` written by eval_bop_run.py under one or more run directories and prints
per-scene and per-object rows. Besides the pose metrics it reports **object coverage** — how
many of a scene's ground-truth objects got any estimate at all — because on these benchmarks
the failures are detections that never happen, not poses that come out wrong, and an average
over produced estimates hides exactly that.

    python objpose/tools/summarize_bop_results.py objpose/output/ycbv_batch_260919_120359
    python objpose/tools/summarize_bop_results.py <dir> --objects
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def load(paths):
    out = []
    for p in paths:
        p = Path(p)
        for f in sorted(p.rglob("eval.json")) if p.is_dir() else [p]:
            out.append(json.loads(f.read_text()))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--objects", action="store_true", help="also print the per-object table")
    a = ap.parse_args()
    reps = load(a.runs)
    if not reps:
        raise SystemExit("no eval.json found")

    print(f"\n{'scene':10s} {'objs':>6s} {'seen':>5s} {'est':>6s} {'ADD<.1d':>8s} "
          f"{'ADD-S<.1d':>10s} {'inst.rec':>9s} {'memory':>9s}")
    print("-" * 72)
    tot_e = tot_a = tot_s = 0.0
    tot_obj = tot_seen = tot_inst = tot_hit = 0
    for r in sorted(reps, key=lambda r: r["scene"]):
        objs = r["objects"]
        n_obj = len(objs)
        seen = sum(1 for v in objs.values() if v.get("estimates"))
        o = r.get("overall", {})
        n = o.get("scored", 0)
        mem = r.get("memory_median_err_mm")
        tot_e += n
        tot_a += (o.get("add_recall") or 0) * n
        tot_s += (o.get("adds_recall") or 0) * n
        tot_obj += n_obj
        tot_seen += seen
        gi = o.get("gt_instances_processed", 0)
        ir = o.get("instance_recall")
        tot_inst += gi
        tot_hit += round((ir or 0) * gi)
        print(f"{Path(r['scene']).name:10s} {n_obj:6d} {seen:5d} {n:6d} "
              f"{(o.get('add_recall') or 0)*100:7.1f}% {(o.get('adds_recall') or 0)*100:9.1f}% "
              f"{'—' if ir is None else f'{ir*100:8.1f}%'} "
              f"{'—' if mem is None else f'{mem:6.1f} mm'}")
    e = max(tot_e, 1)
    print("-" * 72)
    print(f"{'total':10s} {tot_obj:6d} {tot_seen:5d} {int(tot_e):6d} "
          f"{tot_a/e*100:7.1f}% {tot_s/e*100:9.1f}% {tot_hit/max(tot_inst,1)*100:8.1f}%")
    print(f"\nobject coverage:  {tot_seen}/{tot_obj} = {tot_seen/max(tot_obj,1)*100:.1f}% "
          f"of ground-truth objects received at least one estimate")
    print(f"instance recall:  {tot_hit}/{tot_inst} = {tot_hit/max(tot_inst,1)*100:.1f}% "
          f"of ground-truth instances in processed frames got a pose within 0.1d (ADD-S)")
    print("\nADD/ADD-S above are conditional on an estimate existing; instance recall is not.")

    if a.objects:
        per = {}
        for r in reps:
            for name, v in r["objects"].items():
                d = per.setdefault(name, {"est": 0, "scenes": 0, "seen": 0, "add": 0.0, "adds": 0.0, "n": 0})
                d["scenes"] += 1
                d["est"] += v.get("estimates", 0)
                d["seen"] += 1 if v.get("estimates") else 0
                if v.get("scored"):
                    d["add"] += (v.get("add_recall") or 0) * v["scored"]
                    d["adds"] += (v.get("adds_recall") or 0) * v["scored"]
                    d["n"] += v["scored"]
        print(f"\n{'object':26s} {'scenes':>7s} {'seen':>5s} {'est':>6s} {'ADD<.1d':>8s} {'ADD-S<.1d':>10s}")
        print("-" * 68)
        for name, d in sorted(per.items(), key=lambda kv: -kv[1]["est"]):
            n = max(d["n"], 1)
            print(f"{name:26s} {d['scenes']:7d} {d['seen']:5d} {d['est']:6d} "
                  f"{d['add']/n*100:7.1f}% {d['adds']/n*100:9.1f}%"
                  + ("   (never detected)" if not d["est"] else ""))


if __name__ == "__main__":
    main()
