#!/usr/bin/env python3
"""analyze_repeats.py — N real-time repetitions of the same data: baseline hub vs hub with the (fixed) object anchor.

Per run: processed SAM-6D frames, result age, anchor cost (from the run's own logs), and — replaying the run's own
logs with eval_real (same anchor configuration) — SLAM-only vs anchored current-position error against the run's
final trajectory. Every run has its own SLAM stream timing and its own SAM-6D results, so the spread over runs is the
run-to-run variation of the real-time system.
    python analyze_repeats.py            (output: objpose/output/obj_anchor_260924/repeats.json)
"""
import argparse, glob, json, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
OUT = REPO / "objpose/output/obj_anchor_260924"
PY = "/home/jucpark/anaconda3/envs/sam6d/bin/python"
D = "/home/jucpark/DeepLearning/Datasets/data_slam_converted"
DS = {"cbnu": dict(session=f"{D}/260901_cbnu_eightcircle/SAM", x="objpose/rt/260901_cbnu/X_slam_sam_260901_cbnu.json", tau="-0.212"),
      "inlong": dict(session=f"{D}/260915_six_routes/260915_inlong_middle/SAM", x="objpose/rt/260915/X_slam_sam_260915.json", tau="0.0")}
FIX = ["--set", "use_rotation=false", "--anchor-map", "slam", "--kf-anchors", "--set", "min_obs=5", "--reset-on-loop",
       "--set", "deadband_m=0.03", "--set", "sig_obs_t0=0.03", "--set", "max_rot_sigma=2.0"]


def live_stats(d):
    s = json.loads((d / "summary.json").read_text())
    out = {"frames": s.get("sam6d_frames"), "fed": s.get("sam6d_fed"), "age_median_s": s.get("sam6d_age_median_s"),
           "slam_poses": s.get("slam_poses"), "loops": s.get("slam_loop_events")}
    con = (d / "hub_console.log").read_text()
    out["display_dropped"] = int(con.rsplit("display dropped", 1)[1].split()[0]) if "display dropped" in con else None
    log = d / "anchor_log.jsonl"
    if log.exists():
        L = [json.loads(l) for l in log.read_text().splitlines() if l.strip()]
        a = np.array([x["anchor_ms"] for x in L if "anchor_ms" in x])
        out.update(anchor_ms_median=float(np.median(a)), anchor_ms_p95=float(np.percentile(a, 95)), anchor_ms_max=float(a.max()),
                   resets=sum(1 for x in L if "reset" in x))
    return out


def evaluate(ds, d, tag):
    c = DS[ds]
    cmd = [PY, "objpose/tools/obj_anchor/eval_real.py", str(d), "--session", c["session"], "--x", c["x"], "--tau-s", c["tau"],
           "--tag", tag] + FIX
    subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, timeout=3600)
    return json.loads((OUT / f"real_{tag}.json").read_text())


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--jobs", type=int, default=8); a = ap.parse_args()
    runs = []
    for ds in DS:
        for kind in ("b", "f"):
            for d in sorted(glob.glob(str(REPO / f"objpose/output/anchor_live_{ds}_rep[0-9]{kind}_*"))):
                rep = Path(d).name.split("_rep")[1][0]
                runs.append((ds, kind, int(rep), Path(d)))
    with ThreadPoolExecutor(a.jobs) as ex:
        evs = list(ex.map(lambda r: evaluate(r[0], r[3], f"rep_{r[0]}_{r[1]}{r[2]}"), runs))
    rows = []
    for (ds, kind, rep, d), ev in zip(runs, evs):
        row = {"dataset": ds, "hub": "baseline" if kind == "b" else "anchored", "rep": rep, "dir": d.name, **live_stats(d),
               "slam_rmse": ev["raw"]["rmse_mm"], "slam_median": ev["raw"]["median_mm"], "slam_p90": ev["raw"]["p90_mm"],
               "anc_rmse": ev["anchored"]["rmse_mm"], "anc_median": ev["anchored"]["median_mm"], "anc_p90": ev["anchored"]["p90_mm"],
               "pred_hub_5cm": ev["pred_hub"]["within_50mm"], "pred_anc_5cm": ev["pred_anchored"]["within_50mm"]}
        rows.append(row)
    summ = {}
    for ds in DS:
        for hub in ("baseline", "anchored"):
            R = [r for r in rows if r["dataset"] == ds and r["hub"] == hub]
            if not R:
                continue
            f = lambda k: (float(np.mean([r[k] for r in R if r.get(k) is not None])), float(np.std([r[k] for r in R if r.get(k) is not None])))
            summ[f"{ds}/{hub}"] = {k: f(k) for k in ("frames", "age_median_s", "slam_rmse", "anc_rmse", "slam_median", "anc_median",
                                                      "slam_p90", "anc_p90", "pred_hub_5cm", "pred_anc_5cm")
                                   + (("anchor_ms_p95", "anchor_ms_max", "resets") if hub == "anchored" else ())}
            summ[f"{ds}/{hub}"]["n"] = len(R)
            summ[f"{ds}/{hub}"]["display_dropped_total"] = sum(r["display_dropped"] or 0 for r in R)
    (OUT / "repeats.json").write_text(json.dumps({"runs": rows, "summary": summ}, indent=1))
    for r in rows:
        print(f"{r['dataset']:6s} {r['hub']:8s} #{r['rep']} frames {r['frames']} age {r['age_median_s']} | SLAM {r['slam_rmse']:.1f} "
              f"anchored {r['anc_rmse']:.1f} mm | pred {r['pred_hub_5cm'] * 100:.0f}->{r['pred_anc_5cm'] * 100:.0f}%"
              + (f" | anchor p95 {r['anchor_ms_p95']:.1f} ms resets {r['resets']}" if "anchor_ms_p95" in r else ""))
    print(json.dumps(summ, indent=1))


if __name__ == "__main__":
    main()
