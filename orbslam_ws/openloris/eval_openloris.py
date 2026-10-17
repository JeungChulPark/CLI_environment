#!/usr/bin/env python3
"""Score openloris_rgbd runs with the official OpenLORIS-Scene evaluator (openloris-scene-tools/benchmark).

Each run directory (…/<variant>/<sequence>/rep<k>/) holds live_tum.txt and final_tum.txt in the d400_color frame.
For each file the official code transforms the ground truth (base_link) into d400_color with the dataset's
extrinsics, aligns the trajectory to the ground truth with SE(3) (no scale: RGB-D), and computes:
  ate_rmse [m], aoe_rmse [deg]    absolute position / orientation error after alignment
  cr                               correct rate = time with a correct pose / sequence duration (official CR);
                                   computed for "any pose" (no threshold) and for the stricter 0.1 m / 5 deg
plus ate_max [m] from the same per-pose errors, and ate_rmse_sim3 = the official ATE with --scale (Sim(3)
alignment). On office1-1 every variant is ~5 % smaller than the ground truth (the D435i depth or the mocap
scale), which dominates the SE(3) ATE; ate_rmse_sim3 shows the error left once that common factor is removed.

When tracking is lost for good, RGB-D ORB-SLAM3 starts a new map with its own origin, and the official single
alignment then mixes several frames (home/corridor: ATE of metres, AOE ~180 deg). The seg_* columns split the
trajectory at every new map (frames.csv: an OK frame after a frame that was neither OK nor RECENTLY_LOST) and at
every jump of more than SEG_JUMP_M between consecutive poses (a map merge moves the live pose into the older map's
frame mid-run; the robot moves < 3 cm per frame at 30 Hz), score each piece of at least SEG_MIN_S seconds on its own, and pool them:
  seg_n                 maps with at least SEG_MIN_S seconds of poses (seg_all = every map)
  seg_cr                sum of the official CR of those pieces = time covered by a scored pose / sequence time
  seg_ate_se3/_sim3     RMSE over all their poses, each piece aligned separately (SE(3) / Sim(3))
For a run that never loses its map (office) these equal the official numbers. Writes <runs>/results.csv and prints a
per-variant table.

    eval_openloris.py RUNS_DIR [--tools orbslam_ws/output/openloris/tools] [--gt DATASET/gt/per-scene]
"""
import argparse
import csv
import io
import json
import math
import os
import re
import sys
import tempfile
from contextlib import redirect_stdout
from pathlib import Path

import matplotlib
matplotlib.use("Agg")

HERE = Path(__file__).resolve().parent
ap = argparse.ArgumentParser()
ap.add_argument("runs")
ap.add_argument("--tools", default=str(HERE.parent / "output/openloris/tools"))
ap.add_argument("--gt", default=str(Path.home() / "Documents/DefenseMeta/Dataset/openloris/gt/per-scene"))
a = ap.parse_args()
sys.path[:0] = [str(HERE / "tf_shim"), f"{a.tools}/benchmark", f"{a.tools}/dataprocess"]
import evaluate as ol  # noqa: E402  (official evaluator)

SEG_MIN_S = 5.0
SEG_JUMP_M = 0.25
SEQ = re.compile(r"^(office|corridor|home|cafe|market)1-(\d+)$")


def score(tum: Path, scene: str, seq: int, ate_thr: float, aoe_thr: float, auto_scale: bool = False):
    text = f"scene: {scene}\nframe: d400_color\nseq: {seq}\n" + tum.read_text()
    tmp = tum.with_suffix(".openloris.txt")
    tmp.write_text(text)
    info, seqs = ol.parse_input(str(tmp))
    gt_info, gts = ol.parse_input(f"{a.gt}/gt_{scene}_1.txt")
    gts = {seq: gts[seq]}
    if not seqs[seq]["traj"]:
        return None
    with redirect_stdout(io.StringIO()):
        ol.evaluate(seqs, info, gts, gt_info, ate_thr, aoe_thr, 1.0, 60.0, auto_scale, print_results=False)
    s = seqs[seq]
    return {"ate_rmse": s["ate_rmse"], "aoe_rmse": s["aoe_rmse"], "cr": s["cr"],
            "ate_max": max(s["ate"].values()) if s["ate"] else float("nan"), "poses": s["ate_num"]}


def segments(run: Path, tum: Path):
    """Split tum into one text per map, using the tracking states in frames.csv."""
    states = []
    with open(run / "frames.csv") as fh:
        for r in csv.DictReader(fh):
            states.append((float(r["stamp"]), int(r["state"])))
    states.sort()
    seg_of, sid, prev = {}, 0, None
    for t, st in states:
        if st == 2 and prev not in (2, 3):
            sid += 1
        seg_of[round(t, 4)] = sid
        prev = st
    pieces, last_k, last_p = [], None, None
    for line in tum.read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        v = line.split()
        k = seg_of.get(round(float(v[0]), 4))
        if not k:
            continue
        p = [float(x) for x in v[1:4]]
        if k != last_k or math.dist(p, last_p) > SEG_JUMP_M:
            pieces.append([])
        pieces[-1].append(line)
        last_k, last_p = k, p
    return pieces


def seg_score(run: Path, tum: Path, scene: str, seq: int):
    pieces = segments(run, tum)
    out = {"seg_all": len(pieces), "seg_n": 0, "seg_cr": 0.0}
    sq = {"se3": 0.0, "sim3": 0.0}
    n = 0
    with tempfile.TemporaryDirectory() as td:
        for i, lines in enumerate(pieces):
            if float(lines[-1].split()[0]) - float(lines[0].split()[0]) < SEG_MIN_S:
                continue
            f = Path(td) / f"seg{i}.txt"
            f.write_text("\n".join(lines) + "\n")
            r = score(f, scene, seq, math.inf, math.inf)
            r3 = score(f, scene, seq, math.inf, math.inf, auto_scale=True)
            if not r:
                continue
            out["seg_n"] += 1
            out["seg_cr"] += r["cr"]
            sq["se3"] += r["ate_rmse"] ** 2 * r["poses"]
            sq["sim3"] += r3["ate_rmse"] ** 2 * r3["poses"]
            n += r["poses"]
    out["seg_ate_se3"] = math.sqrt(sq["se3"] / n) if n else float("nan")
    out["seg_ate_sim3"] = math.sqrt(sq["sim3"] / n) if n else float("nan")
    return out


rows = []
for summ in sorted(Path(a.runs).glob("*/*/rep*/summary.json")):
    run = summ.parent
    variant, seqname, rep = run.parts[-3], run.parts[-2], run.parts[-1]
    m = SEQ.match(seqname)
    if not m:
        continue
    scene, seq = m.group(1), int(m.group(2))
    js = json.loads(summ.read_text())
    for kind in ("live", "final"):
        f = run / f"{kind}_tum.txt"
        if not f.exists():
            continue
        r_any = score(f, scene, seq, math.inf, math.inf)
        r_strict = score(f, scene, seq, 0.1, 5.0)
        r_sim3 = score(f, scene, seq, math.inf, math.inf, auto_scale=True)
        row = {"variant": variant, "sequence": seqname, "rep": rep, "traj": kind,
               "track_mean_ms": js["track_ms"]["mean"], "track_p99_ms": js["track_ms"]["p99"],
               "dropped": js["dropped"], "ok_ratio": js["ok_ratio_of_all"], "map_changes": js["map_changes"]}
        if r_any:
            row.update({k: r_any[k] for k in ("ate_rmse", "ate_max", "aoe_rmse", "cr", "poses")})
            row["cr_0.1m_5deg"] = r_strict["cr"]
            row["ate_rmse_sim3"] = r_sim3["ate_rmse"]
            row.update(seg_score(run, f, scene, seq))
        rows.append(row)

if not rows:
    sys.exit(f"no runs under {a.runs}")
keys = ["variant", "sequence", "rep", "traj", "ate_rmse", "ate_rmse_sim3", "ate_max", "aoe_rmse", "cr", "cr_0.1m_5deg", "poses",
        "seg_all", "seg_n", "seg_cr", "seg_ate_se3", "seg_ate_sim3", "track_mean_ms", "track_p99_ms", "dropped", "ok_ratio", "map_changes"]
with open(Path(a.runs) / "results.csv", "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore")
    w.writeheader()
    w.writerows(rows)

fmt = lambda v, s="{:.3f}": "-" if v is None or (isinstance(v, float) and math.isnan(v)) else s.format(v)
seqs = sorted({r["sequence"] for r in rows}, key=lambda s: (s.split("1-")[0], int(s.split("1-")[1])))
for kind in ("live", "final"):
    print(f"\n== {kind} trajectory: ATE RMSE [m] / CR(0.1 m, 5 deg), mean over repeats ==")
    print(f"{'variant':<14}" + "".join(f"{s:>16}" for s in seqs))
    for v in sorted({r["variant"] for r in rows}):
        cells = []
        for s in seqs:
            rs = [r for r in rows if r["variant"] == v and r["sequence"] == s and r["traj"] == kind and "ate_rmse" in r]
            if not rs:
                cells.append(f"{'-':>16}")
                continue
            ate = sum(r["ate_rmse"] for r in rs) / len(rs)
            cr = sum(r["cr_0.1m_5deg"] for r in rs) / len(rs)
            cells.append(f"{fmt(ate):>9}/{fmt(cr, '{:.2f}'):>5} ")
        print(f"{v:<14}" + "".join(cells))
print(f"\nwrote {Path(a.runs) / 'results.csv'}")
