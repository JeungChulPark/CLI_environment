#!/usr/bin/env python3
"""score_hdl_ablation.py — score the run_hdl_ablation.sh runs against the LiDAR pseudo-GT, velodyne frame.

Per run three trajectories:
  live   per-scan pose as streamed (map->odom . odom at that moment)
  final  per-scan pose re-anchored to its keyframe's last graph estimate (hub_record.py)
  kf     the optimised graph keyframes alone (hdl_stream --dump-dir keyframes_tum.txt)
Also the earlier run_bench.sh run (out/<s>/hdl/run1, floor on) as a repeat of the floor-on setting.
Scorer: eval_traj.evaluate (SE(3) ATE, rotation after mean offset, RPE 1 s / 10 s, drift tilt).

    python3 score_hdl_ablation.py [--out-csv hdl_ablation.csv]
"""
from __future__ import annotations
import argparse, csv, sys, types
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "objpose/rt/260901/bigeight"))
import eval_traj  # noqa: E402
from trajlib import load_tum, Traj, umeyama_se3, rotvec  # noqa: E402

SESSIONS = ("260901_cbnu_bigeightcircle", "260901_cbnu_eightcircle", "260901_cbnu_digut", "260901_cbnu_longcircle")
FILES = {"live": "live_tum.txt", "final": "final_tum.txt"}


def g(d, *k):
    for x in k:
        if not isinstance(d, dict) or x not in d:
            return float("nan")
        d = d[x]
    return d


def at_keyframes(kf_file, other_files, tg, Tg):
    """ATE of the graph keyframes and of the per-scan trajectories sampled at the SAME keyframe stamps.
    eval_traj samples the estimate at the reference rate (0.2 s gap limit), which the 0.5 m keyframes do not meet,
    so this samples the reference at the keyframe stamps instead: SE(3) Umeyama, position RMSE / max, rotation RMSE
    after removing the mean offset"""
    tk, _ = load_tum(kf_file)
    Tr, okr = Traj(tg, Tg, 0.2).at(tk)
    out = {}
    for name, f in [("kf", kf_file)] + other_files:
        te, Te = load_tum(f)
        # the keyframes are taken as they are (their own gaps exceed the interpolation limit)
        Tq, ok = (Te, np.ones(len(te), bool)) if name == "kf" else Traj(te, Te, 0.2).at(tk)
        ok &= okr & np.isfinite(Tq[:, :3, 3]).all(1)
        R, t = umeyama_se3(Tq[ok, :3, 3], Tr[ok, :3, 3])
        pe = np.linalg.norm(Tq[ok, :3, 3] @ R.T + t - Tr[ok, :3, 3], axis=1)
        dR = np.einsum("nji,njk->nik", Tr[ok, :3, :3], R @ Tq[ok, :3, :3])
        rv = rotvec(dR); rv -= rv.mean(0)
        re = np.degrees(np.linalg.norm(rv, axis=1))
        out[name] = {"n": int(ok.sum()), "ate_cm": float(np.sqrt((pe ** 2).mean()) * 100), "ate_max_cm": float(pe.max() * 100),
                     "rot_deg": float(np.sqrt((re ** 2).mean()))}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-csv", default=str(HERE / "hdl_ablation.csv"))
    a = ap.parse_args()
    opts = types.SimpleNamespace(anchor_s=15.0, rpe_s=[1.0, 10.0])
    rows = []
    for s in SESSIONS:
        tg, Tg = load_tum(REPO / f"objpose/rt/gt/out/{s}/pgo_tum.txt")
        runs = [("floor_on_prev", HERE / f"out/{s}/hdl/run1")]
        for fl in ("on", "off"):
            runs += [(f"floor_{fl}", r) for r in sorted((HERE / f"out/{s}/hdl_floor_{fl}").glob("run*"))]
        for name, r in runs:
            for kind, fn in FILES.items():
                f = r / fn
                if not f.exists() or f.stat().st_size == 0:
                    continue
                try:
                    res, _ = eval_traj.evaluate(f"{name}_{kind}", str(f), tg, Tg, np.array([0.0, 0.0, 1.0]), opts)
                except Exception as e:  # noqa: BLE001
                    rows.append({"session": s, "config": name, "run": r.name, "traj": kind, "error": str(e)[:80]})
                    continue
                rows.append({"session": s, "config": name, "run": r.name, "traj": kind, "poses": res["poses"],
                             "ate_cm": g(res, "ate", "pos_cm", "rmse"), "ate_max_cm": g(res, "ate", "pos_cm", "max"),
                             "rot_deg": g(res, "ate", "rot_deg", "rmse"),
                             "tilt_deg": g(res, "drift", "tilt_deg", "rmse"),
                             "rpe1_cm": g(res, "rpe_1s", "trans_cm", "rmse"), "rpe1_deg": g(res, "rpe_1s", "rot_deg", "rmse"),
                             "rpe10_cm": g(res, "rpe_10s", "trans_cm", "rmse"), "coverage": res["coverage"]})
    print("\nat the graph keyframe stamps (kf = optimised keyframes; live / final sampled at the same stamps)")
    print(f"{'session':28s} {'config':10s} {'n':>4s}  {'kf ATE/max/rot':>20s}  {'final ATE/max/rot':>20s}  {'live ATE/max/rot':>20s}")
    for s in SESSIONS:
        tg, Tg = load_tum(REPO / f"objpose/rt/gt/out/{s}/pgo_tum.txt")
        for fl in ("on", "off"):
            for r in sorted((HERE / f"out/{s}/hdl_floor_{fl}").glob("run*")):
                if not (r / "kf_tum.txt").exists():
                    continue
                k = at_keyframes(r / "kf_tum.txt", [("final", r / "final_tum.txt"), ("live", r / "live_tum.txt")], tg, Tg)
                for kind, v in k.items():
                    rows.append({"session": s, "config": f"floor_{fl}", "run": r.name, "traj": f"{kind}@kf", "poses": v["n"],
                                 "ate_cm": v["ate_cm"], "ate_max_cm": v["ate_max_cm"], "rot_deg": v["rot_deg"]})
                cell = lambda v: f"{v['ate_cm']:6.2f} {v['ate_max_cm']:6.2f} {v['rot_deg']:5.2f}"
                print(f"{s:28s} floor_{fl:4s} {k['kf']['n']:4d}  {cell(k['kf']):>20s}  {cell(k['final']):>20s}  {cell(k['live']):>20s}")
    keys = ["session", "config", "run", "traj", "poses", "ate_cm", "ate_max_cm", "rot_deg", "tilt_deg", "rpe1_cm",
            "rpe1_deg", "rpe10_cm", "coverage", "error"]
    with open(a.out_csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore"); w.writeheader(); w.writerows(rows)
    print(f"{'session':28s} {'config':14s} {'traj':5s} {'n':>6s} {'ATE':>6s} {'max':>6s} {'rot':>5s} {'tilt':>5s} "
          f"{'RPE1':>6s} {'RPE1°':>5s} {'RPE10':>6s}")
    for r in rows:
        if "error" in r:
            print(f"{r['session']:28s} {r['config']:14s} {r['traj']:5s} ERROR {r['error']}"); continue
        print(f"{r['session']:28s} {r['config']:14s} {r['traj']:5s} {r['poses']:6d} {r['ate_cm']:6.2f} {r['ate_max_cm']:6.2f} "
              f"{r['rot_deg']:5.2f} {r['tilt_deg']:5.2f} {r['rpe1_cm']:6.2f} {r['rpe1_deg']:5.2f} {r['rpe10_cm']:6.2f}")
    print(f"wrote {a.out_csv}")


if __name__ == "__main__":
    main()
