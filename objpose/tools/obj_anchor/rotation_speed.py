#!/usr/bin/env python3
"""rotation_speed.py — does fast camera rotation break the object-anchored correction?

For one recorded live run (replayed with eval_real.run), every live pose sample gets the camera angular speed
(from consecutive SLAM poses, deg/s, 0.2 s span). Binned by that speed:
  * SLAM live error and anchored error vs the final trajectory
  * SAM-6D object observation error: distance of each estimate (placed with the final trajectory) from the object's
    median position — does the pose estimator itself degrade when the camera turns fast?
    python rotation_speed.py <run> --x <X json> --session <SAM session> [--tau-s] [--set k=v ...] [eval_real flags]
"""
from __future__ import annotations

import argparse, json, sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "pc"))
import eval_real as ER                                            # noqa: E402
from obj_anchor import AnchorConfig, rotvec                        # noqa: E402

BINS = [0, 5, 10, 20, 30, 45, 1e9]


def ang_speed(traj):
    ts = np.array(sorted(traj))
    out = {}
    for i, t in enumerate(ts):
        j = int(np.searchsorted(ts, t - 200_000_000))
        if j >= i:
            continue
        dt = (t - ts[j]) / 1e9
        out[t] = np.degrees(np.linalg.norm(rotvec(traj[ts[j]][:3, :3].T @ traj[t][:3, :3]))) / dt
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run"); ap.add_argument("--x", required=True); ap.add_argument("--session", required=True)
    ap.add_argument("--tau-s", type=float, default=0.0); ap.add_argument("--tag", required=True)
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--anchor-map", default="slam"); ap.add_argument("--kf-anchors", action="store_true")
    ap.add_argument("--reset-on-loop", action="store_true")
    a = ap.parse_args()
    cfg = AnchorConfig()
    for kv in a.set:
        k, v = kv.split("=")
        cur = getattr(cfg, k)
        setattr(cfg, k, tuple(float(x) for x in v.split(",")) if isinstance(cur, tuple) else (v.lower() in ("1", "true")) if isinstance(cur, bool) else type(cur)(v))
    ns = argparse.Namespace(run=a.run, x=a.x, session=a.session, tau_s=a.tau_s, no_depth_check=False, k_npz=None,
                            range_scale=1.0, landmarks="memory", anchor_map=a.anchor_map, kf_anchors=a.kf_anchors,
                            reset_on_loop=a.reset_on_loop)
    r = ER.run(ns, cfg)
    X = np.asarray(json.loads(Path(a.x).read_text())["T_slam_sam"], float).reshape(4, 4)
    w = ang_speed(r["live_raw"])
    ts = sorted(set(w) & set(r["ref"]) & set(r["live_anc"]))
    e_raw = np.array([np.linalg.norm((r["live_raw"][t] @ X)[:3, 3] - (r["ref"][t] @ X)[:3, 3]) for t in ts]) * 1000
    e_anc = np.array([np.linalg.norm((r["live_anc"][t] @ X)[:3, 3] - (r["ref"][t] @ X)[:3, 3]) for t in ts]) * 1000
    ws = np.array([w[t] for t in ts])
    # object observation error vs angular speed (final trajectory, all estimates)
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from fit_extrinsic import final_poses, jl
    fin = final_poses(Path(a.run))
    wt = np.array(sorted(w)); wv = np.array([w[t] for t in wt])
    by = {}
    for e in jl(Path(a.run) / "sam6d_estimates.jsonl"):
        s = int(e["t_ns"]); T, _ = fin.at(s)
        if T is None:
            continue
        k = int(np.clip(np.searchsorted(wt, s), 0, len(wt) - 1))
        p = (T @ X)[:3, :3] @ (np.asarray(e["t_mm"]) / 1000.0) + (T @ X)[:3, 3]
        by.setdefault(e["object"], []).append((p, wv[k]))
    oe, ow = [], []
    for v in by.values():
        if len(v) < 5:
            continue
        P = np.array([x[0] for x in v]); med = np.median(P, 0)
        oe += list(np.linalg.norm(P - med, axis=1) * 1000); ow += [x[1] for x in v]
    oe, ow = np.array(oe), np.array(ow)
    rows = []
    for lo, hi in zip(BINS[:-1], BINS[1:]):
        m = (ws >= lo) & (ws < hi); mo = (ow >= lo) & (ow < hi)
        rows.append({"bin": f"{lo:g}-{hi:g}" if hi < 1e8 else f">{lo:g}", "share": float(m.mean()),
                     "raw_med": float(np.median(e_raw[m])) if m.any() else None, "anc_med": float(np.median(e_anc[m])) if m.any() else None,
                     "raw_rmse": float(np.sqrt((e_raw[m] ** 2).mean())) if m.any() else None,
                     "anc_rmse": float(np.sqrt((e_anc[m] ** 2).mean())) if m.any() else None,
                     "obs_n": int(mo.sum()), "obs_med": float(np.median(oe[mo])) if mo.any() else None})
    print(f"{a.tag}: angular speed [deg/s] | share | SLAM err med / RMSE | anchored err med / RMSE | SAM-6D obs n / err med")
    for q in rows:
        f = lambda v: "   -  " if v is None else f"{v:6.1f}"
        print(f"  {q['bin']:>7s} {q['share'] * 100:5.1f}% | {f(q['raw_med'])} {f(q['raw_rmse'])} | {f(q['anc_med'])} {f(q['anc_rmse'])} | {q['obs_n']:4d} {f(q['obs_med'])}")
    c1 = np.corrcoef(ws, e_anc - e_raw)[0, 1]
    print(f"  corr(angular speed, anchored - SLAM error) = {c1:+.2f};  corr(angular speed, obs error) = {np.corrcoef(ow, oe)[0, 1]:+.2f}")
    out = Path(ER.OUT) / f"rotspeed_{a.tag}.json"
    out.write_text(json.dumps({"rows": rows, "corr_speed_extra_err": float(c1), "corr_speed_obs_err": float(np.corrcoef(ow, oe)[0, 1]),
                               "speed_median": float(np.median(ws)), "speed_p90": float(np.percentile(ws, 90))}, indent=1))


if __name__ == "__main__":
    main()
