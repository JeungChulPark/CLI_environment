#!/usr/bin/env python3
"""tune_eval.py — score offline SLAM trajectories for the object-pose system.

For each candidate trajectory pair (<name>.txt per-frame live poses, <name>.txt.optimized.txt
final poses, both TUM on the frame clock):

  * local accuracy: difference of 1 s relative motions against a reference trajectory
    (ORB-SLAM3 f2000 optimized) — independent of global alignment;
  * global agreement: RMSE after a rigid alignment to the reference;
  * object consistency: the same SAM-6D detections (a live run's sam6d_frames.jsonl) placed in
    the candidate's map with the calibrated rig X; static objects should not move, so the
    spread of each object's positions around its median is the score that matters here.

    python objpose/pc/tune_eval.py <dir with c0.txt c1.txt ...> [--ref ...] [--detections ...]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "integration"))
from fusion import PoseBuffer  # noqa: E402
from hub import load_extrinsic  # noqa: E402
from rig_offset import read_tum  # noqa: E402


def rotdeg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def as_dict(ts, T):
    return {int(round(t * 1e6)): M for t, M in zip(ts, T)}   # microsecond keys survive float I/O


def rel_motion(P, Q, win_s=1.0):
    keys = sorted(set(P) & set(Q))
    k = np.array(keys)
    dr, dt = [], []
    for i in range(0, len(keys), 5):
        j = int(np.searchsorted(k, keys[i] + int(win_s * 1e6)))
        if j >= len(keys) or keys[j] - keys[i] > int((win_s + 0.1) * 1e6):
            continue
        a, b = keys[i], keys[j]
        RA = np.linalg.inv(P[a]) @ P[b]
        RB = np.linalg.inv(Q[a]) @ Q[b]
        D = np.linalg.inv(RA) @ RB
        dr.append(rotdeg(D[:3, :3]))
        dt.append(np.linalg.norm(RA[:3, 3] - RB[:3, 3]) * 100)
    dr, dt = np.array(dr), np.array(dt)
    return {"rot_p50_deg": round(float(np.median(dr)), 3), "rot_p95_deg": round(float(np.percentile(dr, 95)), 3),
            "trans_p50_cm": round(float(np.median(dt)), 2), "trans_p95_cm": round(float(np.percentile(dt, 95)), 2)}


def ate(P, Q):
    keys = sorted(set(P) & set(Q))
    A = np.array([P[k][:3, 3] for k in keys])
    B = np.array([Q[k][:3, 3] for k in keys])
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    e = np.linalg.norm((B - cb) @ R.T + ca - A, axis=1) * 100
    return {"rmse_cm": round(float(np.sqrt(np.mean(e ** 2))), 2), "max_cm": round(float(e.max()), 2)}


def object_spread(ts, T, X, frames):
    b = PoseBuffer(0.25)
    for t, M in zip(ts, T):
        b.add(int(round(t * 1e9)), "OK", M)
    per = {}
    for f in frames:
        Tw, _ = b.at(int(f["t_ns"]))
        if Tw is None:
            continue
        for d in f["dets"]:
            M = np.eye(4)
            M[:3, :3] = np.asarray(d["R"]).reshape(3, 3)
            M[:3, 3] = np.asarray(d["t_mm"]) / 1000
            per.setdefault(d["object"], []).append((Tw @ X @ M)[:3, 3])
    # robust per object: ignore gross identity errors (> 30 cm from the object's median)
    s = []
    for P in per.values():
        P = np.array(P)
        e = np.linalg.norm(P - np.median(P, 0), axis=1)
        s += list(e[e < 0.30])
    s = np.array(s) * 100
    return {"n": int(len(s)), "median_cm": round(float(np.median(s)), 2), "p90_cm": round(float(np.percentile(s, 90)), 2)}


def main():
    rt = REPO / "objpose" / "rt"
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--names", default="")
    ap.add_argument("--ref", default=str(rt / "slam_traj_ftime.txt.optimized.txt"))
    ap.add_argument("--ref-live", default=str(rt / "slam_traj_ftime.txt"))
    ap.add_argument("--detections", default=str(REPO / "objpose" / "output" / "live_260913_v7" / "sam6d_frames.jsonl"))
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    X, _ = load_extrinsic(rt / "X_slam_sam.json")
    frames = [json.loads(l) for l in open(a.detections)]
    d = Path(a.dir)
    names = a.names.split(",") if a.names else sorted(p.stem for p in d.glob("*.txt") if p.name.count(".") == 1)
    ref = as_dict(*read_tum(a.ref))
    rows = {}
    cands = [("ORB-SLAM3 ref", Path(a.ref_live), Path(a.ref))] + [(n, d / f"{n}.txt", d / f"{n}.txt.optimized.txt") for n in names]
    for n, live_p, opt_p in cands:
        if not live_p.exists():
            continue
        r = {}
        side = Path(str(live_p) + ".json")
        if side.exists():
            j = json.loads(side.read_text())
            r["ok_ratio"] = j.get("ok_ratio")
            tm = j.get("odom_ms") or j.get("track_ms") or {}
            r["track_ms_mean"] = tm.get("mean") if isinstance(tm, dict) else None
            for key in ("loop_closures", "graph_nodes_final"):
                if key in j:
                    r[key] = j[key]
        for tag, p in (("live", live_p), ("final", opt_p)):
            if not p.exists():
                continue
            ts, T = read_tum(p)
            D = as_dict(ts, T)
            r[tag] = {"poses": len(ts), "rel_1s_vs_ref": rel_motion(ref, D), "ate_vs_ref": ate(ref, D),
                      "object_spread": object_spread(ts, T, X, frames)}
        rows[n] = r
    txt = json.dumps(rows, indent=1)
    if a.out:
        Path(a.out).write_text(txt)
    print(f"{'config':16s} {'ok':>5s} {'ms':>6s} | live: spread med/p90   rel1s rot95/tr95 | final: spread med/p90   rel1s rot95/tr95   ATE")
    for n, r in rows.items():
        def fmt(tag):
            if tag not in r:
                return " " * 40
            x = r[tag]
            return (f"{x['object_spread']['median_cm']:5.2f}/{x['object_spread']['p90_cm']:5.2f}  "
                    f"{x['rel_1s_vs_ref']['rot_p95_deg']:5.2f}/{x['rel_1s_vs_ref']['trans_p95_cm']:5.2f}  {x['ate_vs_ref']['rmse_cm']:5.2f}")
        ok = r.get("ok_ratio")
        ms = r.get("track_ms_mean")
        print(f"{n:16s} {('%.3f' % ok) if ok is not None else '  -  ':>5s} {('%.1f' % ms) if ms else '  -  ':>6s} | {fmt('live')} | {fmt('final')}")


if __name__ == "__main__":
    main()
