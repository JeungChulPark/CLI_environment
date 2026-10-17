#!/usr/bin/env python3
"""replace_rt.py — re-place a finished run's SAM-6D estimates with a different camera RT.

Same placement as the hub (objpose/pc/fusion.py): T_w_obj = T_w_slam(t) . X . T_sam_obj, with the
SLAM pose keyframe-corrected exactly the way objpose/pc/compare_runs.py:load_run() does it (last
kf_update snapshot) and SE(3)-interpolated between the bracketing poses (gap <= 0.25 s).
Scores the static-object spread (median / p90 of |p - median(p)| per object) and the
neighbour-observation distance by time gap, for each RT given.

    python3 objpose/rt/260915/replace_rt.py <run dir> old=<X.json> new=<X.json> --out <report.json>
"""
from __future__ import annotations
import argparse, json, sys
from pathlib import Path
import numpy as np

REPO = Path("/home/jucpark/DeepLearning/CLI_environment")
sys.path.insert(0, str(REPO / "objpose" / "pc"))
from fusion import interp_se3, mat  # noqa: E402


def load_poses(d: Path):
    poses = [json.loads(l) for l in open(d / "slam_poses.jsonl")]
    kfu = [json.loads(l) for l in open(d / "kf_updates.jsonl")] if (d / "kf_updates.jsonl").exists() else []
    last_kf = {int(r[0]): mat(r[1:]) for r in kfu[-1]["kfs"]} if kfu else {}
    t, T = [], []
    for m in poses:
        if m.get("state") not in ("OK", "OK_KLT") or m.get("T_wc") is None:
            continue
        M = mat(m["T_wc"])
        if m.get("ref_kf") is not None and m.get("T_w_kf") is not None and int(m["ref_kf"]) in last_kf:
            M = last_kf[int(m["ref_kf"])] @ np.linalg.inv(mat(m["T_w_kf"])) @ M
        t.append(int(m["t_ns"])); T.append(M)
    o = np.argsort(t)
    return np.array(t)[o], np.stack(T)[o]


def at(ts, Ts, t_ns, max_gap_ns=250_000_000):
    i = int(np.searchsorted(ts, t_ns))
    if i == 0 or i >= len(ts):
        return None
    if ts[i] - ts[i - 1] > max_gap_ns:
        return None
    u = (t_ns - ts[i - 1]) / max(ts[i] - ts[i - 1], 1)
    return interp_se3(Ts[i - 1], Ts[i], float(u))


def score(run: Path, X):
    ts, Ts = load_poses(run)
    per = {}
    for line in open(run / "sam6d_estimates.jsonl"):
        e = json.loads(line)
        T = at(ts, Ts, int(e["t_ns"]))
        if T is None:
            continue
        M = np.eye(4); M[:3, :3] = np.array(e["R"]); M[:3, 3] = np.array(e["t_mm"]) / 1000.0
        per.setdefault(e["object"], []).append((int(e["t_ns"]), (T @ X @ M)[:3, 3]))
    spread, gaps = [], []
    for L in per.values():
        L.sort()
        P = np.array([p for _, p in L])
        spread += list(np.linalg.norm(P - np.median(P, 0), axis=1))
        for (ta, pa), (tb, pb) in zip(L, L[1:]):
            gaps.append(((tb - ta) / 1e9, float(np.linalg.norm(pb - pa))))
    s = np.array(spread) * 100
    g = np.array(gaps)
    out = {"objects": len(per), "observations": int(len(s)),
           "spread_median_cm": round(float(np.median(s)), 2), "spread_p90_cm": round(float(np.percentile(s, 90)), 2),
           "spread_mean_cm": round(float(s.mean()), 2),
           "per_object_spread_median_cm": {k: round(float(np.median(np.linalg.norm(
               np.array([p for _, p in v]) - np.median([p for _, p in v], 0), axis=1)) * 100), 2)
               for k, v in sorted(per.items())},
           "per_object_n": {k: len(v) for k, v in sorted(per.items())}}
    for lo, hi, name in ((0, 3, "le3s"), (3, 10, "3_10s"), (10, 60, "10_60s")):
        m = (g[:, 0] > lo) & (g[:, 0] <= hi)
        if m.sum():
            out[f"neighbour_cm_{name}"] = [round(float(np.median(g[m, 1]) * 100), 2),
                                           round(float(np.percentile(g[m, 1], 90) * 100), 2), int(m.sum())]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("rts", nargs="+", help="name=path/to/X.json")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rep = {"run": a.run, "rts": {}}
    for spec in a.rts:
        name, _, path = spec.partition("=")
        X = np.asarray(json.loads(Path(path).read_text())["T_slam_sam"], np.float64)
        rep["rts"][name] = {"path": path, **score(Path(a.run), X)}
        print(name, json.dumps({k: v for k, v in rep["rts"][name].items() if "per_object" not in k}), flush=True)
    Path(a.out).write_text(json.dumps(rep, indent=1))
    print("->", a.out)


if __name__ == "__main__":
    main()
