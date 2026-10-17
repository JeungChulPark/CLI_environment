#!/usr/bin/env python3
"""fit_extrinsic.py — is the rig extrinsic X (T_slam_sam) why the same object lands in different map places?

For a recorded live run every SAM-6D estimate is placed in the map with the FINAL SLAM trajectory (after loop
closures / BA):  p_map = T_map_slam(t + tau) . D . X . p_sam,   D = small correction (rotation, translation) in the
SLAM-camera frame. Objects do not move, so all estimates of one object should coincide. We find D (and optionally
tau) that makes each object's estimates collapse, with the object positions eliminated (per-class mean inside the
residual), robust loss, weak prior on D (the component along the vertical axis of a planar rig is not observable).

Checked on data not used for the fit: other recordings of the same rig, or the other half of the same run.
Metric: spread = distance of each estimate to its object's median position (median / p90 over all estimates).

    python fit_extrinsic.py --x rt/260915/X_slam_sam_260915.json --train runA runB --test runC runD
    python fit_extrinsic.py --x ... --train runA --split-half          # first half fits, second half tests (and swapped)
"""
from __future__ import annotations

import argparse, json, sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/pc"))
from fusion import PoseBuffer                                     # noqa: E402

OUT = REPO / "objpose/output/obj_anchor_260924"


def jl(p):
    return [json.loads(l) for l in Path(p).read_text().splitlines() if l.strip()]


def final_poses(run):
    pb = PoseBuffer(0.25)
    ev = [(int(m["t_ns"]), 0, m) for m in jl(run / "slam_poses.jsonl")] + [(int(u["t_ns"]), 1, u) for u in jl(run / "kf_updates.jsonl")]
    ev.sort(key=lambda e: (e[0], e[1]))
    for t, k, m in ev:
        if k == 0:
            if m.get("T_wc") is None or m.get("state") not in ("OK", "OK_KLT"):
                continue
            pb.add(t, m["state"], m["T_wc"], m.get("map_id"), kf=m.get("ref_kf"), T_w_kf=m.get("T_w_kf"),
                   kf_map_id=m.get("kf_map_id"), map_changed=bool(m.get("map_changed")))
        else:
            pb.apply_kf_update(m.get("map_id"), m.get("kfs", []))
    return pb


class Run:
    def __init__(self, path, tau_s=0.0, min_obs=10):
        self.path = Path(path)
        self.name = self.path.name
        self.pb = final_poses(self.path)
        rows = []
        for e in jl(self.path / "sam6d_estimates.jsonl"):
            rows.append((int(e["t_ns"]), e["object"], np.asarray(e["t_mm"], float) / 1000.0))
        n = {}
        for _, nm, _ in rows:
            n[nm] = n.get(nm, 0) + 1
        self.rows = [r for r in rows if n[r[1]] >= min_obs]
        self.t0 = min(r[0] for r in self.rows)
        self.t1 = max(r[0] for r in self.rows)
        self.set_tau(tau_s)

    def set_tau(self, tau_s):
        R, t, P, cls, ts = [], [], [], [], []
        for tn, nm, p in self.rows:
            T, _ = self.pb.at(int(tn + tau_s * 1e9))
            if T is None:
                continue
            R.append(T[:3, :3]); t.append(T[:3, 3]); P.append(p); cls.append(nm); ts.append(tn)
        self.R, self.t, self.P = np.array(R), np.array(t), np.array(P)
        names = sorted(set(cls))
        self.ci = np.array([names.index(c) for c in cls])
        self.ncls = len(names)
        self.ts = np.array(ts)

    def subset(self, mask):
        o = object.__new__(Run)
        o.__dict__.update(self.__dict__)
        o.R, o.t, o.P, o.ci, o.ts = self.R[mask], self.t[mask], self.P[mask], self.ci[mask], self.ts[mask]
        o.name = self.name + ("[A]" if mask[0] else "[B]")
        return o


def world(run, X, d):
    D = np.eye(4)
    D[:3, :3] = Rotation.from_rotvec(d[:3]).as_matrix()
    D[:3, 3] = d[3:6]
    M = D @ X
    P = run.P * (np.exp(d[6]) if len(d) > 6 else 1.0)          # optional range scale of the SAM-6D translation
    q = P @ M[:3, :3].T + M[:3, 3]                           # in the SLAM camera
    return np.einsum("nij,nj->ni", run.R, q) + run.t


def spread(runs, X, d):
    e = []
    for r in runs:
        W = world(r, X, d)
        for c in range(r.ncls):
            m = r.ci == c
            if m.sum() < 3:
                continue
            e.append(np.linalg.norm(W[m] - np.median(W[m], 0), axis=1))
    e = np.concatenate(e) * 1000
    return {"median_mm": float(np.median(e)), "p90_mm": float(np.percentile(e, 90)), "n": int(len(e))}


def fit(runs, X, prior_r_deg=5.0, prior_t_m=0.10, scale=False, only_scale=False):
    n = 7 if scale or only_scale else 6

    def res(d):
        if only_scale:
            d = np.r_[np.zeros(6), d[6:]]
        out = [d[:3] / np.deg2rad(prior_r_deg), d[3:6] / prior_t_m] + ([d[6:] / 0.2] if n == 7 else [])
        for r in runs:
            W = world(r, X, d)
            for c in range(r.ncls):
                m = r.ci == c
                if m.sum() < 3:
                    continue
                out.append((W[m] - W[m].mean(0)).ravel())
        return np.concatenate(out)
    sol = least_squares(res, np.zeros(n), loss="soft_l1", f_scale=0.03, max_nfev=200)
    d = sol.x
    return np.r_[np.zeros(6), d[6:]] if only_scale else d


def describe(d):
    return {"rot_deg": np.round(np.rad2deg(d[:3]), 2).tolist(), "rot_norm_deg": round(float(np.rad2deg(np.linalg.norm(d[:3]))), 2),
            "trans_cm": np.round(d[3:6] * 100, 1).tolist(), "trans_norm_cm": round(float(np.linalg.norm(d[3:6]) * 100), 1),
            "range_scale": round(float(np.exp(d[6])), 4) if len(d) > 6 else 1.0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", required=True)
    ap.add_argument("--train", nargs="+", required=True)
    ap.add_argument("--test", nargs="*", default=[])
    ap.add_argument("--tau-s", type=float, default=0.0)
    ap.add_argument("--tau-grid", action="store_true", help="also search a common SAM time offset (+-150 ms)")
    ap.add_argument("--split-half", action="store_true")
    ap.add_argument("--tag", required=True)
    ap.add_argument("--model", choices=["X", "X+scale", "scale"], default="X")
    a = ap.parse_args()
    X = np.asarray(json.loads(Path(a.x).read_text())["T_slam_sam"], float).reshape(4, 4)
    base = REPO / "objpose/output"
    tr = [Run(base / p if not Path(p).is_absolute() else p, a.tau_s) for p in a.train]
    te = [Run(base / p if not Path(p).is_absolute() else p, a.tau_s) for p in a.test]
    out = {"x": a.x, "train": a.train, "test": a.test}
    if a.split_half:
        folds = []
        for r in tr:
            mid = (r.t0 + r.t1) / 2
            A, B = r.subset(r.ts < mid), r.subset(r.ts >= mid)
            for fa, fb, lab in ((A, B, "fit 1st half -> test 2nd"), (B, A, "fit 2nd half -> test 1st")):
                d = fit([fa], X, scale=a.model == 'X+scale', only_scale=a.model == 'scale')
                folds.append({"run": r.name, "fold": lab, "D": describe(d), "test_before": spread([fb], X, np.zeros(6)),
                              "test_after": spread([fb], X, d), "train_before": spread([fa], X, np.zeros(6)), "train_after": spread([fa], X, d)})
                f = folds[-1]
                print(f"{r.name} {lab}: D {f['D']}  test spread {f['test_before']['median_mm']:.1f}/{f['test_before']['p90_mm']:.1f}"
                      f" -> {f['test_after']['median_mm']:.1f}/{f['test_after']['p90_mm']:.1f} mm (median/p90)", flush=True)
        out["folds"] = folds
    else:
        taus = [a.tau_s] if not a.tau_grid else [a.tau_s + k * 0.025 for k in range(-6, 7)]
        best = None
        for tau in taus:
            for r in tr + te:
                r.set_tau(tau)
            d = fit(tr, X, scale=a.model == 'X+scale', only_scale=a.model == 'scale')
            s = spread(tr, X, d)
            print(f"tau {tau * 1000:+.0f} ms: train spread {s['median_mm']:.1f}/{s['p90_mm']:.1f} mm, D {describe(d)}", flush=True)
            if best is None or s["median_mm"] < best[2]["median_mm"]:
                best = (tau, d, s)
        tau, d, _ = best
        for r in tr + te:
            r.set_tau(a.tau_s)
        z = np.zeros(len(d))
        b_tr, b_te = spread(tr, X, np.zeros(6)), (spread(te, X, np.zeros(6)) if te else None)
        for r in tr + te:
            r.set_tau(tau)
        out.update(tau_s=tau, D=describe(d), D_vec=d.tolist(), train_before=b_tr, train_after=spread(tr, X, d),
                   per_test=[{"run": r.name, "before": None, "after": spread([r], X, d)} for r in te])
        for r in te:
            r.set_tau(a.tau_s)
        for pt, r in zip(out["per_test"], te):
            pt["before"] = spread([r], X, np.zeros(6))
        print(f"\nfit on {len(tr)} runs: tau {tau * 1000:+.0f} ms, D = {out['D']}")
        print(f"train spread {b_tr['median_mm']:.1f}/{b_tr['p90_mm']:.1f} -> {out['train_after']['median_mm']:.1f}/{out['train_after']['p90_mm']:.1f} mm")
        for pt in out["per_test"]:
            print(f"  test {pt['run']}: {pt['before']['median_mm']:.1f}/{pt['before']['p90_mm']:.1f} -> {pt['after']['median_mm']:.1f}/{pt['after']['p90_mm']:.1f} mm")
        Xn = np.eye(4)
        Xn[:3, :3] = Rotation.from_rotvec(d[:3]).as_matrix(); Xn[:3, 3] = d[3:6]
        Xn = Xn @ X
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / f"X_fit_{a.tag}.json").write_text(json.dumps({"T_slam_sam": Xn.tolist(), "sam_tau_s": tau, "range_scale": float(np.exp(d[6])) if len(d) > 6 else 1.0,
                                                             "source": f"object-consistency fit ({a.tag}) on top of {a.x}"}, indent=1))
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"xfit_{a.tag}.json").write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
