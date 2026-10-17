#!/usr/bin/env python3
"""depth_overlay_check.py — how well does the Object Pose Map overlay sit on the real objects, checked from the depth
image alone (no SAM-6D): every map object's CAD points are put in the camera with the pose the display uses, the front
surface is kept (z-buffer), and its depth is compared with the aligned depth image at those pixels:
    match     |measured - rendered| < 2 cm
    occluded  measured > 5 cm in FRONT of the rendered surface (something covers the object)
    free      measured > 5 cm BEHIND it (the object is not where it is drawn)
    fit       match / (valid - occluded)            (1 = sits on the depth, 0 = nothing there)
Two displays are compared at the same frames: SLAM pose + SLAM-pose map, and anchored pose + anchored map.
    python depth_overlay_check.py <run> --x X --session S [--tau-s] --t0 0 --t1 91 --hz 5 --tag T [eval_real flags]
"""
import argparse, json, sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent)); sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "pc"))
import eval_real as ER                                            # noqa: E402
from obj_anchor import AnchorConfig, inv                          # noqa: E402

REPO = Path(__file__).resolve().parents[3]
PTS = REPO / "sam6d_realtime/assets/model_points"
_pts = {}


def model(nm, n=1500):
    if nm not in _pts:
        p = np.load(PTS / f"{nm}.npy").astype(np.float64) / 1000.0
        if len(p) > n:
            p = p[np.random.default_rng(0).choice(len(p), n, replace=False)]
        _pts[nm] = p
    return _pts[nm]


def check(T_w_cam, lms, depth_m, K, ds=2):
    """per object: (name, fit, match, occluded, free, valid)"""
    H, W = depth_m.shape
    H2, W2 = H // ds, W // ds
    Tcw = inv(T_w_cam)
    out = []
    for nm, T_w_obj in lms:
        T = Tcw @ T_w_obj
        pc = model(nm) @ T[:3, :3].T + T[:3, 3]
        pc = pc[pc[:, 2] > 0.1]
        if len(pc) == 0:
            continue
        u = K[0, 0] * pc[:, 0] / pc[:, 2] + K[0, 2]
        v = K[1, 1] * pc[:, 1] / pc[:, 2] + K[1, 2]
        ok = (u >= 0) & (u < W) & (v >= 0) & (v < H)
        if ok.sum() < 30:
            continue
        ui, vi, z = (u[ok] // ds).astype(int), (v[ok] // ds).astype(int), pc[ok, 2]
        idx = vi * W2 + ui
        zb = np.full(H2 * W2, np.inf)
        np.minimum.at(zb, idx, z)                      # front surface only
        pix = np.flatnonzero(np.isfinite(zb))
        zr = zb[pix]
        d = depth_m[(pix // W2) * ds, (pix % W2) * ds]
        val = d > 0.1
        if val.sum() < 20:
            continue
        r = d[val] - zr[val]
        n = len(r); m = int((np.abs(r) < 0.02).sum()); oc = int((r < -0.05).sum()); fr = int((r > 0.05).sum())
        out.append((nm, m / max(n - oc, 1), m, oc, fr, n))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run"); ap.add_argument("--x", required=True); ap.add_argument("--session", required=True)
    ap.add_argument("--tau-s", type=float, default=0.0); ap.add_argument("--t0", type=float, default=0.0)
    ap.add_argument("--t1", type=float, default=1e9); ap.add_argument("--hz", type=float, default=5.0); ap.add_argument("--tag", required=True)
    ap.add_argument("--set", action="append", default=[]); ap.add_argument("--anchor-map", default="slamcorr")
    ap.add_argument("--kf-anchors", action="store_true"); ap.add_argument("--reset-on-loop", action="store_true")
    a = ap.parse_args()
    cfg = AnchorConfig()
    for kv in a.set:
        k, v = kv.split("="); cur = getattr(cfg, k)
        setattr(cfg, k, tuple(float(x) for x in v.split(",")) if isinstance(cur, tuple) else (v.lower() in ("1", "true")) if isinstance(cur, bool) else type(cur)(v))
    dep = ER.Depths(a.session, int(round(a.tau_s * 1e9)))
    first = min(json.loads(l)["t_ns"] for l in open(Path(a.run) / "slam_poses.jsonl"))
    st = dep.s.t_ns
    pick, nxt = [], first + a.t0 * 1e9
    for i, t in enumerate(st):
        if t >= nxt and t <= first + a.t1 * 1e9:
            pick.append(i); nxt = t + 1e9 / a.hz
    ns = argparse.Namespace(run=a.run, x=a.x, session=a.session, tau_s=a.tau_s, no_depth_check=False, k_npz=None, range_scale=1.0,
                            landmarks="memory", anchor_map=a.anchor_map, kf_anchors=a.kf_anchors, reset_on_loop=a.reset_on_loop,
                            snap_ns=[int(st[i]) for i in pick], snaps=[])
    r = ER.run(ns, cfg)
    X = np.asarray(json.loads(Path(a.x).read_text())["T_slam_sam"], float).reshape(4, 4)
    K = np.asarray(dep.K, float)
    fi = {int(st[i]): i for i in pick}
    rt = np.array(sorted(r["ref"]))
    rows, ms = [], []
    for s in ns.snaps:
        i = fi.get(int(s["t"]))
        if i is None:
            continue
        depth_m = dep.s.read(i, True).depth_raw.astype(np.float64) / 1000.0
        t0 = time.perf_counter()
        a_hub = check(s["T_hub"] @ X, s["hub_lms"], depth_m, K)
        ms.append((time.perf_counter() - t0) * 1e3)
        a_anc = check(s["T_anc"] @ X, s["anc_lms"], depth_m, K)
        j = int(np.argmin(np.abs(rt - s["t"])))
        cam_err = float(np.linalg.norm((s["T_hub"] @ X)[:3, 3] - (r["ref"][int(rt[j])] @ X)[:3, 3]) * 1000)
        anc_err = float(np.linalg.norm((s["T_anc"] @ X)[:3, 3] - (r["ref"][int(rt[j])] @ X)[:3, 3]) * 1000)
        f = lambda A: float(np.mean([x[1] for x in A])) if A else None
        fr = lambda A: float(np.sum([x[4] for x in A]) / max(np.sum([x[5] for x in A]), 1)) if A else None
        rows.append({"t": (s["t"] - first) / 1e9, "fit_slam": f(a_hub), "fit_anc": f(a_anc), "free_slam": fr(a_hub), "free_anc": fr(a_anc),
                     "n_slam": len(a_hub), "n_anc": len(a_anc), "cam_err_slam_mm": cam_err, "cam_err_anc_mm": anc_err,
                     "objs_slam": [(x[0], round(x[1], 3)) for x in a_hub]})
    ms = np.array(ms)
    R = [x for x in rows if x["fit_slam"] is not None]
    fs = np.array([x["fit_slam"] for x in R]); ce = np.array([x["cam_err_slam_mm"] for x in R])
    fa = np.array([x["fit_anc"] for x in R if x["fit_anc"] is not None])
    print(f"{a.tag}: frames {len(rows)} (with objects {len(R)}) | depth check {np.median(ms):.2f} ms median, {np.percentile(ms, 95):.2f} p95, max {ms.max():.2f} ms per frame")
    print(f"  fit SLAM display: mean {fs.mean():.3f} | anchored display: mean {fa.mean():.3f}")
    print(f"  corr(fit, camera error vs final trajectory) = {np.corrcoef(fs, ce)[0, 1]:+.2f}")
    for lo, hi in ((0, 30), (30, 100), (100, 200), (200, 1e9)):
        m = (ce >= lo) & (ce < hi)
        if m.any():
            print(f"  camera error {lo:>3.0f}-{hi if hi < 1e8 else 'inf'} mm: n {m.sum():3d}  fit {fs[m].mean():.3f}")
    out = REPO / "objpose/output/obj_anchor_260924"
    json.dump({"rows": rows, "ms_median": float(np.median(ms)), "ms_p95": float(np.percentile(ms, 95))}, open(out / f"depthcheck_{a.tag}.json", "w"))


if __name__ == "__main__":
    main()
