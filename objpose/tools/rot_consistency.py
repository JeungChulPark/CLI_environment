#!/usr/bin/env python3
"""rot_consistency.py — do static objects drift in the world when the robot turns? (2026-09-29)

For one hub run: every SAM-6D estimate is placed in the world with the run's own SLAM poses and rig extrinsic,
  p_w = T_w_slam(t + tau) @ X_slam_sam @ t_cam_obj
(no keyframe anchoring / memory: the raw geometry the overlay depends on). Objects are static, so each estimate's
distance to its object's median world position measures overlay error. It is binned by the SLAM yaw rate at the
estimate time, and the spread is re-computed while sweeping the time offset tau and an extra yaw on the extrinsic:
if a non-zero tau or yaw collapses the turning error, the cause is clock / rig calibration, not the SLAM parameters.
    python objpose/tools/rot_consistency.py <run dir> <extrinsic json> [--label NAME]
"""
import argparse, json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot, Slerp

ap = argparse.ArgumentParser()
ap.add_argument("run"); ap.add_argument("extrinsic"); ap.add_argument("--label", default="")
ap.add_argument("--min-est", type=int, default=5)
a = ap.parse_args()
run = Path(a.run)
X = np.asarray(json.loads(Path(a.extrinsic).read_text())["T_slam_sam"], float).reshape(4, 4)
P = [json.loads(l) for l in open(run / "slam_poses.jsonl")]
P = [p for p in P if p.get("T_wc") is not None]
pt = np.array([p["t_ns"] for p in P], np.int64); order = np.argsort(pt); pt = pt[order]
PT = np.array([np.asarray(P[i]["T_wc"], float).reshape(4, 4) for i in order])
rots = Rot.from_matrix(PT[:, :3, :3]); slerp = Slerp(pt.astype(float), rots)
E = [json.loads(l) for l in open(run / "sam6d_estimates.jsonl")]
E = [e for e in E if pt[0] <= e["t_ns"] <= pt[-1]]

def pose_at(t):
    t = np.clip(t, pt[0], pt[-1]).astype(float)
    T = np.tile(np.eye(4), (len(t), 1, 1))
    T[:, :3, :3] = slerp(t).as_matrix()
    for k in range(3):
        T[:, k, 3] = np.interp(t, pt.astype(float), PT[:, k, 3])
    return T

names = np.array([e["object"] for e in E]); te = np.array([e["t_ns"] for e in E], float)
tco = np.array([np.asarray(e["t_mm"], float) / 1000.0 for e in E])
# yaw rate (deg/s) about the SLAM world's vertical: ±0.25 s rotation angle
dt = 0.25e9
Ra, Rb = slerp(np.clip(te - dt, pt[0], pt[-1])), slerp(np.clip(te + dt, pt[0], pt[-1]))
rate = np.degrees((Ra.inv() * Rb).magnitude()) / 0.5

def spread(tau_s=0.0, dyaw_deg=0.0):
    Xp = X.copy()
    if dyaw_deg:
        Xp[:3, :3] = Rot.from_euler("z", dyaw_deg, degrees=True).as_matrix() @ X[:3, :3]
        Xp[:3, 3] = Rot.from_euler("z", dyaw_deg, degrees=True).as_matrix() @ X[:3, 3]
    T = pose_at(te + tau_s * 1e9)
    pw = np.einsum("nij,nj->ni", T @ Xp, np.c_[tco, np.ones(len(tco))])[:, :3]
    dev = np.full(len(pw), np.nan)
    for n in set(names):
        m = names == n
        if m.sum() < a.min_est: continue
        med = np.median(pw[m], axis=0)
        dev[m] = np.linalg.norm(pw[m] - med, axis=1)
    return dev

dev0 = spread()
ok = ~np.isnan(dev0)
print(f"== {a.label or run.name}: {ok.sum()} estimates of {len(set(names[ok]))} objects (>= {a.min_est} each)")
bins = [(0, 5), (5, 15), (15, 30), (30, 1e9)]
for lo, hi in bins:
    m = ok & (rate >= lo) & (rate < hi)
    if m.sum(): print(f"   yaw rate {lo:>3.0f}-{hi if hi < 1e9 else 'inf':>3} deg/s: n {m.sum():4d}  world error median {np.nanmedian(dev0[m])*100:5.1f} cm  p90 {np.nanpercentile(dev0[m], 90)*100:5.1f} cm")
rs = [(tau, np.nanmedian(spread(tau)[ok])) for tau in np.arange(-0.5, 0.51, 0.05)]
tb, sb = min(rs, key=lambda x: x[1])
print(f"   tau sweep: tau 0 -> median {np.nanmedian(dev0[ok])*100:.1f} cm | best tau {tb:+.2f} s -> {sb*100:.1f} cm")
turn = ok & (rate >= 15)
if turn.sum() >= 5:
    rt = [(tau, np.nanmedian(spread(tau)[turn])) for tau in np.arange(-0.5, 0.51, 0.05)]
    tbt, sbt = min(rt, key=lambda x: x[1])
    print(f"   turning only (>=15 deg/s): tau 0 -> {np.nanmedian(dev0[turn])*100:.1f} cm | best tau {tbt:+.2f} s -> {sbt*100:.1f} cm")
ry = [(y, np.nanmedian(spread(0.0, y)[ok])) for y in np.arange(-10, 10.1, 1)]
yb, syb = min(ry, key=lambda x: x[1])
print(f"   extrinsic yaw sweep (tau 0): best extra yaw {yb:+.0f} deg -> {syb*100:.1f} cm")

# ---- what was DRAWN: display_objects.jsonl T_cam_obj vs the object's median world position seen from the same pose
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pc"))
Tws_all = None
D = [json.loads(l) for l in open(run / "display_objects.jsonl")]
wmed = {}
T_est = pose_at(te)
pw0 = np.einsum("nij,nj->ni", T_est @ X, np.c_[tco, np.ones(len(tco))])[:, :3]
for n in set(names):
    m = names == n
    if m.sum() >= a.min_est: wmed[n] = np.median(pw0[m], axis=0)
K = None
try:
    from conv_session import ConvSession
    import glob
    meta = json.loads((run / "summary.json").read_text()) if (run / "summary.json").exists() else {}
except Exception:
    meta = {}
fx = fy = 386.8; cx, cy = 322.8, 250.3            # 260901 SAM colour intrinsics (info.json K), for pixel units
rows = []
for d in D:
    t = d["t_ns"]
    if not (pt[0] <= t <= pt[-1]): continue
    Tw = pose_at(np.array([t], float))[0] @ X
    Rw = slerp(np.clip(np.array([t - dt, t + dt], float), pt[0], pt[-1]))
    r_ = np.degrees((Rw[0].inv() * Rw[1]).magnitude()) / 0.5
    for o in d["objects"]:
        if o["name"] not in wmed or o.get("T_cam_obj") is None: continue
        pc = np.asarray(o["T_cam_obj"], float).reshape(4, 4)[:3, 3]
        pe = (np.linalg.inv(Tw) @ np.r_[wmed[o["name"]], 1])[:3]
        if pc[2] < 0.2 or pe[2] < 0.2: continue
        uc = np.array([fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy]); ue = np.array([fx * pe[0] / pe[2] + cx, fy * pe[1] / pe[2] + cy])
        if not (0 <= ue[0] < 640 and 0 <= ue[1] < 480): continue
        rows.append((r_, float(np.linalg.norm(uc - ue)), o["source"], o["age_s"], float(np.linalg.norm(pc - pe))))
if rows:
    R_ = np.array([r[0] for r in rows]); E_ = np.array([r[1] for r in rows]); M_ = np.array([r[4] for r in rows])
    print(f"   DRAWN overlay vs object's world median (in view, {len(rows)} object-frames):")
    for lo, hi in bins:
        m = (R_ >= lo) & (R_ < hi)
        if m.sum(): print(f"      yaw rate {lo:>3.0f}-{hi if hi < 1e9 else 'inf':>3} deg/s: n {m.sum():5d}  pixel error median {np.median(E_[m]):5.1f} px  p90 {np.percentile(E_[m], 90):6.1f} px  | 3D median {np.median(M_[m])*100:5.1f} cm")
