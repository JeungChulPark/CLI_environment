#!/usr/bin/env python3
"""overlay_error_decompose.py — why does the drawn box drift during turns? (2026-09-29)

For every drawn object-frame of a hub run (display_objects.jsonl, T_cam_obj = what the viewer drew):
  A(t)  = the SLAM pose the hub had at display time   (KISS: raw pose unless the smoothed one had arrived
          before display (refine_lag <= display_delay); HDL/ORB: pose with the keyframe corrections received up to t)
  B(t)  = the final pose                              (KISS: smoothed; HDL/ORB: final keyframe corrections)
  p_mem = A(t) X p_cam_drawn                         (the memory landmark the hub drew; recovered per frame)
  p_true= median over the run's SAM-6D estimates placed with B
  drawn error            = | proj( inv(B X) p_true ) - proj(p_cam_drawn) |
    = pose part  (a)     = | proj( inv(B X) p_mem ) - proj( inv(A X) p_mem ) |   display-time pose vs final pose
    + memory part (b)    = | proj( inv(B X) p_true) - proj( inv(B X) p_mem ) |   landmark position vs truth
  all binned by the yaw rate at t. Also: jitter of p_mem (should be ~0 if A is really what was drawn).
    python objpose/tools/overlay_error_decompose.py <run dir> <extrinsic json> --kind kiss|hdl|orb
"""
import argparse, json, bisect
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot, Slerp

ap = argparse.ArgumentParser()
ap.add_argument("run"); ap.add_argument("extrinsic"); ap.add_argument("--kind", required=True, choices=["kiss", "hdl", "orb"])
ap.add_argument("--display-delay", type=float, default=0.3)
a = ap.parse_args()
run = Path(a.run)
X = np.asarray(json.loads(Path(a.extrinsic).read_text())["T_slam_sam"], float).reshape(4, 4)
M = lambda v: np.asarray(v, float).reshape(4, 4)
P = sorted([json.loads(l) for l in open(run / "slam_poses.jsonl")], key=lambda p: p["t_ns"])
P = [p for p in P if p.get("T_wc") is not None]
t = np.array([p["t_ns"] for p in P], float)

if a.kind == "kiss":
    B = np.array([M(p["T_wc"]) for p in P])
    A = np.array([M(p["T_wc"]) if (p.get("refine_lag_s") is not None and p["refine_lag_s"] <= a.display_delay) else M(p.get("T_wc_raw", p["T_wc"])) for p in P])
    print(f"KISS poses {len(P)}: smoothed pose arrived before display for {sum(1 for p in P if p.get('refine_lag_s') is not None and p['refine_lag_s'] <= a.display_delay)}")
    Afun = lambda td: A
else:
    K = [json.loads(l) for l in open(run / "kf_updates.jsonl")]
    K.sort(key=lambda k: k["t_ns"])
    def kf_state(upto_ns):
        st = {}
        for k in K:
            if k["t_ns"] > upto_ns: break
            for row in k["kfs"]:
                st[int(row[0])] = np.vstack([np.asarray(row[1:13], float).reshape(3, 4), [0, 0, 0, 1]])
        return st
    final = kf_state(float("inf"))
    def corrected(st):
        out = []
        for p in P:
            T = M(p["T_wc"]); kf = p.get("ref_kf")
            if kf is not None and p.get("T_w_kf") is not None and int(kf) in st:
                T = st[int(kf)] @ np.linalg.inv(M(p["T_w_kf"])) @ T
            out.append(T)
        return np.array(out)
    B = corrected(final)
    kt = [k["t_ns"] for k in K]
    cache = {}
    def Afun(td):   # pose set as the hub had it at display time td (kf updates with data time <= td)
        n = bisect.bisect_right(kt, td)
        if n not in cache: cache[n] = corrected(kf_state(kt[n - 1]) if n else {})
        return cache[n]

def interp(Ts, tq):
    tq = float(np.clip(tq, t[0], t[-1]))
    k = int(np.clip(np.searchsorted(t, tq), 1, len(t) - 1)); u = (tq - t[k - 1]) / (t[k] - t[k - 1])
    R = Slerp([0, 1], Rot.from_matrix([Ts[k - 1][:3, :3], Ts[k][:3, :3]]))(u).as_matrix()
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = (1 - u) * Ts[k - 1][:3, 3] + u * Ts[k][:3, 3]
    return T

fx = fy = 386.8; cx, cy = 322.8, 250.3
proj = lambda p: np.array([fx * p[0] / p[2] + cx, fy * p[1] / p[2] + cy])
E = [json.loads(l) for l in open(run / "sam6d_estimates.jsonl")]
ptrue = {}
for n in set(e["object"] for e in E):
    ws = [(interp(B, e["t_ns"]) @ X @ np.r_[np.asarray(e["t_mm"]) / 1000, 1])[:3] for e in E if e["object"] == n and t[0] <= e["t_ns"] <= t[-1]]
    if len(ws) >= 5: ptrue[n] = np.median(ws, axis=0)
D = [json.loads(l) for l in open(run / "display_objects.jsonl")]
rows = []
for d in D:
    td = d["t_ns"]
    if not (t[0] <= td <= t[-1]): continue
    TA = interp(Afun(td), td) @ X; TB = interp(B, td) @ X
    Ra = Rot.from_matrix(interp(B, td - 0.25e9)[:3, :3]); Rb = Rot.from_matrix(interp(B, td + 0.25e9)[:3, :3])
    rate = np.degrees((Ra.inv() * Rb).magnitude()) / 0.5
    for o in d["objects"]:
        if o["name"] not in ptrue or o.get("T_cam_obj") is None: continue
        pc = M(o["T_cam_obj"])[:3, 3]
        pmem = (TA @ np.r_[pc, 1])[:3]
        pB_mem = (np.linalg.inv(TB) @ np.r_[pmem, 1])[:3]; pB_true = (np.linalg.inv(TB) @ np.r_[ptrue[o["name"]], 1])[:3]
        if min(pc[2], pB_mem[2], pB_true[2]) < 0.2: continue
        u_true = proj(pB_true)
        if not (0 <= u_true[0] < 640 and 0 <= u_true[1] < 480): continue
        rows.append((rate, np.linalg.norm(u_true - proj(pc)), np.linalg.norm(proj(pB_mem) - proj(pc)),
                     np.linalg.norm(u_true - proj(pB_mem)), o["name"], td, pmem))
R = np.array([r[0] for r in rows]); tot = np.array([r[1] for r in rows]); ea = np.array([r[2] for r in rows]); eb = np.array([r[3] for r in rows])
print(f"== {run.name} ({a.kind}): {len(rows)} drawn object-frames in view")
for lo, hi in [(0, 5), (5, 15), (15, 30), (30, 1e9)]:
    m = (R >= lo) & (R < hi)
    if m.sum(): print(f"   yaw {lo:>3.0f}-{hi if hi < 1e9 else 'inf':>3} deg/s n {m.sum():5d} | drawn error {np.median(tot[m]):5.1f} px | (a) pose at display vs final {np.median(ea[m]):5.1f} px | (b) memory landmark vs truth {np.median(eb[m]):5.1f} px")
# jitter of the recovered landmark between consecutive frames (0 if A is what the hub used)
jit = []
by = {}
for r in rows: by.setdefault(r[4], []).append((r[5], r[6]))
for n, v in by.items():
    v.sort(key=lambda x: x[0])
    for (t1, p1), (t2, p2) in zip(v, v[1:]):
        if t2 - t1 < 0.1e9: jit.append(np.linalg.norm(p2 - p1))
print(f"   recovered landmark frame-to-frame jump: median {np.median(jit)*1000:.1f} mm, p90 {np.percentile(jit, 90)*1000:.1f} mm (small = A reproduces what was drawn)")
