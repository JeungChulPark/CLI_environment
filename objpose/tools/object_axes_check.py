#!/usr/bin/env python3
"""object_axes_check.py — does each object's model frame follow the common convention? (2026-09-29)

Convention: origin at the box centre, +Z up, +Y back, +X right seen from the front.
Every SAM-6D estimate of a run is put in the world with the run's SLAM poses and rig extrinsic, and the world
direction of the object's front (-Y) and up (+Z) is taken. Objects standing on the same shelves face the same way,
so each object's front yaw is compared with the median over all objects (the shelf front). A consistent offset of
~90/180 deg means the MODEL frame is rotated -> fix it with sam6d_realtime/tools/set_object_frame.py (the suggested
--front/--up are printed). A spread of yaws (flips between estimates) means the ESTIMATE is ambiguous, not the frame.
(The first version used the camera direction in the object frame; on runs that circle the shelf it is biased.)
    python objpose/tools/object_axes_check.py <run dir> <extrinsic json> [--tau-s 0]
"""
import argparse, collections, json
import numpy as np
from scipy.spatial.transform import Rotation as Rot, Slerp

ap = argparse.ArgumentParser()
ap.add_argument("run"); ap.add_argument("extrinsic")
ap.add_argument("--tau-s", type=float, default=0.0, help="pose lookup offset (lane tau - SAM tau), normally 0")
a = ap.parse_args()
X = np.asarray(json.load(open(a.extrinsic))["T_slam_sam"], float).reshape(4, 4)
P = sorted([json.loads(l) for l in open(f"{a.run}/slam_poses.jsonl")], key=lambda p: p["t_ns"])
P = [p for p in P if p.get("T_wc")]
t = np.array([p["t_ns"] for p in P], float)
sl = Slerp(t, Rot.from_matrix(np.array([np.asarray(p["T_wc"], float).reshape(4, 4)[:3, :3] for p in P])))
W = collections.defaultdict(list)
for l in open(f"{a.run}/sam6d_estimates.jsonl"):
    e = json.loads(l); tq = e["t_ns"] + a.tau_s * 1e9
    if not (t[0] <= tq <= t[-1]): continue
    Rw = sl(tq).as_matrix() @ X[:3, :3] @ np.asarray(e["R"], float)
    W[e["object"]].append(Rw)
circ = lambda d: np.degrees(np.angle(np.mean(np.exp(1j * np.radians(d)))))
yaw = lambda v: np.degrees(np.arctan2(v[1], v[0]))
front = {n: circ([yaw(-R[:, 1]) for R in Rs]) for n, Rs in W.items()}
shelf = circ(list(front.values()))
print(f"shelf front (median of all objects): {shelf:+.0f} deg")
print(f"{'object':20s}   n  front yaw  off shelf  spread(p90)  up-z   verdict")
AX = ["+X", "+Y", "-X", "-Y"]          # model axes in the horizontal plane, counter-clockwise from +X
for n, Rs in sorted(W.items()):
    ys = np.array([yaw(-R[:, 1]) for R in Rs]); off = (front[n] - shelf + 180) % 360 - 180
    spread = np.percentile(np.abs((ys - front[n] + 180) % 360 - 180), 90)
    upz = np.median([R[2, 2] for R in Rs])
    if abs(upz) < 0.8:
        verdict = "up axis is not +Z -> check --up"
    elif spread > 60:
        verdict = "estimates flip between directions (estimate ambiguity, not the frame)"
    elif abs(off) > 45:
        k = int(round(off / 90)) % 4       # the model axis now pointing at the shelf front
        cur = {0: "-Y", 1: "+X", 2: "+Y", 3: "-X"}[(-k) % 4]
        verdict = f"frame rotated {off:+.0f} deg -> set_object_frame.py --object {n} --front={cur} --up=+Z"
    else:
        verdict = "ok"
    print(f"{n:20s} {len(Rs):3d}  {front[n]:+8.0f}  {off:+9.0f}  {spread:10.0f}  {upz:5.2f}   {verdict}")
