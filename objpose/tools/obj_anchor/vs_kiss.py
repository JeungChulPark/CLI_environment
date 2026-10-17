#!/usr/bin/env python3
"""vs_kiss.py — current-position error of the SAM camera against KISS-ICP (LiDAR odometry) as pseudo ground truth.
Trajectories (all SAM camera, data clock):
  KISS       T_w_lidar (KISS-ICP lane of a recorded run) . X_lidar_sam
  ORB live   T_wc streamed by ORB-SLAM3 . X_slam_sam           (what the hub has at that moment)
  ORB final  the same frames under ORB-SLAM3's final keyframes (after loop closures)
  anchored   object-anchored correction, map registered at the corrected pose (eval_real, slamcorr)
ORB world -> KISS world: one SE(3) fitted on the first --align-s seconds of ORB live (before any drift).
    python vs_kiss.py --orb <orb run> --kiss <kiss run> --session <SAM session> --tag T --out fig.png
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent)); sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "pc"))
import eval_real as ER                                        # noqa: E402
from obj_anchor import AnchorConfig                           # noqa: E402

REPO = Path(__file__).resolve().parents[3]
ap = argparse.ArgumentParser()
ap.add_argument("--orb", required=True); ap.add_argument("--kiss", required=True); ap.add_argument("--session", required=True)
ap.add_argument("--x", default=str(REPO / "objpose/rt/260915/X_slam_sam_260915.json"))
ap.add_argument("--xl", default=str(REPO / "objpose/rt/260915/X_lidar_sam_260915.json"))
ap.add_argument("--icp-hz", type=float, default=0.0); ap.add_argument("--loop-blend-s", type=float, default=0.0); ap.add_argument("--align-s", type=float, default=25.0); ap.add_argument("--tag", required=True); ap.add_argument("--title", default="")
ap.add_argument("--set", action="append", default=[], metavar="key=value", help="AnchorConfig override")
ap.add_argument("--tau-s", type=float, default=0.0, help="the run's SAM tau (260901 cbnu: -0.212)")
a = ap.parse_args()
X = np.asarray(json.loads(Path(a.x).read_text())["T_slam_sam"], float).reshape(4, 4)
XL = np.asarray(json.loads(Path(a.xl).read_text())["T_slam_sam"], float).reshape(4, 4)
cfg = AnchorConfig(use_rotation=False, min_obs=5, deadband_m=0.03, sig_obs_t0=0.03, max_rot_sigma=2.0, blend_s=0.5, loop_blend_s=a.loop_blend_s)
for kv in a.set:
    k, v = kv.split("="); cur = getattr(cfg, k)
    setattr(cfg, k, (v.lower() in ("1", "true")) if isinstance(cur, bool) else type(cur)(v))
ns = argparse.Namespace(run=a.orb, x=a.x, session=a.session, tau_s=a.tau_s, no_depth_check=False, k_npz=None, range_scale=1.0,
                        landmarks="memory", anchor_map="slamcorr", kf_anchors=True, reset_on_loop=True, icp_hz=a.icp_hz)
r = ER.run(ns, cfg)
kiss = [json.loads(l) for l in open(Path(a.kiss) / "slam_poses.jsonl")]
kt = np.array([int(m["t_ns"]) for m in kiss if m.get("T_wc")])
kp = np.array([(np.asarray(m["T_wc"], float).reshape(4, 4) @ XL)[:3, 3] for m in kiss if m.get("T_wc")])
ts = sorted(set(r["live_raw"]) & set(r["ref"]) & set(r["live_anc"]))
t0 = ts[0]
P = {k: np.array([(r[d][t] @ X)[:3, 3] for t in ts]) for k, d in (("live", "live_raw"), ("final", "ref"), ("anc", "live_anc"))}
ts = np.array(ts)
j = np.clip(np.searchsorted(kt, ts), 1, len(kt) - 1); j = np.where(np.abs(kt[j - 1] - ts) < np.abs(kt[j] - ts), j - 1, j)
ok = np.abs(kt[j] - ts) < 60_000_000
G = kp[j]
m = ok & ((ts - t0) / 1e9 <= a.align_s)
A, B = P["live"][m], G[m]
ma, mb = A.mean(0), B.mean(0); U, S, Vt = np.linalg.svd((A - ma).T @ (B - mb))
D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]); R = Vt.T @ D @ U.T; tt = mb - R @ ma
E = {k: np.linalg.norm(v @ R.T + tt - G, axis=1) * 1000 for k, v in P.items()}
fit = float(np.sqrt((E["live"][m] ** 2).mean()))
loops = [((u["t_ns"] - t0) / 1e9) for u in map(json.loads, open(Path(a.orb) / "kf_updates.jsonl")) if u.get("reason") not in ("periodic", "end")]
res = {"align_rmse_mm": fit, "loops_s": loops}
for k in P:                                   # path smoothness: frame-to-frame moves of the displayed camera
    dmv = np.linalg.norm(np.diff(P[k], axis=0), axis=1) * 1000
    res["jump_" + k] = {"max_mm": float(dmv.max()), "n_over_50mm": int((dmv > 50).sum())}
for k in E:
    e = E[k][ok]; res[k] = {"rmse_mm": float(np.sqrt((e ** 2).mean())), "median_mm": float(np.median(e)), "p90_mm": float(np.percentile(e, 90)), "max_mm": float(e.max())}
print(json.dumps(res, indent=1))
out = REPO / "objpose/output/obj_anchor_260924"
xy = {"xy_" + k: (v[ok] @ R.T + tt).tolist() for k, v in P.items()}
xy["xy_kiss"] = G[ok].tolist()
if r.get("icp_ms"):
    res["icp_ms_mean"] = float(np.mean(r["icp_ms"])); res["icp_ms_p95"] = float(np.percentile(r["icp_ms"], 95)); res["icp_aligned"] = r["icp_n"]
json.dump({**xy, "t": ((ts[ok] - t0) / 1e9).tolist(), **{"series_" + k: E[k][ok].tolist() for k in E}, **res}, open(out / f"vs_kiss_{a.tag}.json", "w"))
