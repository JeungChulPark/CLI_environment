#!/usr/bin/env python3
"""diag_anchor.py — why are the frozen anchors of a real run centimetres away from the objects' final-trajectory positions?

For every object, the anchor is its memory pose when it reached 5 observations. Candidate explanations:
  (a) SLAM drift / pre-loop-closure coordinates at that moment -> place the SAME first observations with the FINAL
      trajectory: if the gap disappears, (a) is the cause
  (b) single-viewpoint bias of the estimates (view-dependent pose/scale error) -> the first observations placed with the
      final trajectory still differ from the all-viewpoint median
  (c) chained drift of the corrected frame (anchors registered with already-corrected poses) -> anchor registered with
      live SLAM poses vs with corrected poses
Also lists loop-closure events against the anchored error jumps.
    python diag_anchor.py <run> --x <X json>
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from fit_extrinsic import final_poses, jl                             # noqa: E402

ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--x", required=True); ap.add_argument("--first", type=int, default=5)
a = ap.parse_args()
run = Path(a.run); X = np.asarray(json.loads(Path(a.x).read_text())["T_slam_sam"], float).reshape(4, 4)
fin = final_poses(run)
# live pose at a time = the pose message itself (what the hub had then)
live = {int(m["t_ns"]): np.asarray(m["T_wc"], float).reshape(4, 4) for m in jl(run / "slam_poses.jsonl") if m.get("T_wc")}
lt = np.array(sorted(live))
est = sorted(jl(run / "sam6d_estimates.jsonl"), key=lambda e: e["t_ns"])
t0 = int(lt[0])
by = {}
for e in est:
    s = int(e["t_ns"]); Tf, _ = fin.at(s)
    j = int(np.clip(np.searchsorted(lt, s), 1, len(lt) - 1)); k = j if abs(lt[j] - s) < abs(lt[j - 1] - s) else j - 1
    if Tf is None or abs(lt[k] - s) > 50_000_000:
        continue
    p = np.asarray(e["t_mm"]) / 1000.0
    pf = (Tf @ X)[:3, :3] @ p + (Tf @ X)[:3, 3]
    Tl = live[int(lt[k])] @ X
    pl = Tl[:3, :3] @ p + Tl[:3, 3]
    cam_err = np.linalg.norm((live[int(lt[k])])[:3, 3] - Tf[:3, 3])
    by.setdefault(e["object"], []).append(((s - t0) / 1e9, pf, pl, cam_err, (Tf @ X)[:3, 3]))
print(f"{run.name}: first {a.first} observations of each object")
print(f"{'object':22s} {'t(s)':>6s} {'cam err':>8s} {'(a) live-first vs final-all':>28s} {'(b) final-first vs final-all':>29s} {'view spread':>11s}")
A, B = [], []
for nm, v in sorted(by.items()):
    if len(v) < a.first + 3:
        continue
    allf = np.median(np.array([x[1] for x in v]), 0)
    first = v[:a.first]
    ff = np.median(np.array([x[1] for x in first]), 0)
    lf = np.median(np.array([x[2] for x in first]), 0)
    ce = np.median([x[3] for x in first]) * 1000
    # angular spread of the first viewpoints around the object
    dirs = np.array([(x[4] - allf) / np.linalg.norm(x[4] - allf) for x in first])
    ang = np.degrees(np.arccos(np.clip(dirs @ dirs.mean(0) / np.linalg.norm(dirs.mean(0)), -1, 1))).max()
    ea, eb = np.linalg.norm(lf - allf) * 1000, np.linalg.norm(ff - allf) * 1000
    A.append(ea); B.append(eb)
    print(f"{nm:22s} {first[-1][0]:6.1f} {ce:7.1f}  {ea:26.1f}  {eb:27.1f}  {ang:9.1f}°")
print(f"median: (a) live-first vs final-all {np.median(A):.1f} mm   (b) final-first vs final-all {np.median(B):.1f} mm")
# how much of the viewpoint bias remains if the anchor waits for more viewpoints
for n in (5, 10, 20, 40):
    e = [np.linalg.norm(np.median(np.array([x[1] for x in v[:n]]), 0) - np.median(np.array([x[1] for x in v]), 0)) * 1000 for v in by.values() if len(v) >= n + 3]
    print(f"  anchor from first {n:2d} obs (final traj): median gap {np.median(e):.1f} mm over {len(e)} objects")
ku = jl(run / "kf_updates.jsonl")
ev = [((u["t_ns"] - t0) / 1e9, u.get("reason")) for u in ku if u.get("reason") not in ("periodic", "end")]
print("loop / map events:", [(round(t, 1), r) for t, r in ev])
