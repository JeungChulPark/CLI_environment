#!/usr/bin/env python3
"""spread_parts.py — where does the object-position spread of a real run come from?

With the final SLAM trajectory and the (fitted) rig extrinsic, every estimate of an object is split into
  * short-term noise: distance to the same object's estimate in the previous result within 0.5 s (SLAM and rig error
    are nearly constant over 0.5 s, so this is the SAM-6D frame-to-frame jitter; /sqrt(2) = per-estimate noise)
  * along-ray part of the deviation from the object's median position: + = farther from the camera than the median,
    - = closer. A pose estimator that sees only the front surface puts the centre too close -> negative, and since the
    ray direction changes with the viewpoint, this looks like a viewpoint-dependent error in the map.
  * across-ray part
    python spread_parts.py <run> --x <X json> [--tau-s]
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from fit_extrinsic import Run, world                               # noqa: E402

ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--x", required=True); ap.add_argument("--tau-s", type=float, default=0.0)
a = ap.parse_args()
xj = json.loads(Path(a.x).read_text()); X = np.asarray(xj["T_slam_sam"], float).reshape(4, 4)
r = Run(Path(a.run), a.tau_s)
W = world(r, X, np.zeros(6))
cam = r.R @ X[:3, 3] + r.t                 # SAM camera centre in the map
names = sorted(set())
jit, along, across, dist = [], [], [], []
for c in range(r.ncls):
    m = np.where(r.ci == c)[0]
    if len(m) < 5: continue
    med = np.median(W[m], 0)
    for k, i in enumerate(m):
        ray = W[i] - cam[i]; rn = np.linalg.norm(ray); u = ray / rn
        dv = W[i] - med
        along.append(dv @ u); across.append(np.linalg.norm(dv - (dv @ u) * u)); dist.append(rn)
        if k and (r.ts[i] - r.ts[m[k - 1]]) < 0.5e9:
            jit.append(np.linalg.norm(W[i] - W[m[k - 1]]))
along, across, jit, dist = np.array(along) * 1000, np.array(across) * 1000, np.array(jit) * 1000, np.array(dist)
tot = np.sqrt(along ** 2 + across ** 2)
print(f"{r.name}: n {len(tot)}  total dev median {np.median(tot):.1f} p90 {np.percentile(tot, 90):.1f} mm")
print(f"  short-term jitter (pairs <0.5 s): median {np.median(jit):.1f} p90 {np.percentile(jit, 90):.1f} mm  (n {len(jit)})")
print(f"  along ray : mean {along.mean():+.1f}  median |.| {np.median(np.abs(along)):.1f} p90 {np.percentile(np.abs(along), 90):.1f} mm")
print(f"  across ray: median {np.median(across):.1f} p90 {np.percentile(across, 90):.1f} mm")
print(f"  share of squared deviation along the ray: {np.sum(along ** 2) / np.sum(tot ** 2) * 100:.0f}%  (random 3D = 33%)")
k = np.polyfit(dist, along, 1); print(f"  along-ray vs distance: slope {k[0]:+.1f} mm/m, at 1 m {np.polyval(k, 1.0):+.1f} mm")
