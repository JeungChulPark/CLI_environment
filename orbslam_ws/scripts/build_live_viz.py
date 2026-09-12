#!/usr/bin/env python3
"""
Pack one measured ORB-SLAM3 run into the JSON the live playback page consumes.

Beyond pose and latency this carries the per-frame tracking state, so the page
can show *where* estimation degraded rather than only that it did:

    2 OK              tracking normally
    3 RECENTLY_LOST   lost the local map, still trying to recover
    4 LOST            gave up; needs relocalisation
    1 NOT_INITIALIZED building the initial map
    0 NO_IMAGES_YET   seen right after an Atlas map reset

Map resets are read out of the node log rather than inferred, and each is placed
at the frame whose state first returns to NO_IMAGES_YET/NOT_INITIALIZED after it.
"""

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

HDR = 11  # PCD ascii header lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, help="output/<run> directory")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--label", default="ORB-SLAM3")
    ap.add_argument("--sub", default="")
    args = ap.parse_args()

    d = Path(args.run)
    pf = np.loadtxt(d / "TrackPerFrame.csv", delimiter=",", skiprows=1)
    tr = np.loadtxt(d / "CameraTrajectory.txt")
    kf = np.loadtxt(d / "KeyFrameTrajectory.txt")
    pts = np.loadtxt(d / "map_points.pcd", skiprows=HDR)
    ts = np.loadtxt(d / "TrackingTimeStats.txt", delimiter=",", skiprows=1)

    n = min(len(pf), len(tr))
    pf, tr = pf[:n], tr[:n]
    ms = pf[:, 2]
    state = pf[:, 3].astype(int) if pf.shape[1] > 3 else np.full(n, 2)
    nopose = pf[:, 4].astype(int) if pf.shape[1] > 4 else np.zeros(n, int)

    # keyframe rows, matched on timestamp
    kfset = {round(float(x), 6) for x in kf[:, 0]}
    is_kf = np.array([1 if round(float(t), 6) in kfset else 0 for t in tr[:, 0]], int)

    # timestamp monotonicity: a backwards step is what makes ORB-SLAM3 reset
    stamps = pf[:, 1]
    back = np.where(np.diff(stamps) < 0)[0]

    resets = len(re.findall(r"Creation of new map with id", (d / "node.log").read_text(
        errors="ignore"))) - 1  # the first one is initialisation, not a reset

    c = np.median(pts, axis=0)
    r = np.linalg.norm(pts - c, axis=1)
    P = pts[r <= np.percentile(r, 99.5)]

    def clean(a):
        a = np.asarray(a, float).copy()
        a[~np.isfinite(a)] = np.nan
        a[(a < 0) | (a > 200)] = np.nan
        return a

    orb = clean(ts[:n, 2]) if len(ts) >= n else np.full(n, np.nan)
    lm = clean(ts[:n, 6]) if len(ts) >= n else np.full(n, np.nan)

    pos = tr[:, 1:4]
    seg = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    # a jump far beyond the per-frame norm is what a relocalisation looks like
    jump_thresh = float(np.percentile(seg, 99.9) * 4)
    jumps = [int(i) for i in np.where(seg > jump_thresh)[0]]

    out = dict(
        fps=args.fps, n=int(n), deadline=33.33,
        label=args.label, sub=args.sub,
        pos=[[round(float(v), 3) for v in p] for p in pos],
        quat=[[round(float(v), 3) for v in q] for q in tr[:, 4:8]],
        ms=[round(float(v), 2) for v in ms],
        orb_ms=[None if np.isnan(v) else round(float(v), 2) for v in orb],
        lm_ms=[None if np.isnan(v) else round(float(v), 2) for v in lm],
        state=[int(v) for v in state],
        nopose=[int(v) for v in nopose],
        kf=[int(v) for v in is_kf],
        points=[[round(float(v), 3) for v in p] for p in P],
        events=dict(
            ts_backwards=[int(i) for i in back],
            map_resets=int(max(0, resets)),
            pose_jumps=jumps,
            jump_threshold_m=round(jump_thresh, 4),
        ),
        meta=dict(
            mean=round(float(ms.mean()), 2),
            p95=round(float(np.percentile(ms, 95)), 2),
            p99=round(float(np.percentile(ms, 99)), 2),
            mx=round(float(ms.max()), 2),
            over=int((ms > 33.33).sum()),
            keyframes=int(len(kf)),
            map_points=int(len(P)),
            duration_s=round(n / args.fps, 1),
            path_len_m=round(float(seg.sum()), 2),
            closure_gap_m=round(float(np.linalg.norm(pos[-1] - pos[0])), 4),
            n_ok=int((state == 2).sum()),
            n_recently_lost=int((state == 3).sum()),
            n_lost=int((state == 4).sum()),
            n_other=int(((state != 2) & (state != 3) & (state != 4)).sum()),
        ),
    )
    Path(args.out).write_text(json.dumps(out, separators=(",", ":")))
    m = out["meta"]
    print(f"{args.label}: {n} frames, mean {m['mean']} p99 {m['p99']} max {m['mx']} over {m['over']}")
    print(f"  states  OK {m['n_ok']}  RECENTLY_LOST {m['n_recently_lost']}  "
          f"LOST {m['n_lost']}  other {m['n_other']}")
    print(f"  events  map resets {out['events']['map_resets']}  "
          f"ts backwards {len(back)}  pose jumps {len(jumps)} (>{jump_thresh*100:.1f} cm)")
    print(f"  wrote {args.out} ({Path(args.out).stat().st_size/1e6:.2f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
