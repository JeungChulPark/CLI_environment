#!/usr/bin/env python3
"""
Keyframe-anchored camera poses from a live_orbslam or live_rtabmap run, for attaching other estimates
(e.g. SAM-6D object poses T_cam_obj) to the SLAM map so they follow its loop closures and optimisation.

Files (written by mac_harness/live_orbslam or live_rtabmap into the run directory):
  FrameKeyframeRef.csv       frame -> reference keyframe id, T_kf_cam, live T_w_cam
  KeyFramePosesFinal.csv     keyframe id -> final T_w_kf
  KeyFramePoseSnapshots.csv  keyframe id -> T_w_kf as known at a frame (once a second)
  MapEvents.csv              frames where the map was corrected or restarted

Three camera poses for frame i:
  live     T_w_cam the tracker returned at frame i (what a robot had at that moment)
  final    T_w_kf(final) * T_kf_cam, identical to CameraTrajectory.txt
  asof(j)  T_w_kf(latest snapshot taken by frame j) * T_kf_cam: frame i as re-anchored with the map
           known at frame j, the online update rule for an object seen at frame i

    keyframe_anchor.py RUN_DIR            check the files against CameraTrajectory.txt and summarise

In code:
    run = AnchoredRun("output/mac_live_orb_...")
    T_w_cam = run.final(i)                 # 4x4 nested lists
    T_w_obj = mul(run.final(i), T_cam_obj)
    run.near_event(i, seconds=1.0)         # frames near any map event
    run.near_correction(i, 1.0, 0.02)      # frames near a correction that moved keyframes by >= 2 cm
Works for live_orbslam and live_rtabmap runs alike (RTAB-Map's keyframes are its graph nodes). RTAB-Map
corrects its graph almost every second once loops close, so near_correction is the useful flag there.
Poses are camera x right / y down / z forward; world = the SLAM's first camera / keyframe.
"""

import bisect
import csv
import math
import sys
from pathlib import Path


# ---------------------------------------------------------------- SE3 as 4x4 nested lists, no numpy
def from_tq(tx, ty, tz, qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), tx],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), ty],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), tz],
            [0.0, 0.0, 0.0, 1.0]]


def mul(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def translation(T):
    return (T[0][3], T[1][3], T[2][3])


def _se3(row, prefix):
    keys = [f"{prefix}_{k}" for k in ("tx", "ty", "tz", "qx", "qy", "qz", "qw")]
    if any(row[k] == "" for k in keys):
        return None
    return from_tq(*(float(row[k]) for k in keys))


class AnchoredRun:
    def __init__(self, run_dir):
        d = Path(run_dir)
        self.frames = {}  # frame -> dict(stamp, state, lost, kf, T_kf_cam, T_w_cam_live)
        with open(d / "FrameKeyframeRef.csv") as f:
            for r in csv.DictReader(f):
                self.frames[int(r["frame"])] = {
                    "stamp": float(r["stamp"]), "state": int(r["state"]), "lost": r["lost"] == "1",
                    "kf": int(r["kf_id"]) if r["kf_id"] else None,
                    "T_kf_cam": _se3(r, "kf_cam"), "T_w_cam_live": _se3(r, "live_w_cam")}
        self.kf_final = {}
        with open(d / "KeyFramePosesFinal.csv") as f:
            for r in csv.DictReader(f):
                self.kf_final[int(r["kf_id"])] = _se3(r, "w_kf")
        # snapshots: ordered list of (frame, {kf: T_w_kf})
        snaps = {}
        with open(d / "KeyFramePoseSnapshots.csv") as f:
            for r in csv.DictReader(f):
                s = snaps.setdefault(int(r["snapshot"]), (int(r["frame"]), {}))
                s[1][int(r["kf_id"])] = _se3(r, "w_kf")
        self.snapshots = sorted(snaps.values(), key=lambda s: s[0])
        self._snap_frames = [s[0] for s in self.snapshots]
        self.events = []
        with open(d / "MapEvents.csv") as f:
            self.events = [(int(r["frame"]), float(r["stamp"]), r["event"]) for r in csv.DictReader(f)]
        # how far each snapshot moved the keyframes it shares with the previous one: (frame, stamp, max shift m)
        self.shifts = []
        stamp_of = {fr: v["stamp"] for fr, v in self.frames.items()}
        for (_, prev), (frame, cur) in zip(self.snapshots, self.snapshots[1:]):
            common = [k for k in cur if k in prev and cur[k] and prev[k]]
            shift = max((math.dist(translation(cur[k]), translation(prev[k])) for k in common), default=0.0)
            self.shifts.append((frame, stamp_of.get(frame, 0.0), shift))

    def usable(self, i):
        fr = self.frames.get(i)
        return fr is not None and not fr["lost"] and fr["kf"] is not None and fr["T_kf_cam"] is not None

    def live(self, i):
        return self.frames[i]["T_w_cam_live"]

    def final(self, i):
        fr = self.frames[i]
        T_w_kf = self.kf_final.get(fr["kf"])
        return mul(T_w_kf, fr["T_kf_cam"]) if T_w_kf and self.usable(i) else None

    def asof(self, i, j):
        """Frame i re-anchored with the keyframe poses of the latest snapshot taken at or before frame j."""
        k = bisect.bisect_right(self._snap_frames, j) - 1
        fr = self.frames[i]
        if k < 0 or not self.usable(i):
            return None
        T_w_kf = self.snapshots[k][1].get(fr["kf"])
        return mul(T_w_kf, fr["T_kf_cam"]) if T_w_kf else None

    def near_event(self, i, seconds=1.0):
        t = self.frames[i]["stamp"]
        return any(abs(t - s) <= seconds for _, s, e in self.events)

    def near_correction(self, i, seconds=1.0, min_shift=0.02):
        """True near a snapshot where the map moved some keyframe by at least min_shift metres."""
        t = self.frames[i]["stamp"]
        return any(abs(t - s) <= seconds for _, s, m in self.shifts if m >= min_shift)


def main():
    run = AnchoredRun(sys.argv[1])
    d = Path(sys.argv[1])
    saved = {}
    for line in (d / "CameraTrajectory.txt").read_text().splitlines():
        v = line.split()
        if len(v) == 8:
            saved[round(float(v[0]), 6)] = tuple(map(float, v[1:4]))

    # 1) final re-anchoring must reproduce CameraTrajectory.txt
    err, n = 0.0, 0
    for i, fr in run.frames.items():
        T = run.final(i)
        s = saved.get(round(fr["stamp"], 6))
        if T and s:
            err = max(err, math.dist(translation(T), s))
            n += 1
    print(f"final = T_w_kf(final) * T_kf_cam vs CameraTrajectory.txt: {n} frames, max difference {err * 1000:.3f} mm")

    # 2) how far the live pose sits from the final one, and how much re-anchoring at the end recovers
    gaps = sorted(math.dist(translation(run.live(i)), translation(run.final(i)))
                  for i in run.frames if run.usable(i) and run.live(i) and run.final(i))
    if gaps:
        print(f"live vs final: median {gaps[len(gaps) // 2] * 100:.1f} cm, p95 {gaps[int(.95 * (len(gaps) - 1))] * 100:.1f} cm, "
              f"max {gaps[-1] * 100:.1f} cm")
    last = max(run.frames)
    at_end = sorted(math.dist(translation(run.asof(i, last)), translation(run.final(i)))
                    for i in run.frames if run.usable(i) and run.asof(i, last) and run.final(i))
    if at_end:
        print(f"re-anchored with the last snapshot vs final: median {at_end[len(at_end) // 2] * 100:.2f} cm, "
              f"max {at_end[-1] * 100:.2f} cm")

    print(f"keyframes {len(run.kf_final)}, snapshots {len(run.snapshots)}, map events {len(run.events)}:")
    for f_, s, e in run.events[:8]:
        print(f"  frame {f_} ({s - 1000:.2f} s) {e}")
    if len(run.events) > 8:
        print(f"  ... {len(run.events) - 8} more")
    flagged = sum(1 for i in run.frames if run.near_event(i))
    print(f"frames within 1 s of a map event: {flagged} of {len(run.frames)}")
    big = [(f_, s, m) for f_, s, m in run.shifts if m >= 0.02]
    print(f"snapshots that moved a keyframe by >= 2 cm: {len(big)}"
          + "".join(f"\n  frame {f_} ({s - 1000:.2f} s) max shift {m * 100:.1f} cm" for f_, s, m in big[:8]))
    flagged = sum(1 for i in run.frames if run.near_correction(i))
    print(f"frames within 1 s of such a correction: {flagged} of {len(run.frames)}")


if __name__ == "__main__":
    main()
