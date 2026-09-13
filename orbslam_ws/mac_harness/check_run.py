#!/usr/bin/env python3
"""Checks on one harness run: stamp monotonicity, map resets, closure error, path length."""
import math
import re
import sys
from pathlib import Path

d = Path(sys.argv[1])
rows = [l.split(",") for l in (d / "TrackPerFrame.csv").read_text().splitlines()[1:]]
stamps = [float(r[1]) for r in rows]
back = sum(1 for a, b in zip(stamps, stamps[1:]) if b <= a)
states = {}
for r in rows:
    states[r[3]] = states.get(r[3], 0) + 1
log = (d / "run.log").read_text(errors="replace")
maps = len(re.findall(r"Creation of new map with id", log))
older = len(re.findall(r"timestamp older than previous", log))
lba = len(re.findall(r"Loop detected|Merge detected|Loop closed", log, re.I))

P = []
for l in (d / "CameraTrajectory.txt").read_text().splitlines():
    v = l.split()
    if len(v) == 8:
        P.append(tuple(map(float, v[1:4])))
path = sum(math.dist(a, b) for a, b in zip(P, P[1:]))
print(f"frames {len(rows)}  non-monotonic stamps {back}  states {states}")
print(f"maps created {maps}  'older timestamp' errors {older}  loop/merge log lines {lba}")
print(f"trajectory poses {len(P)}  closure error {math.dist(P[0], P[-1])*100:.1f} cm  path {path:.2f} m")
