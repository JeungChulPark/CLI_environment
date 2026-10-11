"""Shared frame list / loader / stats for the same-GPU timing comparison."""
from __future__ import annotations

import statistics
import sys
from pathlib import Path

import numpy as np

ROOT = Path("/home/jucpark/DeepLearning/CLI_environment")
DATASET = Path("/home/jucpark/DeepLearning/Dataset/260915_eightcircle/SAM")
OUT = Path(__file__).resolve().parent

# the eight objects of the indoor runs (paper Sec. experiments); Sauce_high is enabled in
# the deployed config but is not one of the eight, so both pipelines use exactly these.
OBJECTS = ["milk", "choco_hazelnut_high", "Febreze_high", "Mugcup_high", "saffron",
           "Sikhye_high", "Bear", "Dinosaur"]

FRAMES = sorted(set(int(round(x)) for x in np.linspace(300, 3500, 30)) | {1808})
WARMUP = [FRAMES[0], 1808, FRAMES[-1]]


def load_frames():
    sys.path.insert(0, str(ROOT / "objpose/pc"))
    from conv_session import ConvSession
    sys.path.remove(str(ROOT / "objpose/pc"))
    sess = ConvSession(DATASET, offset_ns=2_000_000)
    out = {}
    for i in sorted(set(FRAMES) | set(WARMUP)):
        fr = sess.read(i, with_depth=True)
        out[i] = (fr.color_bgr.copy(), fr.depth_raw.copy(), int(fr.t_ns))
    return out, np.asarray(sess.K, dtype=np.float64)


def stats(xs):
    xs = [float(x) for x in xs]
    return {"median": round(statistics.median(xs), 1), "p90": round(float(np.percentile(xs, 90)), 1),
            "mean": round(statistics.fmean(xs), 1), "min": round(min(xs), 1), "max": round(max(xs), 1),
            "n": len(xs)}
