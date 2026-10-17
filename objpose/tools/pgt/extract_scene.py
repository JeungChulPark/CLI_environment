#!/usr/bin/env python3
"""extract_scene.py — converted SAM-camera bag -> two BOP-style scenes for the estimator comparison.

  build/000001   frames i % 3 == 1   detections here make the map-based pseudo ground truth
  eval/000001    frames i % 3 == 0   the three estimators are scored here (disjoint from build)
Frame ids are the bag's colour frame index; times.json keeps each id's data-clock stamp (ns).
"""
import json, sys
from pathlib import Path
import cv2, numpy as np
REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/pc"))
from conv_session import ConvSession

sess_dir, out = Path(sys.argv[1]), Path(sys.argv[2])
s = ConvSession(sess_dir, cache_dir=REPO / "objpose/output/.clock_cache")
K = s.K
print("frames", len(s), "K", K.ravel().tolist(), "WxH", s.W, s.H, flush=True)
for name, r in (("build", 1), ("eval", 0)):
    d = out / name / "000001"
    (d / "rgb").mkdir(parents=True, exist_ok=True); (d / "depth").mkdir(parents=True, exist_ok=True)
    cam, times = {}, {}
    for i in range(len(s)):
        if i % 3 != r:
            continue
        f = s.read(i, True)
        cv2.imwrite(str(d / "rgb" / f"{i:06d}.png"), f.color_bgr)
        cv2.imwrite(str(d / "depth" / f"{i:06d}.png"), f.depth_raw.astype(np.uint16))
        cam[str(i)] = {"cam_K": K.ravel().tolist(), "depth_scale": 1.0}
        times[str(i)] = int(f.t_ns)
    (d / "scene_camera.json").write_text(json.dumps(cam))
    (d / "times.json").write_text(json.dumps(times))
    print(name, len(cam), flush=True)
