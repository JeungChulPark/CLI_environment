#!/usr/bin/env python3
"""make_gt_scans.py — FM-Fusion 입력 폴더(scans/<장면>/color, depth, pose)를 정답 카메라 자세로 다시 만든다. 영상은 기존 폴더(SLAM 자세로 만든 것)의 color · depth 를 링크하고 pose 만 새로 쓴다.
  "같은 인식 결과, 다른 지도 생성" 실험에서 모든 지도 생성 방식이 같은 카메라 자세(scene_camera.json 의 cam_R_w2c)를 쓰게 하려는 것이다.
    python3 objpose/tools/mapgen/make_gt_scans.py --table objpose/rt/bop/ycbv_objects.json --src ~/DeepLearning/Datasets/fmfusion_ycbv_260928/scans --out ~/DeepLearning/Datasets/fmfusion_ycbv_gtpose/scans
"""
import argparse, json, os
from pathlib import Path
import numpy as np
ap = argparse.ArgumentParser(); ap.add_argument("--table", type=Path, required=True); ap.add_argument("--src", type=Path, required=True); ap.add_argument("--out", type=Path, required=True); a = ap.parse_args()
root = Path(json.loads(a.table.read_text())["scenes_dir"])
for sc in sorted(p.name for p in a.src.expanduser().iterdir() if p.is_dir()):
    cam = json.loads((root / sc / "scene_camera.json").read_text()); ids = sorted(cam, key=int); d = a.out.expanduser() / sc; (d / "pose").mkdir(parents=True, exist_ok=True)
    for sub in ("color", "depth"):
        if not (d / sub).exists(): os.symlink((a.src.expanduser() / sc / sub).resolve(), d / sub)
    for i, k in enumerate(ids):
        R = np.array(cam[k]["cam_R_w2c"]).reshape(3, 3); T = np.eye(4); T[:3, :3] = R.T; T[:3, 3] = -R.T @ (np.array(cam[k]["cam_t_w2c"]) / 1000.0); np.savetxt(d / "pose" / f"frame-{i:06d}.txt", T, fmt="%.9f")
    print(sc, len(ids), "frames", len(list((d / "color").iterdir())), "color", flush=True)
