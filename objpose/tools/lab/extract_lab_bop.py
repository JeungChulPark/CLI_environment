#!/usr/bin/env python3
"""extract_lab_bop.py — 직접 녹화한 세션(data_slam_converted/<세션>)의 카메라 영상을 BOP 형식 폴더로 꺼낸다. ObChange 실험과 같은 도구들이 그대로 읽게 하기 위한 것.

  카메라    인식용 카메라 녹화(SAM)가 있으면 그것, 없으면 SLAM용 카메라 녹화. 한 세션에 카메라 하나만 쓴다(SLAM 도 같은 영상으로 돌린다).
  프레임    녹화 시각 기준 --fps (기본 10)로 고르게 뽑는다(각 시각에 가장 가까운 프레임).
  깊이      0.1 mm 단위 16비트(depth_scale 0.1, ObChange 폴더와 같음). 6.5 m 넘는 값은 0.
  <out>/test/<세션>/rgb/000001.png …, depth/…, scene_camera.json (cam_K, depth_scale, 녹화 시각), scene_gt.json (비어 있음: 정답 없음)
    <sam6d python> objpose/tools/lab/extract_lab_bop.py [--sessions …] [--out ~/DeepLearning/Datasets/lab_sessions_bop]
"""
import argparse, json, sys
from pathlib import Path
import cv2
import numpy as np
REPO = Path(__file__).resolve().parents[3]; sys.path.insert(0, str(REPO / "objpose/pc"))
from conv_session import ConvSession   # noqa: E402
SRC = Path.home() / "DeepLearning/Datasets/data_slam_converted"
ALL = ["260901_cbnu_bigeightcircle", "260901_cbnu_digut", "260901_cbnu_eightcircle", "260901_cbnu_longcircle", "260826_etri_differentObject", "260826_etri_eightcircle_dark", "260826_etri_longcircle_dark", "260826_etri_similarObject"]
ap = argparse.ArgumentParser(); ap.add_argument("--sessions", nargs="*", default=ALL); ap.add_argument("--out", type=Path, default=Path.home() / "DeepLearning/Datasets/lab_sessions_bop"); ap.add_argument("--fps", type=float, default=10.0)
a = ap.parse_args()
for s in a.sessions:
    sub = "SAM" if (SRC / s / "SAM").is_dir() else "SLAM"; S = ConvSession(SRC / s / sub); d = a.out / "test" / s
    if (d / "scene_camera.json").exists(): print(s, "이미 있음"); continue
    (d / "rgb").mkdir(parents=True, exist_ok=True); (d / "depth").mkdir(exist_ok=True)
    t = (S.t_ns - S.t_ns[0]) / 1e9; ticks = np.arange(0, t[-1], 1.0 / a.fps); idx = np.clip(np.searchsorted(t, ticks), 0, len(t) - 1); idx = np.where(np.abs(t[np.maximum(idx - 1, 0)] - ticks) < np.abs(t[idx] - ticks), np.maximum(idx - 1, 0), idx)
    cam = {}; gt = {}; K = S.K.reshape(-1).tolist()
    for n, i in enumerate(idx, 1):
        f = S.read(int(i), with_depth=True); dep = f.depth_raw.astype(np.uint32) * 10; dep[dep > 65000] = 0
        cv2.imwrite(str(d / "rgb" / f"{n:06d}.png"), f.color_bgr); cv2.imwrite(str(d / "depth" / f"{n:06d}.png"), dep.astype(np.uint16))
        cam[str(n)] = {"cam_K": K, "depth_scale": 0.1, "stamp": float(S.t_ns[i]) / 1e9, "src_index": int(i)}; gt[str(n)] = []
    (d / "scene_gt.json").write_text(json.dumps(gt)); (d / "scene_camera.json").write_text(json.dumps(cam))
    (d / "info.json").write_text(json.dumps({"session": s, "camera": sub, "fps": a.fps, "frames": len(idx), "source_frames": len(S), "source_fps": round(len(S) / t[-1], 2), "duration_s": round(float(t[-1]), 1), "distortion": S.kc}))
    print(s, sub, len(idx), "frames", f"{t[-1]:.0f}s", flush=True)
