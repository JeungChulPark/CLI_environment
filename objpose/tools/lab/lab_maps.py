#!/usr/bin/env python3
"""lab_maps.py — lab_rt.py 의 실행 결과(세션별 sam6d_frames.jsonl)로 Object Pose Map 을 만든다. 지도 설정은 ObChange · YCB 실험과 같다(proposed_maps.py 의 함수를 그대로 쓴다).
  <run>/maps/<세션>.json : 등록 객체 [{name, center, R, n_obs, status}]  (좌표계 = 그 세션 ORB-SLAM3 의 월드)
    BENCH_ROOT=~/DeepLearning/Datasets/lab_sessions_bop BENCH_TABLE=objpose/rt/bop/lab8_objects.json BENCH_FPS=10 <sam6d python> objpose/tools/lab/lab_maps.py --run <dir> --poses <slam dir>
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "obchange"))
import proposed_maps as PM   # noqa: E402
BM = PM.BM
ap = argparse.ArgumentParser(); ap.add_argument("--run", type=Path, required=True); ap.add_argument("--poses", type=Path, required=True); a = ap.parse_args()
table = json.loads(BM.TABLE.read_text())["objects"]; ident = {o["name"]: o["name"] for o in table}; names = list(ident); (a.run / "maps").mkdir(exist_ok=True)
for d in sorted(p for p in a.run.iterdir() if (p / "sam6d_frames.jsonl").exists()):
    s = d.name; K = np.array(next(iter(json.loads((BM.YCBV / s / "scene_camera.json").read_text()).values()))["cam_K"]).reshape(3, 3)
    mem, poses = PM.new_memory(names, K); PM.feed(mem, poses, a.run, a.poses, s, 0); objs = PM.snapshot(mem, ident)
    (a.run / "maps" / f"{s}.json").write_text(json.dumps(objs)); print(s, [(o["name"], o["n_obs"]) for o in objs], flush=True)
