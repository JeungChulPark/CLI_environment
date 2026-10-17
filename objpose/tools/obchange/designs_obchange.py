#!/usr/bin/env python3
"""designs_obchange.py — ObChange 실행(six_rt.py + proposed_maps.py)의 기록으로 객체 지도를 지금 방식과 설계 조합 11가지로 만든다(objpose/tools/lab/map_designs.py 를 그대로 쓴다).

  실제 환경 실험(직접 수집한 영상)과 다른 점은 입력뿐이다: 카메라 자세 = 데이터셋 제공값(scene_camera.json), 깊이 = BOP 폴더의 깊이 영상, 지금 방식의 지도 = <run>/maps/proposed/<방문>.json.
  <out>/c<구성>_<방문>.json : {"V0": …, "1": …, "1n": …, "1+2": …, …} 객체마다 name(ycbv_ 이름), T_w_obj, 깊이 지지율
    <sam6d python> objpose/tools/obchange/designs_obchange.py --root objpose/output/six_obchange_261011 [--rooms smallroom] [-f]
"""
import argparse, json, sys
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lab"))
_argv = sys.argv; sys.argv = [_argv[0]]
import map_designs as MD      # noqa: E402
sys.argv = _argv
REPO = MD.REPO; TAB = json.loads((REPO / "objpose/rt/bop/obchange_objects.json").read_text()); BOP = Path(TAB["scenes_dir"]); BY_YCB = {o["ycb"]: o["name"] for o in TAB["objects"]}
T0_NS = 1_600_000_000_000_000_000; STEP_NS = 100_000_000


class ObScene(MD.Scene):
    def __init__(self, run: Path, scene: str):
        self.run = run; d = run / scene; self.cam = json.loads((BOP / scene / "scene_camera.json").read_text()); self.keys = sorted(self.cam, key=int); self.bop = BOP / scene
        self.K = np.array(self.cam[self.keys[0]]["cam_K"], float).reshape(3, 3); self.ses = SimpleNamespace(W=640, H=480); self._depth = {}; self.model = {}; self.tau = 0
        self.frames = MD.jl(d / "sam6d/frames.jsonl"); self.dets = MD.jl(d / "sam6d/detections.jsonl"); self.Tws = {}
        for f in self.frames:
            c = self.cam[self.keys[(int(f["stamp_ns"]) - T0_NS) // STEP_NS]]; T = np.eye(4); T[:3, :3] = np.array(c["cam_R_w2c"]).reshape(3, 3); T[:3, 3] = np.array(c["cam_t_w2c"]) / 1000.0; self.Tws[int(f["stamp_ns"])] = np.linalg.inv(T)
        self.lm0 = []
        for o in json.loads((run / "maps/proposed" / f"{scene}.json").read_text()):
            T = np.eye(4); T[:3, :3] = np.asarray(o["R"], float); T[:3, 3] = np.asarray(o["center"], float); self.lm0.append({"name": BY_YCB[o["name"]], "T_w_obj": T.tolist(), "n_obs": o["n_obs"], "status": o["status"]})
        self.summary = {}

    def depth(self, stamp):
        if stamp not in self._depth:
            k = self.keys[(int(stamp) - T0_NS) // STEP_NS]; raw = cv2.imread(str(self.bop / "depth" / f"{int(k):06d}.png"), cv2.IMREAD_UNCHANGED)
            self._depth[stamp] = None if raw is None else raw.astype(np.float32) * float(self.cam[k]["depth_scale"]) / 1000.0
            if len(self._depth) > 400: self._depth.pop(next(iter(self._depth)))
        return self._depth[stamp]


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--root", type=Path, required=True); ap.add_argument("--rooms", nargs="*", default=["smallroom", "office", "livingarea", "kitchen"]); ap.add_argument("-f", action="store_true"); a = ap.parse_args()
    out = a.root / "designs"; out.mkdir(exist_ok=True)
    for c in range(1, 9):
        for r in a.rooms:
            for n in range(2, 7):
                s = f"{r}_s{n}"; f = out / f"c{c}_{s}.json"
                if not (a.root / f"c{c}/maps/proposed/{s}.json").exists() or (f.exists() and not a.f): continue
                R = MD.run(ObScene((a.root / f"c{c}").resolve(), s)); R["scene"] = s; R["config"] = c; f.write_text(json.dumps(R, ensure_ascii=False))
                print(f"c{c} {s}", {v: len(R[v]) for v in ("V0", "1n", "1n+2", "1+2+3")}, R["counts"], flush=True)
