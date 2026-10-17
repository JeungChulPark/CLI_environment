#!/usr/bin/env python3
"""proposed_maps.py — ObChange 에서 제안 방법(Object Pose Map)의 객체 지도를 만든다.

  방문별 지도     <out>/maps/<tag>/<scene>.json            방문 하나만 넣은 Object Pose Map 의 등록 객체
  이어 넣기(A→B)  <out>/maps/<tag>_online/<A>__<B>.json    한 지도에 방문 A 를 넣고 이어서 방문 B 를 넣는다.
                                                           A 가 끝난 순간과 B 가 끝난 순간의 등록 객체(같은 object_id 유지)
지도 설정은 YCB-V 실험과 같다(build_memory_maps.run_memory): 연관 0.30 m, 깊이 검사(0.15 m, 3프레임).
카메라 자세는 --poses 폴더의 <scene>_frames.txt (기본: 데이터셋 tf). 세계 좌표 = 그 자세의 세계(tf 면 map).

    BENCH_ROOT=<ObChange>/bop BENCH_TABLE=objpose/rt/bop/obchange_objects.json BENCH_FPS=10 \
      <sam6d python> objpose/tools/obchange/proposed_maps.py --run <out>/proposed --out <out> --tag proposed
"""
from __future__ import annotations
import argparse, itertools, json, os, sys
from pathlib import Path
import cv2, numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
import build_memory_maps as BM          # noqa: E402  (BENCH_* 환경변수를 읽는다)
from fusion import Fusion, PoseBuffer    # noqa: E402
from memory import ObjectMemory, DepthCheckConfig, map_occlusion_kwargs  # noqa: E402

KEEP = ("active", "lost", "remembered")


def new_memory(names, K):
    poses = PoseBuffer(max_gap_s=0.25)
    mem = ObjectMemory(Fusion(np.eye(4), poses), K, (640, 480),
                       depth_check=DepthCheckConfig(free_margin_m=0.15, confirm_frames=3, max_speed_mps=0.6, max_rot_dps=45.0),
                       assoc_trans_gate_m=0.30, assoc_rot_gate_deg=None, **map_occlusion_kwargs(names))
    return mem, poses


def feed(mem, poses, run: Path, pose_dir: Path, scene: str, off_ns: int, T_fix=None):
    sc = BM.YCBV / scene
    n = len(json.loads((sc / "scene_camera.json").read_text()))
    a = np.loadtxt(pose_dir / f"{scene}_frames.txt", ndmin=2)
    for r in a:
        i = int(round(r[0] * BM.FPS))
        T = np.eye(4); T[:3, :3] = BM.quat_to_rot(r[4:8]); T[:3, 3] = r[1:4]
        if T_fix is not None:
            T = T_fix @ T
        if 0 <= i < n:
            poses.add(BM.T0_NS + off_ns + i * BM.STEP_NS, "OK", T, map_id=0)
    frames = [json.loads(l) for l in (run / scene / "sam6d_frames.jsonl").read_text().splitlines() if l.strip()]
    frames.sort(key=lambda f: f.get("done_s", f["idx"]))
    for f in frames:
        raw = cv2.imread(str(sc / "depth" / f"{f['idx'] + 1:06d}.png"), cv2.IMREAD_UNCHANGED)
        depth = np.rint(raw.astype(np.float32) * 0.1).astype(np.uint16)[::8, ::8]
        mem.add_frame(int(f["t_ns"]) + off_ns, [(d["object"], d["score"], d["R"], d["t_mm"]) for d in f["dets"]], depth=depth)
        mem.update()
    return n


def snapshot(mem, ycb_of):
    out = []
    for m in mem.landmarks():
        if m["status"] not in KEEP:
            continue
        T = np.asarray(m["T_w_obj"], float)
        out.append({"id": int(m["object_id"]), "name": ycb_of[m["name"]], "center": T[:3, 3].tolist(), "R": T[:3, :3].tolist(),
                    "n_obs": m["n_obs"], "status": m["status"]})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--tag", default="proposed"); ap.add_argument("--poses", type=Path, default=None)
    ap.add_argument("--room", default="kitchen"); ap.add_argument("--no-online", action="store_true", help="방문별 지도만 만든다(이어 넣기 생략)")
    a = ap.parse_args()
    pose_dir = a.poses or (BM.YCBV.parent / "poses_tf")
    table = json.loads(BM.TABLE.read_text())["objects"]
    ycb_of = {o["name"]: o["ycb"] for o in table}; names = list(ycb_of)
    scenes = sorted(p.name for p in a.run.iterdir() if p.is_dir() and p.name.startswith(a.room + "_s"))
    K = np.array(next(iter(json.loads((BM.YCBV / scenes[0] / "scene_camera.json").read_text()).values()))["cam_K"]).reshape(3, 3)
    d1 = a.out / "maps" / a.tag; d2 = a.out / "maps" / f"{a.tag}_online"
    d1.mkdir(parents=True, exist_ok=True); d2.mkdir(parents=True, exist_ok=True)
    for s in scenes:
        mem, poses = new_memory(names, K)
        feed(mem, poses, a.run, pose_dir, s, 0)
        objs = snapshot(mem, ycb_of)
        (d1 / f"{s}.json").write_text(json.dumps(objs, indent=1))
        print(s, [(o["name"][4:], o["n_obs"]) for o in objs], mem.stats().get("by_status"), flush=True)
    for A, B in ([] if a.no_online else itertools.permutations(scenes, 2)):
        mem, poses = new_memory(names, K)
        n = feed(mem, poses, a.run, pose_dir, A, 0)
        snapA = snapshot(mem, ycb_of)
        feed(mem, poses, a.run, pose_dir, B, (n + 100) * BM.STEP_NS)
        snapB = snapshot(mem, ycb_of)
        (d2 / f"{A}__{B}.json").write_text(json.dumps({"A": snapA, "B": snapB}, indent=1))
    print("online pairs", len(list(d2.glob("*.json"))))


if __name__ == "__main__":
    main()
