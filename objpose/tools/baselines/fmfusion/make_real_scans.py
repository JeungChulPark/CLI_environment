#!/usr/bin/env python3
"""make_real_scans.py — 직접 녹화한 세션의 실시간 실행 결과(허브 run 폴더)를 FM-Fusion 입력 형식으로 바꾼다 (make_ycbv_scans.py 의 실제 주행 판).

  <out>/scans/<name>/color/frame-XXXXXX.png   인식 카메라 컬러
                    /depth/frame-XXXXXX.png   인식 카메라 깊이 (mm → config depth_scale 1000)
                    /pose/frame-XXXXXX.txt    인식 카메라→월드 4x4 = 최종 SLAM 자세(키프레임 보정 반영) × 두 카메라 사이 변환
                    /meta.json                K, 크기, 프레임 시각
                    /landmarks.json           Object Pose Map 의 최종 물체(같은 run 기록을 허브 기본 설정으로 다시 넣은 결과, 같은 월드)
  XXXXXX = 세션 프레임 번호. --gap 마다 한 장만 쓴다(FM-Fusion 은 frame_gap 의 배수 번호만 읽는다).
  자세가 없는 프레임(SLAM 추적 실패·시작 전)은 pose 를 쓰지 않는다 → FM-Fusion 이 건너뛴다.

    /home/jucpark/anaconda3/envs/sam6d/bin/python objpose/tools/baselines/fmfusion/make_real_scans.py <run> --x <X json> \
        --session <SAM session> --name 260901_cbnu_longcircle --out ~/DeepLearning/Datasets/fmfusion_real
"""
from __future__ import annotations

import argparse, json, sys
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "objpose/pc"))
sys.path.insert(0, str(REPO / "objpose/tools/obj_anchor"))
from fusion import Fusion, PoseBuffer                              # noqa: E402
from memory import ObjectMemory                                    # noqa: E402
from core.depth_check import DepthCheckConfig                      # noqa: E402
from conv_session import ConvSession                               # noqa: E402
from eval_real import jl, DEPTH_DS                                 # noqa: E402
from fit_extrinsic import final_poses                              # noqa: E402

KEEP = ("active", "lost", "remembered")


def replay_memory(run: Path, X, ses: ConvSession):
    """허브와 같은 순서(자세·키프레임 갱신·SAM-6D 결과 도착)로 run 기록을 Object Pose Map 에 다시 넣는다 (eval_reassoc_real.py 와 같음)."""
    age = {int(e["t_ns"]): float(e["result_age_s"]) for e in jl(run / "sam6d_estimates.jsonl") if e.get("result_age_s") is not None}
    med = float(np.median(list(age.values()))) if age else 0.5
    ev = [(int(m["t_ns"]), 0, "pose", m) for m in jl(run / "slam_poses.jsonl")] + [(int(u["t_ns"]), 1, "kf", u) for u in jl(run / "kf_updates.jsonl")]
    ev += [(int(f["t_ns"]) + int(age.get(int(f["t_ns"]), med) * 1e9), 2, "sam", f) for f in jl(run / "sam6d_frames.jsonl") if f["dets"]]
    ev.sort(key=lambda e: (e[0], e[1]))
    pb = PoseBuffer(max_gap_s=0.25)
    mem = ObjectMemory(Fusion(X, pb), ses.K, (ses.W, ses.H), assoc_trans_gate_m=0.30, assoc_rot_gate_deg=None,
                       depth_check=DepthCheckConfig(free_margin_m=0.15, confirm_frames=3, max_speed_mps=0.6, max_rot_dps=45.0))
    for t, _, kind, m in ev:
        if kind == "pose":
            if m.get("T_wc") is not None and m.get("state") in ("OK", "OK_KLT"):
                pb.add(t, m.get("state"), m.get("T_wc"), m.get("map_id"), kf=m.get("ref_kf"), T_w_kf=m.get("T_w_kf"),
                       kf_map_id=m.get("kf_map_id"), map_changed=bool(m.get("map_changed")))
        elif kind == "kf":
            pb.apply_kf_update(m.get("map_id"), m.get("kfs", []))
        else:
            s = int(m["t_ns"])
            i = int(np.argmin(np.abs(ses.t_ns - s)))
            depth = None
            if abs(int(ses.t_ns[i]) - s) <= 5_000_000:
                depth = np.ascontiguousarray(ses.read(i, True).depth_raw[::DEPTH_DS, ::DEPTH_DS]).astype(np.uint16)
            mem.add_frame(s, [(x["object"], x.get("score", 0.0), x["R"], x["t_mm"]) for x in m["dets"]], depth=depth)
            mem.update()
    return [m for m in mem.landmarks() if m["status"] in KEEP]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path)
    ap.add_argument("--x", type=Path, required=True)
    ap.add_argument("--session", type=Path, required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--gap", type=int, default=5)
    a = ap.parse_args()
    X = np.asarray(json.loads(a.x.read_text())["T_slam_sam"], float).reshape(4, 4)
    ses = ConvSession(a.session, offset_ns=0, cache_dir=REPO / "objpose/output/.clock_cache")
    dst = a.out / "scans" / a.name
    for d in ("color", "depth", "pose"):
        (dst / d).mkdir(parents=True, exist_ok=True)

    lms = replay_memory(a.run, X, ses)
    (dst / "landmarks.json").write_text(json.dumps(
        [{"name": m["name"], "T_w_obj": np.asarray(m["T_w_obj"], float).tolist(), "n_obs": int(m["n_obs"]), "status": m["status"]} for m in lms], indent=1))
    print("Object Pose Map:", ", ".join(f"{m['name']}({m['n_obs']})" for m in lms), flush=True)

    pb = final_poses(a.run)
    n_pose, frames = 0, []
    for i in range(0, len(ses.t_ns), a.gap):
        f = f"frame-{i:06d}"
        fr = ses.read(i, True)
        cv2.imwrite(str(dst / "color" / f"{f}.png"), fr.color_bgr)
        cv2.imwrite(str(dst / "depth" / f"{f}.png"), np.asarray(fr.depth_raw, np.uint16))
        T, _ = pb.at(int(ses.t_ns[i]))
        if T is not None:
            np.savetxt(dst / "pose" / f"{f}.txt", T @ X, fmt="%.9f"); n_pose += 1
        frames.append((i, int(ses.t_ns[i]), T is not None))
    (dst / "meta.json").write_text(json.dumps({"run": str(a.run), "session": str(a.session), "K": np.asarray(ses.K, float).tolist(),
                                               "size": [ses.W, ses.H], "gap": a.gap, "frames": frames}))
    print(f"{a.name}: {len(frames)} frames written, {n_pose} with pose → {dst}", flush=True)


if __name__ == "__main__":
    main()
