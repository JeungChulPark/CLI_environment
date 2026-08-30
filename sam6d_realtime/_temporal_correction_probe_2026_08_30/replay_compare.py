#!/usr/bin/env python3
"""replay_compare.py — 시간축 포즈 보정: 전 프레임(30fps) vs 실시간(2fps) 비교

동일한 보정기(ObjectAnchorManager: 대칭 인지 클러스터 → 앵커 등록/해제)에
같은 bag 의 SAM-6D 검출 스트림을 두 가지 속도로 흘려 넣고 결과를 비교한다.

  FULL : 모든 프레임 (오프라인 가정, 30fps)
  RT   : 추론 500ms 를 모사 — 직전 처리 프레임 + 0.5s 이후의 첫 프레임만 처리 (~2fps)

평가 기준(pseudo-GT): 물체는 정적이므로 map 좌표계 포즈는 상수여야 한다.
전체 30fps 관측의 대칭 인지 dominant cluster medoid 를 기준 포즈로 삼는다.

입력:
  --detections  output/longcircle2_visualization/detections.jsonl (전 프레임 캡처)
  --trajectory  ORB-SLAM3 CameraTrajectory.txt (SLAM 카메라, TUM)
  --extrinsic   X_camSLAM_camSAM.npy  (T_map_camSAM = T_map_camSLAM · X)

원본 코드는 수정하지 않는다. 이 폴더만 지우면 원상 복구.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

PROBE = Path(__file__).resolve().parent
REPO = PROBE.parent
sys.path.insert(0, str(REPO / "realtime"))
sys.path.insert(0, str(REPO / "tools"))

from evaluate_slam_pose import load_trajectory, nearest_pose, load_jsonl  # noqa: E402
from slam_pose_memory import (ObjectAnchorManager, pose_matrix,           # noqa: E402
                              pose_distance, dominant_pose_cluster, pose_medoid)

SYMMETRY_AXES = {                       # verify_config.py 의 보수적 표와 동일
    "Sikhye_high": [0.0, 0.0, 1.0],
    "Sauce_high": [0.0, 0.0, 1.0],
    "Mugcup_high": [0.1057, 0.0337, 0.9938],
}
REG_ROT_DEG, REG_TRANS_M = 20.0, 0.050   # 앵커 등록 임계와 동일한 판정 기준


def load_measurements(det_path, timestamps, traj, X):
    """검출 행 → (stamp_s, object, T_cam_obj, T_map_cam, T_map_obj) 목록 (시간순)."""
    rows = load_jsonl(det_path)
    out = []
    n_no_slam = 0
    for r in rows:
        if r.get("rejection_reason"):
            continue
        stamp_ns = int(r["stamp_ns"])
        T_slam, dt = nearest_pose(stamp_ns, timestamps, traj)
        if T_slam is None:
            n_no_slam += 1
            continue
        T_map_cam = T_slam @ X
        T_cam_obj = pose_matrix(np.asarray(r["R"], float),
                                np.asarray(r["t_mm"], float) / 1000.0)
        out.append({"stamp_s": stamp_ns / 1e9, "stamp_ns": stamp_ns,
                    "frame_seq": r.get("frame_seq"), "object": r["object"],
                    "T_cam_obj": T_cam_obj, "T_map_cam": T_map_cam,
                    "T_map_obj": T_map_cam @ T_cam_obj})
    out.sort(key=lambda m: (m["stamp_ns"], m["object"]))
    return out, n_no_slam, len(rows)


def build_pseudo_gt(measurements):
    """물체별 map 포즈 dominant cluster medoid + occupancy."""
    gt = {}
    by_obj = defaultdict(list)
    for m in measurements:
        by_obj[m["object"]].append(m["T_map_obj"])
    for name, poses in by_obj.items():
        axis = SYMMETRY_AXES.get(name)
        cluster = dominant_pose_cluster(poses, REG_ROT_DEG, REG_TRANS_M, axis, 10)
        med = pose_medoid(poses, cluster, axis, 10, REG_ROT_DEG, REG_TRANS_M)
        gt[name] = {"pose": poses[med], "n": len(poses),
                    "cluster": len(cluster),
                    "occupancy": len(cluster) / len(poses)}
    return gt


def select_frames(measurements, mode, period_s=0.5):
    """처리할 frame stamp 집합을 고른다. full=전부, rt=직전+period 이후 첫 프레임."""
    stamps = sorted({m["stamp_ns"] for m in measurements})
    if mode == "full":
        return set(stamps)
    sel, last = set(), None
    for s in stamps:
        if last is None or (s - last) >= period_s * 1e9:
            sel.add(s)
            last = s
    return sel


def soft_pose(history, axis, k):
    """등록 전 soft 보정: 최근 k개 map 관측의 dominant-cluster medoid.

    앵커 등록과 같은 임계(20°/50mm)·같은 medoid 함수를 쓰되, 관측이 k개
    미만이어도 있는 것만으로 동작한다. 관측 1개면 그 관측 자체(=원 측정)다.
    """
    poses = list(history)[-k:]
    if not poses:
        return None
    if len(poses) == 1:
        return poses[0]
    cluster = dominant_pose_cluster(poses, REG_ROT_DEG, REG_TRANS_M, axis, 10)
    med = pose_medoid(poses, cluster, axis, 10, REG_ROT_DEG, REG_TRANS_M)
    return poses[med]


def replay(measurements, frame_set, gt, soft_k=None):
    """스트림 하나를 보정기에 흘리고 물체별 출력 타임라인을 만든다.

    soft_k: None 이면 soft 끔(등록 전 원 측정). 정수면 운용 코드의
    soft_window(ObjectAnchorManager.soft_map_pose) 를 그 값으로 켠다.
    """
    anchor = ObjectAnchorManager("longcircle2", {"symmetry_axes": SYMMETRY_AXES,
                                                 "soft_window": int(soft_k or 0)})
    timeline = defaultdict(list)          # obj -> [(stamp_s, err_deg, err_mm, source)]
    events = defaultdict(list)            # obj -> 등록/해제 이벤트
    first_obs = {}
    for m in (m for m in measurements if m["stamp_ns"] in frame_set):
        name = m["object"]
        first_obs.setdefault(name, m["stamp_s"])
        was = name in anchor.anchors
        anchor.observe(name, m["T_map_cam"], m["T_cam_obj"], "TRACKING_OK",
                       m["stamp_ns"], m["stamp_ns"])
        now = name in anchor.anchors
        if now and not was:
            events[name].append(("registered", m["stamp_s"]))
        elif was and not now:
            events[name].append(("released", m["stamp_s"]))
        # 출력 포즈: 앵커 > soft medoid(운용 코드 경로) > 원 측정
        if now:
            T_out_map, source = anchor.anchors[name], "anchor"
        else:
            T_soft = anchor.soft_map_pose(name) if soft_k else None
            if T_soft is None:                     # soft 끔 / 관측<2 → 원 측정
                T_out_map, source = m["T_map_obj"], "raw"
            else:
                T_out_map, source = T_soft, "soft"
        err_deg, err_m = pose_distance(gt[name]["pose"], T_out_map,
                                       SYMMETRY_AXES.get(name), 10)
        timeline[name].append((m["stamp_s"], err_deg, err_mm := err_m * 1000.0, source))
    return timeline, events, first_obs


def summarize(timeline, events, first_obs, t_end, max_hold_s=1.0):
    """표시 시간 가중 오차 통계.

    출력은 다음 출력까지 유지하되 max_hold_s 에서 만료시킨다 — 검출이 끊긴
    구간(화면 밖)에 마지막 포즈를 무한정 '표시 중'으로 계산하는 왜곡을 막는다.
    """
    out = {}
    for name, tl in sorted(timeline.items()):
        hold_bad = hold_total = 0.0
        werr_deg = werr_mm = 0.0
        n_src = defaultdict(int)
        for i, (t, err_deg, err_mm, source) in enumerate(tl):
            dt = (tl[i + 1][0] if i + 1 < len(tl) else t_end) - t
            dt = min(max(dt, 0.0), max_hold_s)
            hold_total += dt
            werr_deg += err_deg * dt
            werr_mm += err_mm * dt
            if err_deg > REG_ROT_DEG or err_mm > REG_TRANS_M * 1000.0:
                hold_bad += dt
            n_src[source] += 1
        reg = [t for e, t in events.get(name, []) if e == "registered"]
        out[name] = {
            "outputs": len(tl),
            "anchor_outputs": n_src["anchor"],
            "soft_outputs": n_src["soft"],
            "raw_outputs": n_src["raw"],
            "time_to_first_anchor_s": (round(reg[0] - first_obs[name], 2)
                                       if reg else None),
            "n_registered": len(reg),
            "n_released": sum(e == "released" for e, _ in events.get(name, [])),
            "displayed_s": round(hold_total, 2),
            "coverage_pct": round(100.0 * hold_total /
                                  max(t_end - first_obs[name], 1e-9), 1),
            "bad_display_s": round(hold_bad, 2),
            "bad_display_pct": round(100.0 * hold_bad / hold_total, 1) if hold_total else None,
            "time_weighted_err_deg": round(werr_deg / hold_total, 2) if hold_total else None,
            "time_weighted_err_mm": round(werr_mm / hold_total, 1) if hold_total else None,
        }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detections",
                    default=str(REPO / "output/longcircle2_visualization/detections.jsonl"))
    ap.add_argument("--trajectory",
                    default="/home/jucpark/DeepLearning/CLI_environment/output_slam/"
                            "260804_office/orb3_slam/longcircle2/CameraTrajectory.txt")
    ap.add_argument("--extrinsic",
                    default="/home/jucpark/DeepLearning/CLI_environment/integration/"
                            "data/260804_office/longcircle2/calib/X_camSLAM_camSAM.npy")
    ap.add_argument("--rt-period-s", type=float, default=0.5)
    ap.add_argument("--out", default=str(PROBE / "outputs" / "compare.json"))
    a = ap.parse_args()

    timestamps, traj = load_trajectory(a.trajectory)
    X = np.asarray(np.load(a.extrinsic), float).reshape(4, 4)
    ms, n_no_slam, n_rows = load_measurements(a.detections, timestamps, traj, X)
    print(f"[replay] 검출 행 {n_rows}개 → 유효 측정 {len(ms)}개 "
          f"(SLAM 포즈 미매칭 {n_no_slam}개 제외)")
    if not ms:
        raise SystemExit("no measurements")
    t_end = max(m["stamp_s"] for m in ms)

    gt = build_pseudo_gt(ms)
    print("\n[pseudo-GT] 물체별 map 포즈 클러스터 (전체 30fps 관측 기준):")
    for name, g in sorted(gt.items()):
        print(f"  {name:22s} 관측 {g['n']:5d}  클러스터 {g['cluster']:5d} "
              f"(occupancy {g['occupancy']*100:.1f}%)")

    result = {"detections": a.detections, "n_rows": n_rows, "n_valid": len(ms),
              "rt_period_s": a.rt_period_s,
              "pseudo_gt": {k: {"n": v["n"], "cluster": v["cluster"],
                                "occupancy": round(v["occupancy"], 4)}
                            for k, v in gt.items()},
              "streams": {}}
    STREAMS = [("full", "full", None), ("rt", "rt", None),
               ("rt_soft3", "rt", 3), ("rt_soft5", "rt", 5), ("rt_soft7", "rt", 7),
               ("full_soft5", "full", 5)]
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    for label, mode, soft_k in STREAMS:
        frame_set = select_frames(ms, mode, a.rt_period_s)
        tl, ev, first = replay(ms, frame_set, gt, soft_k=soft_k)
        summary = summarize(tl, ev, first, t_end)
        result["streams"][label] = {"n_frames": len(frame_set), "soft_k": soft_k,
                                    "objects": summary}
        print(f"\n[{label}] 처리 프레임 {len(frame_set)}개"
              + (f" (soft K={soft_k})" if soft_k else ""))
        for name, s in summary.items():
            print(f"  {name:22s} 출력 {s['outputs']:5d} "
                  f"(anchor {s['anchor_outputs']:4d} soft {s['soft_outputs']:4d}) "
                  f"등록 {s['time_to_first_anchor_s'] if s['time_to_first_anchor_s'] is not None else '-':>7}s "
                  f"해제 {s['n_released']}회  "
                  f"커버리지 {s['coverage_pct']:>5}%  "
                  f"나쁜표시 {s['bad_display_pct']:>5}%  "
                  f"오차 {s['time_weighted_err_deg']:>6}° / {s['time_weighted_err_mm']:>7}mm")
        raw = {k: [(round(t, 4), round(d, 3), round(mm, 2), s)
                   for t, d, mm, s in v] for k, v in tl.items()}
        json.dump(raw, open(out.parent / f"timeline_{label}.json", "w"))

    json.dump(result, open(out, "w"), indent=1, ensure_ascii=False)
    print(f"\n결과 저장: {out}")


if __name__ == "__main__":
    main()
