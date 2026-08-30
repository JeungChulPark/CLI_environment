#!/usr/bin/env python3
"""운용 코드에 넣은 soft 보정 검증 (GPU/모델 불필요, 합성 포즈).

1) ObjectAnchorManager.soft_map_pose: flip 소수파를 다수결이 누르는지
2) Sam6DCore._apply_anchors: collecting 행이 soft_medoid 로 대체되는지
3) soft_window=0 이면 예전 동작과 동일한지 / 등록 후엔 앵커가 우선인지
"""
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "realtime"))

from slam_pose_memory import ObjectAnchorManager, pose_matrix  # noqa: E402

R_ID = np.eye(3)
R_FLIP = np.diag([1.0, -1.0, -1.0])          # x축 180° flip
T_GOOD = pose_matrix(R_ID, [1.0, 0.0, 2.0])
T_FLIP = pose_matrix(R_FLIP, [1.0, 0.0, 2.0])
TWC = np.eye(4)                               # 카메라 = 맵 (단순화)
STAMP = 1_000_000_000


def observe_seq(mgr, seq):
    for i, T in enumerate(seq):
        mgr.observe("obj", TWC, T, "TRACKING_OK", STAMP + i, STAMP + i)


def test_soft_majority_beats_flip():
    mgr = ObjectAnchorManager("m", {"soft_window": 5})
    observe_seq(mgr, [T_GOOD, T_GOOD, T_FLIP])   # 마지막이 flip
    soft = mgr.soft_map_pose("obj")
    assert soft is not None
    assert np.allclose(soft[:3, :3], R_ID, atol=1e-9), "다수파(정상)가 이겨야 한다"
    print("PASS 1: 다수결이 flip 을 누른다")


def test_soft_disabled_and_min_obs():
    mgr = ObjectAnchorManager("m", {"soft_window": 0})
    observe_seq(mgr, [T_GOOD, T_GOOD, T_FLIP])
    assert mgr.soft_map_pose("obj") is None, "soft_window=0 이면 꺼져야 한다"
    mgr2 = ObjectAnchorManager("m", {"soft_window": 5})
    observe_seq(mgr2, [T_GOOD])
    assert mgr2.soft_map_pose("obj") is None, "관측 1개면 None(원 측정 유지)"
    print("PASS 2: soft_window=0 비활성 / 관측<2 는 None")


def test_anchor_takes_over():
    mgr = ObjectAnchorManager("m", {"soft_window": 5, "window": 4,
                                    "register_count": 4, "release_window": 3,
                                    "release_count": 3})
    observe_seq(mgr, [T_GOOD] * 4)               # window=4 채워 등록
    assert "obj" in mgr.anchors, "앵커가 등록돼야 한다"
    assert mgr.soft_map_pose("obj") is None, "등록 후 soft 는 물러나야 한다"
    print("PASS 3: 앵커 등록 후엔 앵커 우선")


def test_apply_anchors_integration():
    from sam6d_core import Sam6DCore
    core = object.__new__(Sam6DCore)             # 모델 로드 없이 메서드만 검증
    core.anchor_manager = ObjectAnchorManager("m", {"soft_window": 5})
    core.last_frame_diag = {}
    core._pts = {}
    slam_context = {"map_id": "m", "T_map_camera": TWC.tolist(),
                    "tracking_state": "TRACKING_OK",
                    "pose_stamp_ns": STAMP, "rgb_stamp_ns": STAMP}

    def frame(T):
        row = {"object": "obj", "score": 1.0,
               "R": [[float(v) for v in r] for r in T[:3, :3]],
               "t_mm": [float(v) * 1000.0 for v in T[:3, 3]],
               "bbox": [0, 0, 10, 10], "pose_source": "sam6d",
               "rejection_reason": None}
        shadow = {"object": "obj", "R": T[:3, :3], "t_mm": T[:3, 3] * 1000.0,
                  "score": 1.0, "verify": {"accepted": True}, "ism": {}}
        return [row], [shadow]

    # 프레임 1~2: 정상 → 프레임 3: flip 측정
    for T in (T_GOOD, T_GOOD):
        rows, shadows = frame(T)
        out = core._apply_anchors(rows, shadows, [], np.zeros((4, 4)), np.eye(3),
                                  (4, 4), slam_context)
    rows, shadows = frame(T_FLIP)
    out = core._apply_anchors(rows, shadows, [], np.zeros((4, 4)), np.eye(3),
                              (4, 4), slam_context)
    row = out[0]
    assert row["pose_source"] == "soft_medoid", f"pose_source={row['pose_source']}"
    assert row["anchor_state"] == "soft"
    assert np.allclose(np.asarray(row["R"]), R_ID, atol=1e-6), \
        "flip 측정이 soft medoid(정상)로 대체돼야 한다"
    assert np.allclose(np.asarray(row["raw_R"]), R_FLIP, atol=1e-6), \
        "원 측정은 raw_R 로 보존돼야 한다"
    assert core.last_frame_diag["anchor"]["soft_objects"] == ["obj"]
    print("PASS 4: _apply_anchors 가 flip 행을 soft_medoid 로 대체 (raw 보존)")

    # soft_window=0 이면 원 측정 그대로 (예전 동작)
    core.anchor_manager = ObjectAnchorManager("m", {"soft_window": 0})
    for T in (T_GOOD, T_GOOD):
        rows, shadows = frame(T)
        core._apply_anchors(rows, shadows, [], np.zeros((4, 4)), np.eye(3),
                            (4, 4), slam_context)
    rows, shadows = frame(T_FLIP)
    out = core._apply_anchors(rows, shadows, [], np.zeros((4, 4)), np.eye(3),
                              (4, 4), slam_context)
    assert out[0]["pose_source"] == "sam6d" and "raw_R" not in out[0]
    print("PASS 5: soft_window=0 은 예전 동작 그대로")


if __name__ == "__main__":
    test_soft_majority_beats_flip()
    test_soft_disabled_and_min_obs()
    test_anchor_takes_over()
    test_apply_anchors_integration()
    print("\n모든 테스트 통과")
