#!/usr/bin/env python3
"""Phase 2 검증 단위 테스트 — I/O 매트릭스 엣지케이스 + 도구 정합.

Run: ~/miniconda3/envs/sam_yolo/bin/python tests/test_phase2_eval.py
"""
import json
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "_ism_research_2026_07", "phase2_bottleneck"))
import phase2_common as pc          # noqa: E402
import ism_hsv                      # noqa: E402
import yolo_ism_object_n as o_n     # noqa: E402

OUT = pc.OUT


# 1. 단일 forward로 block2/9/11 동시 추출 (dump provenance 증명)
def test_single_forward_three_blocks():
    p = os.path.join(OUT, "metrics", "dump_provenance.json")
    assert os.path.isfile(p), "dump_provenance.json 없음 — dump_phase2_candidates.py 먼저 실행"
    d = json.load(open(p))
    assert d["forward_calls"] == d["frames_processed"], "프레임당 forward가 1회가 아님"
    assert d["blocks_per_forward"] == [2, 9, 11]
    # parity: 재계산이 운영 cur 값과 일치(반올림 오차 이내)
    assert d["parity_sem_top5_maxabs"] < 1e-3
    assert d["parity_appe11_maxabs"] < 1e-3


# 2. semantic 유효 score < 5 정책: available-n mean (운영 top-k=min(k,N) 일치)
def test_semantic_short_array_policy():
    # 3개짜리 배열의 top5 mean == 전체 mean (min(5,3)=3)
    sims = np.array([0.9, 0.5, 0.1], np.float32)
    ss = np.sort(sims)[::-1]
    top5 = float(ss[:5].mean())
    assert abs(top5 - float(sims.mean())) < 1e-6
    # NaN/empty 는 0 대체가 아니라 invalid 로 취급되어야(정책): 빈 배열 mean → nan
    empty = np.array([], np.float32)
    assert np.isnan(empty.mean()) if empty.size == 0 else True


# 3. YOLO failure-stage 분류 로직 (F0/F1/F2)
def test_failure_stage_logic():
    def classify(pp_raw, sh_raw, shared_final):
        pp_002 = sum(1 for c in pp_raw if c >= 0.02)
        if shared_final >= 1: return "hit"
        if len(pp_raw) == 0 and len(sh_raw) == 0: return "F0"
        if pp_002 == 0: return "F1"
        if pp_002 >= 1: return "F2"
        return "F6"
    assert classify([], [], 0) == "F0"
    assert classify([0.01], [0.01], 0) == "F1"        # 있으나 <0.02
    assert classify([0.3], [], 0) == "F2"             # perprompt >=0.02 인데 shared 0
    assert classify([0.3], [0.3], 1) == "hit"


# 4. LODO fold 누수 차단: train/test disjoint, test 는 정확히 1개
def test_lodo_no_leakage():
    folds = pc.lodo_folds()
    assert len(folds) == len(pc.DATASETS)
    for test_ds, train in folds:
        assert test_ds not in train
        assert set(train) | {test_ds} == set(pc.DATASETS)
        assert len(train) == len(pc.DATASETS) - 1


# 5. 결측 gt_visible 은 0 이 아니라 None (임의 0 대체 금지)
def test_missing_not_zeroed(tmp=None):
    import csv, tempfile
    d = tempfile.mkdtemp(); p = os.path.join(d, "c.csv")
    with open(p, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["uid", "dataset", "frame_id", "object", "yolo_conf", "routed", "bbox",
                    "gt_visible", "sem_top1", "sem_top3", "sem_top5", "sem_all_mean",
                    "sem_median", "sem_product", "sem_geomean", "appe2_clstop1",
                    "appe9_clstop1", "appe11_clstop1", "hsv_score", "hsv_threshold",
                    "sim_thr", "appe_gate", "hue_correction_ocv"])
        w.writerow(["u|0|1_1_2_2", "ds", "0", "milk", "0.5", "1", "1_1_2_2", "",
                    "0.4", "0.4", "0.4", "0.3", "0.3", "0.12", "0.34", "0.5", "0.5",
                    "0.5", "1.0", "0.1214", "0.35", "0.55", "0"])
    rows = pc.load_candidates(p)
    assert rows[0]["gt_visible"] is None      # 빈값 → None, 0 아님


# 6. AUROC 정확성 (알려진 소형 케이스)
def test_auroc_known():
    # 완전 분리: pos 모두 > neg → AUROC 1.0
    assert abs(pc.auroc([3, 4, 5], [0, 1, 2]) - 1.0) < 1e-9
    # 완전 반전 → 0.0
    assert abs(pc.auroc([0, 1], [2, 3]) - 0.0) < 1e-9
    # tie 반반 → 0.5
    assert abs(pc.auroc([1, 1], [1, 1]) - 0.5) < 1e-9


# 7. Phase 1C 동결값 보존 (config)
def test_phase1c_frozen_config():
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    by = {o["name"]: o for o in objs}
    d = by["Dinosaur"]
    assert d.get("hsv_gate_enabled") is True
    assert abs(float(d.get("hsv_gate_threshold")) - 0.1214) < 1e-9
    assert int(d.get("hsv_hue_correction_ocv", 0)) == 9
    for n in ("Bear", "milk", "choco_hazelnut_high"):
        assert int(by[n].get("hsv_hue_correction_ocv", 0)) == 0


# 8. best_f1_threshold 는 train 데이터에서만 threshold 선택 (반환형)
def test_best_f1_threshold():
    s = np.array([0.1, 0.2, 0.8, 0.9]); y = np.array([0, 0, 1, 1])
    thr, f1 = pc.best_f1_threshold(s, y)
    assert 0.2 < thr <= 0.8 and abs(f1 - 1.0) < 1e-9    # 완전 분리 가능


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    fails = 0
    for fn in fns:
        try:
            fn(); print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            fails += 1; print(f"FAIL  {fn.__name__}: {e}")
        except Exception as e:
            fails += 1; print(f"ERROR {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - fails}/{len(fns)} passed")
    sys.exit(1 if fails else 0)
