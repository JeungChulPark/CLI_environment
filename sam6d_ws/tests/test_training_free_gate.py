#!/usr/bin/env python3
"""Phase 2.2 training-free gate 단위 테스트.
Run: ~/miniconda3/envs/sam_yolo/bin/python tests/test_training_free_gate.py
"""
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
import ism_training_free_gate as tfg      # noqa: E402
import yolo_ism_object_n as o_n           # noqa: E402


# 1. OFF/phase1c => 완전 no-op (Phase 1C 보존)
def test_off_is_noop():
    for method in (None, "", "phase1c"):
        ch, r = tfg.decide(method, {"accepted": False, "decision": "no-object(below-appe)"}, {})
        assert ch is False


# 2. driver hook: disabled => res 불변
def test_hook_disabled_noop():
    res = {"accepted": False, "decision": "no-object(below-appe)"}
    before = dict(res)
    o_n._apply_tf_gate({"training_free_gate_enabled": False}, res)
    assert res == before      # tf_gate_reason 도 안 붙음


# 3. 이미 accept면 무개입
def test_already_accepted():
    ch, r = tfg.decide("one_borderline_rescue", {"accepted": True}, {})
    assert ch is False and r == "already_accepted"


# 4. one_borderline_rescue: appe만 borderline, sem·hsv strong => rescue
def test_rescue_appe_borderline():
    # appe_gate 0.55, borderline=0.9*0.55=0.495 ; strong sem>=0.35+0.1*0.65=0.415, hsv>=0.1214+0.1*0.8786=0.209
    ok, reason = tfg.one_borderline_rescue(
        sem=0.90, appe=0.52, hsv=0.80, sim_thr=0.35, appe_gate=0.55, hsv_thr=0.1214,
        sem_ok=True, appe_ok=False, hsv_ok=True)
    assert ok is True and reason == "rescue_borderline_A"


# 5. appe severe-fail(<0.5*gate) => rescue 금지
def test_no_rescue_severe():
    ok, reason = tfg.one_borderline_rescue(
        sem=0.9, appe=0.20, hsv=0.8, sim_thr=0.35, appe_gate=0.55, hsv_thr=0.1214,
        sem_ok=True, appe_ok=False, hsv_ok=True)
    assert ok is False and "severe" in reason


# 6. HSV severe mismatch => 절대 rescue 금지 (FP 폭증 방지)
def test_hsv_hard_veto():
    ok, reason = tfg.one_borderline_rescue(
        sem=0.9, appe=0.9, hsv=0.02, sim_thr=0.35, appe_gate=0.55, hsv_thr=0.1214,
        sem_ok=True, appe_ok=True, hsv_ok=False)
    assert ok is False and "hsv_severe" in reason


# 7. 2개 gate 실패 => rescue 금지 (one-borderline only)
def test_no_rescue_two_fail():
    ok, reason = tfg.one_borderline_rescue(
        sem=0.30, appe=0.50, hsv=0.8, sim_thr=0.35, appe_gate=0.55, hsv_thr=0.1214,
        sem_ok=False, appe_ok=False, hsv_ok=True)
    assert ok is False and "fails=2" in reason


# 8. borderline이지만 나머지가 strong 아니면 rescue 금지
def test_no_rescue_others_weak():
    ok, reason = tfg.one_borderline_rescue(
        sem=0.36, appe=0.52, hsv=0.13, sim_thr=0.35, appe_gate=0.55, hsv_thr=0.1214,
        sem_ok=True, appe_ok=False, hsv_ok=True)   # sem/hsv 겨우 통과(strong 아님)
    assert ok is False and "others_weak" in reason


# 9. decide 경유 hook: sem-stage 미도달 후보는 unreachable
def test_decide_unreachable_sem():
    o = {"similarity_threshold": 0.35, "appe_gate": 0.55, "hsv_gate_threshold": 0.1214}
    res = {"accepted": False, "decision": "no-object(below-sim)", "best_sem": 0.30}
    ch, r = tfg.decide("one_borderline_rescue", res, o)
    assert ch is False and "unreachable" in r


# 10. feature 결측(masked_appe None) => fail-safe no rescue
def test_missing_scores_failsafe():
    o = {"similarity_threshold": 0.35, "appe_gate": 0.55, "hsv_gate_threshold": 0.1214}
    res = {"accepted": False, "decision": "no-object(below-appe)", "best_sem": 0.9, "masked_appe": None}
    ch, r = tfg.decide("one_borderline_rescue", res, o)
    assert ch is False


# 11. config 기본값이 OFF 인지 (Phase 1C 보존 계약)
def test_config_default_off():
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    for o in objs:
        assert bool(o.get("training_free_gate_enabled", False)) is False
        assert o.get("training_free_gate_method", "phase1c") == "phase1c"


# 12. unknown method => no-op
def test_unknown_method():
    ch, r = tfg.decide("some_ml_head", {"accepted": False}, {})
    assert ch is False and "unknown" in r


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
