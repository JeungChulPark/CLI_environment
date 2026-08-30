#!/usr/bin/env python3
"""Phase 2.1 FN root-cause 단위 테스트.
Run: ~/miniconda3/envs/sam_yolo/bin/python tests/test_phase21_rootcause.py
"""
import csv
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "_ism_research_2026_07", "phase21_fn_audit"))
import phase21_common as pc   # noqa: E402


# 1. IoU 계산
def test_iou():
    assert abs(pc.iou((0, 0, 10, 10), (0, 0, 10, 10)) - 1.0) < 1e-9
    assert pc.iou((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0
    # half overlap: (0,0,10,10) & (5,0,15,10) inter=50 union=150 -> 1/3
    assert abs(pc.iou((0, 0, 10, 10), (5, 0, 15, 10)) - (50 / 150)) < 1e-9


# 2. GT coverage
def test_gt_coverage():
    # pred fully covers gt -> 1.0 ; small gt inside big pred
    assert abs(pc.gt_coverage((0, 0, 100, 100), (10, 10, 20, 20)) - 1.0) < 1e-9
    # half of gt covered
    assert abs(pc.gt_coverage((0, 0, 10, 10), (5, 0, 15, 10)) - 0.5) < 1e-9


# 3. proposal purity
def test_purity():
    # big proposal, small gt -> low purity
    assert abs(pc.proposal_purity((0, 0, 100, 100), (0, 0, 10, 10)) - (100 / 10000)) < 1e-9
    assert abs(pc.proposal_purity((0, 0, 10, 10), (0, 0, 10, 10)) - 1.0) < 1e-9


# 4. one-to-one greedy matching (동일 class 다중 instance)
def test_greedy_one_to_one():
    gts = [(0, 0, 10, 10), (100, 100, 110, 110)]
    preds = [((0, 0, 10, 10), 0.9), ((100, 100, 110, 110), 0.8), ((0, 0, 9, 9), 0.95)]
    m = pc.greedy_match(preds, gts, thr=0.5)
    # 각 GT는 최대 1회 매칭 (one-to-one)
    gis = [gi for _, gi, _ in m]
    assert len(gis) == len(set(gis)), "GT 중복 매칭 발생"
    assert len(m) == 2


# 5. 매칭 threshold 미달이면 매칭 없음
def test_greedy_no_match_below_thr():
    m = pc.greedy_match([((0, 0, 10, 10), 0.9)], [(50, 50, 60, 60)], thr=0.5)
    assert m == []


# 6. 결정트리 우선순위: no candidate -> RC1
def test_decision_no_candidate():
    prim, sec = pc.fn_primary_cause({"best": None, "cands": [], "accept": False})
    assert prim == "RC1_no_candidate"


# 7. semantic 우선 (sem 실패면 appe/hsv 무관하게 RC8)
def test_decision_semantic_priority():
    b = {"passS": False, "passA": False, "passH": False, "accept": False}
    cell = {"best": b, "cands": [b], "accept": False}
    prim, sec = pc.fn_primary_cause(cell)
    assert prim == "RC8_semantic"
    assert "appearance" in sec and "hsv" in sec   # secondary 기록


# 8. appe 단독 실패 -> RC9
def test_decision_appearance():
    b = {"passS": True, "passA": False, "passH": True, "accept": False}
    cell = {"best": b, "cands": [b], "accept": False}
    assert pc.fn_primary_cause(cell)[0] == "RC9_appearance"


# 9. hsv 단독 실패 -> RC10
def test_decision_hsv():
    b = {"passS": True, "passA": True, "passH": False, "accept": False}
    cell = {"best": b, "cands": [b], "accept": False}
    assert pc.fn_primary_cause(cell)[0] == "RC10_hsv"


# 10. selection: best 실패하나 다른 후보 전 gate 통과 -> RC11
def test_decision_selection():
    best = {"passS": False, "passA": True, "passH": True, "accept": False}
    other = {"passS": True, "passA": True, "passH": True, "accept": True}
    cell = {"best": best, "cands": [best, other], "accept": False}
    assert pc.fn_primary_cause(cell)[0] == "RC11_selection"


# 11. 일관성: FN root-cause 합 == 313 (산출물 존재 시)
def test_consistency_sum_313():
    p = os.path.join(pc.OUT, "csv", "fn_root_cause_summary.csv")
    if not os.path.isfile(p):
        print("  (skip: summary CSV 없음 — build_fn_object_index 먼저 실행)")
        return
    tot = sum(int(r["count"]) for r in csv.DictReader(open(p)))
    assert tot == 313, f"FN 합 {tot} != 313"


# 12. 배타성: per-object CSV 각 행 primary 1개
def test_exclusive_primary():
    p = os.path.join(pc.OUT, "csv", "fn_root_cause_per_object.csv")
    if not os.path.isfile(p):
        print("  (skip: per-object CSV 없음)")
        return
    rows = list(csv.DictReader(open(p)))
    assert len(rows) == 313
    for r in rows:
        assert r["primary_root_cause"] and ";" not in r["primary_root_cause"]


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
