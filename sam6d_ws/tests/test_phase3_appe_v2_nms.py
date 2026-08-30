"""Phase 3: appearance-v2 (block 2+9 + matched gate) and cross-object NMS.

Contract: BOTH flags default OFF and OFF must be a byte-identical no-op, so the
frozen Phase 1C operating point survives untouched.
"""
import os, sys
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import yolo_ism_object_n as m


@pytest.fixture(scope="module")
def objs():
    _, o = m.load_config(m.DEFAULT_CONFIG)
    return o


def test_deployed_default_is_on(objs):
    """Promoted 2026-07-23 on location-verified evidence (TP +34 / FP -13)."""
    for o in objs:
        assert o.get("appe_v2_enabled") is True
        assert o.get("cross_object_nms_enabled") is True


def test_deployed_blocks_and_gate_move_as_a_pair(objs):
    for o in objs:
        assert m._blocks_of(o) == [2, 9]
        assert m._appe_gate_of(o) == pytest.approx(0.605)


def test_off_switch_restores_phase1c_exactly(objs):
    """Rollback path: flags false => the frozen Phase 1C operating point, bit-identical."""
    for o in objs:
        off = dict(o, appe_v2_enabled=False)
        assert m._blocks_of(off) == [11]
        assert m._appe_gate_of(off) == pytest.approx(0.55)


def test_on_swaps_blocks_and_gate_together(objs):
    o = dict(objs[0], appe_v2_enabled=True)
    assert m._blocks_of(o) == [2, 9]
    assert m._appe_gate_of(o) == pytest.approx(0.605)


def test_gate_is_never_the_bare_0_55_with_v2_blocks(objs):
    """The 2026-07-15 regression was [2,9] scored against the 0.55 gate."""
    o = dict(objs[0], appe_v2_enabled=True)
    assert m._appe_gate_of(o) != pytest.approx(float(o["appe_gate"]))


def test_v2_blocks_configurable(objs):
    o = dict(objs[0], appe_v2_enabled=True, appe_v2_blocks=9)
    assert m._blocks_of(o) == [9]


def _res(box, rank, acc=True):
    return {"accepted": acc, "box": box, "rank_appe": rank, "masked_appe": rank}


def test_nms_off_is_noop():
    o = {"name": "a", "cross_object_nms_enabled": False}
    r = {"a": _res([0, 0, 10, 10], 0.6), "b": _res([0, 0, 10, 10], 0.9)}
    m.apply_cross_object_nms([o, dict(o, name="b")], r)
    assert r["a"]["accepted"] and r["b"]["accepted"]


def test_nms_keeps_highest_rank_appe():
    o = {"name": "a", "cross_object_nms_enabled": True, "cross_object_nms_iou": 0.9}
    r = {"a": _res([0, 0, 10, 10], 0.60), "b": _res([0, 0, 10, 10], 0.90)}
    m.apply_cross_object_nms([o, dict(o, name="b")], r)
    assert r["b"]["accepted"] is True
    assert r["a"]["accepted"] is False
    assert r["a"]["decision"] == "no-object(cross-object-nms)"
    assert r["a"]["cross_object_nms_loser_to"] == "b"


def test_nms_spares_distinct_boxes():
    o = {"name": "a", "cross_object_nms_enabled": True}
    r = {"a": _res([0, 0, 10, 10], 0.6), "b": _res([100, 100, 110, 110], 0.9)}
    m.apply_cross_object_nms([o, dict(o, name="b")], r)
    assert r["a"]["accepted"] and r["b"]["accepted"]


def test_nms_iou_threshold_respected():
    """Boxes overlapping below the IoU threshold are two objects, not one."""
    o = {"name": "a", "cross_object_nms_enabled": True, "cross_object_nms_iou": 0.9}
    r = {"a": _res([0, 0, 10, 10], 0.6), "b": _res([5, 0, 15, 10], 0.9)}   # IoU = 1/3
    m.apply_cross_object_nms([o, dict(o, name="b")], r)
    assert r["a"]["accepted"] and r["b"]["accepted"]


def test_nms_ignores_rejected_and_boxless():
    o = {"name": "a", "cross_object_nms_enabled": True}
    r = {"a": _res([0, 0, 10, 10], 0.9, acc=False), "b": _res(None, 0.8),
         "c": _res([0, 0, 10, 10], 0.5)}
    m.apply_cross_object_nms([o, dict(o, name="b"), dict(o, name="c")], r)
    assert r["c"]["accepted"] is True          # survives: 'a' was already rejected


def test_nms_falls_back_to_masked_appe_when_rank_unset():
    o = {"name": "a", "cross_object_nms_enabled": True}
    r = {"a": {"accepted": True, "box": [0, 0, 10, 10], "rank_appe": 0.0, "masked_appe": 0.9},
         "b": {"accepted": True, "box": [0, 0, 10, 10], "rank_appe": 0.0, "masked_appe": 0.4}}
    m.apply_cross_object_nms([o, dict(o, name="b")], r)
    assert r["a"]["accepted"] and not r["b"]["accepted"]


def test_nms_three_way_keeps_exactly_one():
    o = {"name": "a", "cross_object_nms_enabled": True}
    names = ["a", "b", "c"]
    r = {n: _res([0, 0, 10, 10], v) for n, v in zip(names, (0.5, 0.9, 0.7))}
    m.apply_cross_object_nms([dict(o, name=n) for n in names], r)
    assert sum(r[n]["accepted"] for n in names) == 1
    assert r["b"]["accepted"]


def test_new_keys_are_overridable(objs):
    for k in ("appe_v2_enabled", "appe_v2_blocks", "appe_v2_gate",
              "cross_object_nms_enabled", "cross_object_nms_iou"):
        assert k in m._OVERRIDABLE
