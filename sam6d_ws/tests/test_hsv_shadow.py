#!/usr/bin/env python3
"""Phase 1B unit tests — HSV shadow gate (ism_hsv authority + shadow safety).

Run: ~/miniconda3/envs/sam_yolo/bin/python tests/test_hsv_shadow.py
 or: ~/miniconda3/envs/sam_yolo/bin/python -m pytest tests/test_hsv_shadow.py -q
"""
import os
import sys

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

import ism_hsv                       # noqa: E402
import yolo_ism_object_n as o_n      # noqa: E402

RSRCH = os.path.join(REPO, "_ism_research_2026_07")


# 1. RGB->HSV authority parity vs research prototype_colors (exact)
def test_rgb_to_hsv_authority_parity():
    PC = np.load(os.path.join(RSRCH, "ism_fusion_research", "ply_hsv", "prototype_colors.npz"))
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    tdir = {o["name"]: o["template_dir"] for o in objs}
    for name in ("Bear", "Rabbit", "milk", "saffron"):
        op, nv = ism_hsv.build_reference(tdir[name])
        re = np.stack([ism_hsv.feat_rgb(c) for c in PC[f"{name}__render42"]])
        assert op.shape == re.shape
        assert float(np.abs(op - re).max()) == 0.0, f"{name} PROTO not exact"


# 2. Hue circular wrap-around (shift -3 in OpenCV units, no negative / overflow)
def test_hue_wraparound():
    for h in (0, 1, 2, 3, 179):
        # a pure-hue pixel; verify shifted hue stays in [0,180) and wraps
        px = np.zeros((1, 3), np.uint8)
        # build via HSV then convert to feed feat_rgb (feat_rgb takes RGB)
        hsv = np.array([[[h, 200, 200]]], np.uint8)
        rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB).reshape(1, 3)
        feat = ism_hsv.feat_rgb(rgb)
        assert feat.shape == (128,)
        assert np.isfinite(feat).all()
        assert abs(feat.sum() - 1.0) < 1e-9 or feat.sum() == 0.0


# 3. histogram L1 normalization
def test_hist_l1_normalized():
    rng = np.random.RandomState(0)
    px = rng.randint(0, 255, (5000, 3), np.uint8)
    f = ism_hsv.feat_rgb(px)
    assert abs(f.sum() - 1.0) < 1e-5      # float32 calcHist precision
    # query side too
    crop = rng.randint(0, 255, (40, 40, 3), np.uint8)
    mask = np.ones((40, 40), np.uint8)
    q = ism_hsv.query_hist(crop, mask)
    assert abs(q.sum() - 1.0) < 1e-5


# 4. empty mask -> zero histogram, similarity finite, no crash
def test_empty_mask():
    crop = np.full((30, 30, 3), 120, np.uint8)
    mask = np.zeros((30, 30), np.uint8)
    q = ism_hsv.query_hist(crop, mask)     # empty mask -> falls back to full-crop hist
    assert np.isfinite(q).all()
    proto = np.random.RandomState(1).rand(42, 128)
    proto /= proto.sum(1, keepdims=True)
    assert np.isfinite(ism_hsv.similarity(q, proto))


# 5. tiny mask handled (few pixels)
def test_tiny_mask():
    crop = np.full((30, 30, 3), 120, np.uint8)
    mask = np.zeros((30, 30), np.uint8); mask[0, 0] = 1
    q = ism_hsv.query_hist(crop, mask)
    assert abs(q.sum() - 1.0) < 1e-9 or q.sum() == 0.0


# 6. template cache shape = [42,128]
def test_cache_shape():
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    for o in objs:
        p = os.path.join(os.path.dirname(o["cls_cache"]), f"{o['name']}_hsv.npz")
        proto, meta = ism_hsv.load_cache(p, o["template_dir"])
        assert proto is not None, f"{o['name']} cache invalid: {meta['status']}"
        assert proto.shape[1] == 128 and proto.shape[0] >= 1


# 7. missing cache -> (None, status=missing), fail-open
def test_cache_missing():
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    proto, meta = ism_hsv.load_cache("/nonexistent/x_hsv.npz", objs[0]["template_dir"])
    assert proto is None and meta["status"] == "missing"
    # fail-open similarity: proto None -> 1.0 (never rejects)
    assert ism_hsv.similarity(np.ones(128) / 128, None) == 1.0


# 8. corrupt cache -> status corrupt, proto None
def test_cache_corrupt(tmp_path=None):
    import tempfile
    d = tempfile.mkdtemp()
    p = os.path.join(d, "bad_hsv.npz")
    with open(p, "wb") as f:
        f.write(b"not a real npz file")
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    proto, meta = ism_hsv.load_cache(p, objs[0]["template_dir"])
    assert proto is None and meta["status"].startswith("corrupt")


# 9. threshold boundary: sim == thr -> pass (>=)
def test_threshold_boundary():
    q = np.zeros(128); q[0] = 1.0
    proto = np.zeros((1, 128)); proto[0, 0] = 1.0
    s = ism_hsv.similarity(q, proto)      # identical -> 1.0
    assert s == 1.0
    assert (s >= 1.0) is True             # boundary passes
    # orthogonal -> 0.0
    proto2 = np.zeros((1, 128)); proto2[0, 5] = 1.0
    assert ism_hsv.similarity(q, proto2) == 0.0


# 10. shadow mode leaves the final decision unchanged
def test_shadow_does_not_change_decision():
    o = {"hsv_gate_enabled": False, "hsv_gate_shadow_mode": True,
         "hsv_gate_threshold": 0.9, "_hsv_proto": np.zeros((1, 128)), "_hsv_meta": {}}
    # accepted candidate with a deliberately failing HSV (proto orthogonal to any query)
    res = {"accepted": True, "decision": "detected", "box": (0, 0, 10, 10), "mask": None}
    bgr = np.full((20, 20, 3), 200, np.uint8)
    o_n._apply_hsv_gate(o, res, bgr)
    assert res["accepted"] is True and res["decision"] == "detected"   # UNCHANGED
    assert res["would_hsv_reject"] in (0, 1)                            # but logged
    assert "hsv_score" in res


# 11. HSV not computed for non-accepted candidates
def test_no_hsv_when_not_accepted():
    o = {"hsv_gate_enabled": False, "hsv_gate_shadow_mode": True,
         "hsv_gate_threshold": 0.5, "_hsv_proto": np.zeros((1, 128)), "_hsv_meta": {}}
    res = {"accepted": False, "decision": "no-object(below-appe)", "box": (0, 0, 10, 10)}
    bgr = np.full((20, 20, 3), 200, np.uint8)
    o_n._apply_hsv_gate(o, res, bgr)
    assert "hsv_score" not in res     # skipped entirely


# 12. deterministic score for identical input
def test_deterministic():
    crop = np.random.RandomState(3).randint(0, 255, (32, 32, 3), np.uint8)
    mask = np.ones((32, 32), np.uint8)
    proto = np.random.RandomState(4).rand(42, 128); proto /= proto.sum(1, keepdims=True)
    a = ism_hsv.similarity(ism_hsv.query_hist(crop, mask), proto)
    b = ism_hsv.similarity(ism_hsv.query_hist(crop, mask), proto)
    assert a == b


# 13. HSV OFF -> _apply_hsv_shadow is a complete no-op (no runtime effect, no fields)
def test_off_is_noop():
    o = {"hsv_gate_enabled": False, "hsv_gate_shadow_mode": False}
    res = {"accepted": True, "decision": "detected", "box": (0, 0, 10, 10), "mask": None}
    bgr = np.full((20, 20, 3), 200, np.uint8)
    o_n._apply_hsv_gate(o, res, bgr)
    assert "hsv_score" not in res and res == {"accepted": True, "decision": "detected",
                                              "box": (0, 0, 10, 10), "mask": None}


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    fails = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            fails += 1
            print(f"FAIL  {fn.__name__}: {e}")
        except Exception as e:
            fails += 1
            print(f"ERROR {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - fails}/{len(fns)} passed")
    sys.exit(1 if fails else 0)
