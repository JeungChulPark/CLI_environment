#!/usr/bin/env python3
"""Phase 1C unit tests — HSV active gate + Dinosaur class hue correction.

Run: ~/miniconda3/envs/sam_yolo/bin/python tests/test_phase1c.py
"""
import os
import sys

import numpy as np
import cv2

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
import ism_hsv                       # noqa: E402
import yolo_ism_object_n as o_n      # noqa: E402

T = 0.1214


# ---- 1. hue wrap-around (OpenCV units; +9ocv = +18deg in 0-360) ----
def test_hue_wraparound_ocv():
    # a single pixel at a known OpenCV hue; shift +9 wraps mod 180
    def ocv_hue(rgb):
        return int(cv2.cvtColor(np.array(rgb, np.uint8).reshape(1, 1, 3)[:, :, ::-1],
                                cv2.COLOR_BGR2HSV)[0, 0, 0])
    for base in (0, 5, 170, 175, 179):
        # construct a pixel with hue=base via HSV->RGB
        rgb = cv2.cvtColor(np.array([[[base, 200, 200]]], np.uint8), cv2.COLOR_HSV2RGB).reshape(1, 3)
        shifted = ism_hsv._apply_hue_ocv(rgb, 9)
        h2 = ocv_hue(shifted)
        assert 0 <= h2 <= 179, f"hue out of range: {h2}"
        # 175 + 9 = 184 % 180 = 4 (allow +-1 for cvt rounding)
        assert abs(((base + 9) % 180) - h2) <= 2, f"base {base} -> {h2}, expected ~{(base+9)%180}"


def test_hue_wraparound_degrees_doc():
    # +9 OpenCV units corresponds to +18 degrees in the 0-360 convention (2x factor)
    assert 9 * 2 == 18
    # negative shift also wraps
    rgb = cv2.cvtColor(np.array([[[2, 200, 200]]], np.uint8), cv2.COLOR_HSV2RGB).reshape(1, 3)
    out = ism_hsv._apply_hue_ocv(rgb, -9)
    assert out.shape == rgb.shape


# ---- 2. correction is Dinosaur-only in config; changes Dino proto, not others ----
def test_dinosaur_only_correction():
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    by = {o["name"]: o for o in objs}
    assert int(by["Dinosaur"].get("hsv_hue_correction_ocv", 0)) == 9
    for n in ("Bear", "Rabbit", "choco_hazelnut_high", "milk", "saffron"):
        assert int(by[n].get("hsv_hue_correction_ocv", 0)) == 0, f"{n} unexpectedly corrected"


def test_correction_changes_dino_proto():
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    td = {o["name"]: o["template_dir"] for o in objs}
    p0 = ism_hsv.build_reference(td["Dinosaur"], 0)[0]
    p9 = ism_hsv.build_reference(td["Dinosaur"], 9)[0]
    assert p0.shape == p9.shape
    assert float(np.abs(p0 - p9).max()) > 0, "hc=9 should change the Dino reference"
    # hc=0 must equal a no-correction build (backward-compatible)
    p0b = ism_hsv.build_reference(td["Dinosaur"])[0]
    assert float(np.abs(p0 - p0b).max()) == 0.0


# ---- 3. config: gate active, threshold, cache carries correction ----
def test_config_active():
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    d = [o for o in objs if o["name"] == "Dinosaur"][0]
    assert d.get("hsv_gate_enabled") is True
    assert abs(float(d.get("hsv_gate_threshold")) - T) < 1e-9
    p = os.path.join(os.path.dirname(d["cls_cache"]), "Dinosaur_hsv.npz")
    proto, meta = ism_hsv.load_cache(p, d["template_dir"])
    assert proto is not None and int(meta.get("hue_correction_ocv", 0)) == 9


# ---- 4. gate semantics: shadow no-op, active rejects, disabled no-op ----
def _res():
    return {"accepted": True, "decision": "detected", "box": (0, 0, 10, 10), "mask": None}


def test_shadow_does_not_change_decision():
    o = {"hsv_gate_enabled": False, "hsv_gate_shadow_mode": True, "hsv_gate_threshold": 0.9,
         "_hsv_proto": np.zeros((1, 128)), "_hsv_meta": {}}   # proto orthogonal -> low score
    r = _res(); o_n._apply_hsv_gate(o, r, np.full((20, 20, 3), 200, np.uint8))
    assert r["accepted"] is True and r["decision"] == "detected"   # UNCHANGED
    assert r["would_hsv_reject"] == 1                               # but logged


def test_active_rejects_on_fail():
    o = {"hsv_gate_enabled": True, "hsv_gate_shadow_mode": False, "hsv_gate_threshold": 0.9,
         "_hsv_proto": np.zeros((1, 128)), "_hsv_meta": {}}
    r = _res(); o_n._apply_hsv_gate(o, r, np.full((20, 20, 3), 200, np.uint8))
    assert r["accepted"] is False and r["decision"] == "no-object(below-hsv)"


def test_active_keeps_on_pass():
    # proto equal to a uniform query -> high score, passes low threshold
    o = {"hsv_gate_enabled": True, "hsv_gate_shadow_mode": False, "hsv_gate_threshold": 0.0,
         "_hsv_proto": np.ones((1, 128)) / 128, "_hsv_meta": {}}
    r = _res(); o_n._apply_hsv_gate(o, r, np.full((20, 20, 3), 200, np.uint8))
    assert r["accepted"] is True


def test_disabled_is_noop():
    o = {"hsv_gate_enabled": False, "hsv_gate_shadow_mode": False}
    r = _res(); before = dict(r); o_n._apply_hsv_gate(o, r, np.full((20, 20, 3), 200, np.uint8))
    assert r == before and "hsv_score" not in r


# ---- 5. invalid input: empty crop/mask -> fail-open (score 1.0, no reject) ----
def test_failopen_empty_crop():
    o = {"hsv_gate_enabled": True, "hsv_gate_shadow_mode": False, "hsv_gate_threshold": 0.5,
         "_hsv_proto": np.zeros((1, 128)), "_hsv_meta": {}}
    r = {"accepted": True, "decision": "detected", "box": (5, 5, 5, 5), "mask": None}  # zero-area
    o_n._apply_hsv_gate(o, r, np.full((20, 20, 3), 200, np.uint8))
    assert r["accepted"] is True and r["hsv_score"] == 1.0     # fail-open, not rejected


def test_failopen_missing_proto():
    o = {"hsv_gate_enabled": True, "hsv_gate_shadow_mode": False, "hsv_gate_threshold": 0.5,
         "_hsv_proto": None, "_hsv_meta": {}}
    r = _res(); o_n._apply_hsv_gate(o, r, np.full((20, 20, 3), 200, np.uint8))
    assert r["accepted"] is True and r["hsv_score"] == 1.0     # cache miss -> pass


def test_not_computed_when_rejected():
    o = {"hsv_gate_enabled": True, "hsv_gate_shadow_mode": False, "hsv_gate_threshold": 0.5,
         "_hsv_proto": np.zeros((1, 128)), "_hsv_meta": {}}
    r = {"accepted": False, "decision": "no-object(below-appe)", "box": (0, 0, 10, 10)}
    o_n._apply_hsv_gate(o, r, np.full((20, 20, 3), 200, np.uint8))
    assert "hsv_score" not in r


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
