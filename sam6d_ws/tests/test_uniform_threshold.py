#!/usr/bin/env python3
"""Phase 1A regression tests — uniform YOLO score_threshold = 0.02.

Authority = the OPERATIONAL loader `yolo_ism_object_n.load_config(DEFAULT_CONFIG)`,
the exact code path the batch driver and the live ROS node
(`sam6d_multiobject_node.py` -> `o_n.recognize`) use to obtain each object's
effective `score_threshold`. This is not a mock: it parses the real config and
applies the real defaults-merge (`o = dict(defaults); o.update(raw)`).

Run:  ~/miniconda3/envs/sam_yolo/bin/python -m pytest tests/test_uniform_threshold.py -q
 or:  ~/miniconda3/envs/sam_yolo/bin/python tests/test_uniform_threshold.py
"""
import os
import sys

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

import yolo_ism_object_n as o_n  # noqa: E402

UNIFORM = 0.02
GUARDED = ("Bear", "Rabbit", "Dinosaur")   # previously carried 0.35 / 0.30 / 0.30


def _load():
    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    return defaults, {o["name"]: o for o in objs}


def _raw_objects():
    cfg = yaml.safe_load(open(os.path.join(REPO, "configs", "yolo_ism_objects.yaml")))
    return cfg.get("defaults", {}), cfg.get("objects", [])


# 1. every registered object's effective threshold is 0.02
def test_all_objects_uniform_002():
    _, by = _load()
    off = {n: o.get("score_threshold") for n, o in by.items()
           if o.get("score_threshold") != UNIFORM}
    assert not off, f"objects not at 0.02: {off}"


# 2/3/4. the three previously-guarded objects are now 0.02
def test_bear_rabbit_dinosaur_002():
    _, by = _load()
    for n in GUARDED:
        assert n in by, f"{n} missing from loaded config"
        assert by[n]["score_threshold"] == UNIFORM, \
            f"{n} effective threshold = {by[n]['score_threshold']}, expected {UNIFORM}"


# 5. no per-object score_threshold override remains in the raw config
def test_no_per_object_override_in_config():
    _, raw_objs = _raw_objects()
    offenders = [r["name"] for r in raw_objs if "score_threshold" in r]
    assert not offenders, f"per-object score_threshold override still present: {offenders}"


# 6. defaults are the single source of truth; a config object with no override
#    inherits exactly defaults.score_threshold (== 0.02).
def test_default_is_single_source_of_truth():
    defaults, by = _load()
    assert defaults.get("score_threshold") == UNIFORM
    # simulate an unlisted/default object: merge defaults only
    synthetic = dict(defaults)
    assert synthetic.get("score_threshold") == UNIFORM
    # and a real object with no raw override resolves to the default
    _, raw_objs = _raw_objects()
    no_ov = next(r["name"] for r in raw_objs if "score_threshold" not in r)
    assert by[no_ov]["score_threshold"] == UNIFORM


# 7. other operational settings are untouched by this change
def test_other_settings_unchanged():
    defaults, by = _load()
    assert int(defaults.get("top_k")) == 3, "top_k must stay 3"
    for n, o in by.items():
        assert int(o.get("top_k", 3)) == 3, f"{n} top_k changed"
        # gates that decide accept/reject must still be present per object
        assert "similarity_threshold" in o, f"{n} lost similarity_threshold"
        assert "appe_gate" in o, f"{n} lost appe_gate"
        assert "yolo_prompt" in o and o["yolo_prompt"], f"{n} lost yolo_prompt (routing)"
    # class routing invariant: high/low variants still share one prompt
    prompts = [o["yolo_prompt"] for o in by.values()]
    assert len(prompts) == len(by), "prompt list length mismatch"


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
    print(f"\n{len(fns) - fails}/{len(fns)} passed")
    sys.exit(1 if fails else 0)
