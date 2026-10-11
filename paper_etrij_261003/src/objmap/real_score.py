"""Score the object maps of the real recordings (common.REAL_SESSIONS) with the rule of realdata/eval_real.py.

No object ground truth exists: the reference is the position of each of the 8 shelf objects (eval_real
reference(): median over earlier deployed runs of ours on the recording). A map object is correct when its
class is one of the 8 and its position is within 20 cm of that object's reference, one correct object per
class; every other named object is wrong (wrong place, duplicate). Score = correct / (8 + wrong).
Objects labelled 'unknown' (DualMap's FastSAM segments) claim no class and are not counted.

Position of an object: centroid of its points (largest connected cluster, as evaluate.prep with --clean) for the
fusion maps; for ours the final keyframe-anchored map of the live run (summary.json T_w_obj), the map
eval_real scores, so the numbers are those of results_real*.json.

    python real_score.py [--scenes 1 2 ...] [--methods fmfusion dualmap dualmap_bgr]
"""
import argparse
import importlib
import json
import os
import sys
from pathlib import Path

import numpy as np

from common import OUT, REAL_NAMES, REAL_SESSIONS, load_map
from evaluate import clean, downsample

REPO = Path(__file__).resolve().parents[3]
REALDATA = Path(__file__).resolve().parent.parent / "realdata"
SESSION_KEY = {"260901_cbnu_bigeightcircle": "260901", "260915_eightcircle": "260915"}


def reference(session):
    """eval_real's reference for a recording (its module reads the session at import time)."""
    os.environ["EVAL_REAL_SESSION"] = SESSION_KEY.get(session, session)
    sys.path.insert(0, str(REALDATA))
    import eval_real
    eval_real = importlib.reload(eval_real)
    return eval_real, eval_real.reference()


def score(entries, ref, judge, radius):
    """eval_real's judge, plus the kind of every wrong entry: duplicate (a second object on a found object),
    label (on another of the 8 objects, wrong class), elsewhere (near none of the 8: background, wrong place)."""
    ok, wrong = judge(entries, ref)
    near = lambda p: [m for m, q in ref.items() if np.linalg.norm(p - q) <= radius]
    kinds, seen = {"duplicate": 0, "label": 0, "elsewhere": 0}, set()
    for n, p in entries:
        at = near(p)
        if n in at and n not in seen:
            seen.add(n)
        elif n in at:
            kinds["duplicate"] += 1
        elif at:
            kinds["label"] += 1
        else:
            kinds["elsewhere"] += 1
    return {"correct": len(ok), "wrong": wrong, "entries": len(entries), "missed": sorted(set(REAL_NAMES) - ok),
            "score_pct": round(100 * len(ok) / (8 + wrong), 1), "wrong_kinds": kinds}


def fusion_entries(method, scene):
    d = OUT / method / "real" / f"{scene:06d}"
    if not (d / "map.json").exists():
        return None
    _, objs = load_map(d, cad_if_no_points=False)
    out = []
    for o in objs:
        if o["label"] in REAL_NAMES:
            P = clean(downsample(o["points"]))   # as evaluate.prep(..., do_clean=True)
            if len(P):
                out.append((o["label"], P.mean(0)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", type=int, nargs="*", default=sorted(REAL_SESSIONS))
    ap.add_argument("--methods", nargs="+", default=["fmfusion", "dualmap", "dualmap_bgr"])
    a = ap.parse_args()
    res = json.load(open(OUT / "real_scores.json")) if (OUT / "real_scores.json").exists() else {}
    for s in a.scenes:
        session, run = REAL_SESSIONS[s]
        if not (REPO / "objpose/output" / run / "summary.json").exists():
            print(f"{session}: run {run} missing, skipped")
            continue
        er, ref = reference(session)
        row = {"session": session, "run": run, "radius_m": er.RADIUS,
               "reference": {n: [round(float(v), 3) for v in p] for n, p in ref.items()}}
        o = er.run_one(REPO / "objpose/output" / run, ref)
        fin = [(n.split("#")[0], np.asarray(v["T_w_obj"], float).reshape(4, 4)[:3, 3])
               for n, v in json.load(open(REPO / "objpose/output" / run / "summary.json")).get("objects", {}).items()]
        row["ours"] = dict(score(fin, ref, er.judge, er.RADIUS), rate_hz=o["rate_hz"], mean_score_pct=o["mean_score_pct"])
        assert row["ours"]["correct"] == o["final_map"]["correct"] and row["ours"]["wrong"] == o["final_map"]["wrong"]
        for m in a.methods:
            e = fusion_entries(m, s)
            if e is not None:
                row[m] = score(e, ref, er.judge, er.RADIUS)
        res[session] = row
        print(session, " ".join(f"{m}={row[m]['correct']}/{row[m]['wrong']}" for m in ["ours"] + a.methods if m in row))
    json.dump(res, open(OUT / "real_scores.json", "w"), indent=1)
    print("->", OUT / "real_scores.json")


if __name__ == "__main__":
    main()
