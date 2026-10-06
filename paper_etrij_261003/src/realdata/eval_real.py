"""Score live runs on the real recording 260901_cbnu_bigeightcircle (no object pose ground truth).

Reference: the 8 objects on the two shelves (checked on the video: saffron, Febreze_high, Dinosaur,
Bear on one shelf; milk, choco_hazelnut_high, Mugcup_high, Sikhye_high on the other). Their map
positions are the median over earlier deployed runs on this recording (live_260901_..._orbslam3,
_f2000_dev01, _f2000_dup5; spread median 0.2-2.5 cm). All runs use the same Mac ORB-SLAM3 f2000
and extrinsic, so their map frames coincide.

A map entry is correct when its class is one of the 8 and its position is within RADIUS (20 cm,
above the largest loop-closure correction seen on this recording, 15 cm) of that object's reference;
one correct entry per object; everything else is wrong (absent class, wrong place, duplicate).
Positions only: no orientation check (no ground truth).

Per display frame (the whole map is logged as T_cam_obj), position in the map = T_wc(SLAM, nearest
pose) . T_slam_sam . T_cam_obj. Reported at 10/30/60/120 s and at the end: correct, wrong,
score = correct / (8 + wrong); time to map per object; recognition rate.

    python eval_real.py --runs real_260901_ours real_260901_g13 ...

EVAL_REAL_SESSION=<recording> scores another recording with the same 8 objects (REF_BY_SESSION below):
its reference is the median over deployed runs on that recording (Mac ORB-SLAM3 f2000). Recordings
without an earlier usable run get a reference run of the deployed recogniser first (ref_<session>_ours).
Each run's own T_slam_sam (summary.json X_slam_sam) is used, so recordings with another extrinsic work.
"""
import argparse, bisect, glob, json, os
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3] / "objpose" / "output"
SESSION = os.environ.get("EVAL_REAL_SESSION", "260901")
_XP = ROOT.parent / f"rt/{SESSION[:6]}/X_slam_sam_{SESSION[:6]}.json"
X = np.asarray(json.load(open(_XP if _XP.exists() else ROOT.parent / "rt/X_slam_sam.json"))["T_slam_sam"],
               float).reshape(4, 4)


def run_X(run):
    """T_slam_sam this run used (falls back to the session default)."""
    s = json.load(open(Path(run) / "summary.json"))
    return np.asarray(s["X_slam_sam"], float).reshape(4, 4) if s.get("X_slam_sam") else X
REF_RUNS = ["live_260901_cbnu_bigeightcircle_orbslam3", "live_260901_cbnu_bigeightcircle_orbslam3_f2000_dev01",
            "live_260901_cbnu_bigeightcircle_orbslam3_f2000_dup5"]
REF_BY_SESSION = {
    "260915": ["live_260915_eightcircle_orbslam3"],
    "260915_eightcircle": ["live_260915_eightcircle_orbslam3"],
    "260826_etri_eightcircle_dark": ["live_260826_etri_dark_orbslam3_f2000", "live_260826_etri_dark_orbslam3_f2000_dup",
                                     "live_260826_etri_dark_orbslam3_f2000_dup5"],
    "260901_cbnu_eightcircle": ["live_260901_cbnu_eightcircle_orbslam3"],
    "260901_cbnu_longcircle": ["ref_260901_cbnu_longcircle_ours"],
    "260910_object": ["ref_260910_object_ours"],
}
REF_RUNS = REF_BY_SESSION.get(SESSION, REF_RUNS)
OBJS = ["saffron", "Febreze_high", "Dinosaur", "Bear", "milk", "choco_hazelnut_high", "Mugcup_high", "Sikhye_high"]
RADIUS = 0.20
TT = [10, 30, 60, 120]


def reference():
    pos = {}
    for r in REF_RUNS:
        for n, o in json.load(open(ROOT / r / "summary.json"))["objects"].items():
            pos.setdefault(n.split("#")[0], []).append(np.asarray(o["T_w_obj"], float).reshape(4, 4)[:3, 3])
    return {n: np.median(np.array(pos[n]), 0) for n in OBJS}


def judge(entries, ref):
    """entries [(name, p_w)] -> (correct set, wrong count)"""
    ok, wrong = set(), 0
    for n, p in entries:
        if n in ref and n not in ok and np.linalg.norm(p - ref[n]) <= RADIUS:
            ok.add(n)
        else:
            wrong += 1
    return ok, wrong


def run_one(run, ref):
    Xr = run_X(run)
    poses = [json.loads(l) for l in open(run / "slam_poses.jsonl")]
    poses = [p for p in poses if p.get("T_wc")]
    pt = [p["t_ns"] for p in poses]
    snaps, first, t0 = [], {}, None
    for line in open(run / "display_objects.jsonl"):
        d = json.loads(line)
        t0 = t0 or d["t_ns"]
        i = min(max(bisect.bisect_left(pt, d["t_ns"]), 0), len(pt) - 1)
        Twc = np.asarray(poses[i]["T_wc"], float).reshape(4, 4)
        ents = []
        for o in d["objects"]:
            if o.get("T_cam_obj"):
                p = (Twc @ Xr @ np.asarray(o["T_cam_obj"], float).reshape(4, 4))[:3, 3]
                ents.append((o["name"].split("#")[0], p))
        ok, wrong = judge(ents, ref)
        t = (d["t_ns"] - t0) / 1e9
        for n in ok:
            first.setdefault(n, t)
        snaps.append((t, ok, wrong))
    s = json.load(open(run / "summary.json"))
    fin = [(n.split("#")[0], np.asarray(o["T_w_obj"], float).reshape(4, 4)[:3, 3]) for n, o in s.get("objects", {}).items()]
    fok, fwrong = judge(fin, ref)
    dur = (snaps[-1][0] if snaps else 1.0)
    frames = sum(1 for _ in open(run / "sam6d_frames.jsonl"))

    def at(t):
        x = [v for v in snaps if v[0] <= t]
        return x[-1] if x else (t, set(), 0)
    out = {"rate_hz": round(frames / max(dur, 1e-6), 2), "duration_s": round(dur, 1),
           "time_to_map_s": {n: (round(first[n], 1) if n in first else None) for n in OBJS},
           "final_map": {"correct": len(fok), "wrong": fwrong, "entries": len(fin), "missed": sorted(set(OBJS) - fok),
                         "score_pct": round(100 * len(fok) / (8 + fwrong), 1)}}
    for t in TT + ["end"]:
        _, ok, w = snaps[-1] if t == "end" else at(t)
        out[f"{t}"] = {"correct": len(ok), "wrong": w, "score_pct": round(100 * len(ok) / (8 + w), 1)}
    out["mean_score_pct"] = round(100 * float(np.mean([len(ok) / (8 + w) for _, ok, w in snaps])), 1) if snaps else None
    ft = [v for v in out["time_to_map_s"].values() if v is not None]
    out["time_to_map_median_s"] = round(float(np.median(ft)), 1) if ft else None
    out["curve"] = [(round(t, 1), len(ok), w) for t, ok, w in snaps[::10]]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", nargs="+", required=True)
    ap.add_argument("--out", default=str(Path(__file__).parent / "results_real.json"))
    a = ap.parse_args()
    ref = reference()
    res = {"reference": {n: [round(float(v), 3) for v in p] for n, p in ref.items()}, "radius_m": RADIUS, "runs": {}}
    for r in a.runs:
        res["runs"][r] = o = run_one(ROOT / r, ref)
        print(r, json.dumps({k: v for k, v in o.items() if k not in ("curve", "time_to_map_s")}))
    json.dump(res, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
