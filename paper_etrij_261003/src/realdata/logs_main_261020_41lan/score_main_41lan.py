"""2026-10-20 (.41 hub + .192 modified ORB, run_main_lan41.sh): score_unused_41lan.py with the "main configuration" run names (gyro where IMU, else 3000 features; 3 repeats + original 0.5).

These recordings have no earlier run, so (as ref_260910_object_ours in doc 08) each one's reference is the final
map of a deployed-recogniser run on it (ref_<ses>_ours_l5090). Unlike eval_real.reference() (median over several
runs) a single run may hold a class twice or not at all: a class is a reference only when the reference run holds
it once and it has >= MIN_EST estimates; the denominator is the number of reference classes (8 when complete).
A map entry is correct when its class is a reference class and within 20 cm of it, one per class; every other entry
of the 8 shelf classes or any other class is wrong.

    python score_unused.py [session ...]   -> results_main_41lan.json
"""
import bisect, json, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3] / "objpose" / "output"
OBJS = ["saffron", "Febreze_high", "Dinosaur", "Bear", "milk", "choco_hazelnut_high", "Mugcup_high", "Sikhye_high"]
RADIUS, MIN_EST, TT = 0.20, 5, [10, 30, 60, 120]
SESSIONS = ["260901_cbnu_digut", "260915_longcircle", "260826_etri_longcircle_dark", "260915_inlong_fast", "260915_inlong_middle",
            "260915_outlong_fast", "260915_outlong_middle", "260915_height", "260915_dense", "260915_distance"]
CONFIGS = [("ref", "ref_{s}_ours_41lan"), ("origL05_main", "live_261020_{s}_41lan_origL05_main"), ("main_r1", "live_261020_{s}_41lan_hyb3cf_main_r1"),
           ("main_r2", "live_261020_{s}_41lan_hyb3cf_main_r2"), ("main_r3", "live_261020_{s}_41lan_hyb3cf_main_r3")]


def final_entries(run):
    s = json.load(open(run / "summary.json"))
    return s, [(n.split("#")[0], np.asarray(o["T_w_obj"], float).reshape(4, 4)[:3, 3], o.get("est_count", 0))
               for n, o in s.get("objects", {}).items()]


def reference(run):
    _, ents = final_entries(run)
    by = {}
    for n, p, c in ents:
        by.setdefault(n, []).append((p, c))
    ref = {n: v[0][0] for n, v in by.items() if n in OBJS and len(v) == 1 and v[0][1] >= MIN_EST}
    notes = {n: ("twice" if len(by[n]) > 1 else f"{by[n][0][1]} estimates") if n in by else "absent"
             for n in OBJS if n not in ref}
    others = sorted({n for n in by if n not in OBJS})
    return ref, notes, others


def judge(entries, ref):
    ok, wrong, kinds = set(), 0, {"duplicate": 0, "label": 0, "elsewhere": 0}
    for n, p in entries:
        near = [m for m, q in ref.items() if np.linalg.norm(p - q) <= RADIUS]
        if n in ref and n not in ok and n in near:
            ok.add(n); continue
        wrong += 1
        kinds["duplicate" if n in near else "label" if near else "elsewhere"] += 1
    return ok, wrong, kinds


def run_one(run, ref):
    s, fin = final_entries(run)
    X = np.asarray(s["X_slam_sam"], float).reshape(4, 4)
    poses = [p for p in map(json.loads, open(run / "slam_poses.jsonl")) if p.get("T_wc")]
    pt = [p["t_ns"] for p in poses]
    snaps, first, t0, N = [], {}, None, max(len(ref), 1)
    for line in open(run / "display_objects.jsonl"):
        d = json.loads(line)
        t0 = t0 or d["t_ns"]
        i = min(bisect.bisect_left(pt, d["t_ns"]), len(pt) - 1)
        Twc = np.asarray(poses[i]["T_wc"], float).reshape(4, 4)
        ents = [(o["name"].split("#")[0], (Twc @ X @ np.asarray(o["T_cam_obj"], float).reshape(4, 4))[:3, 3])
                for o in d["objects"] if o.get("T_cam_obj")]
        ok, w, _ = judge(ents, ref)
        t = (d["t_ns"] - t0) / 1e9
        for n in ok:
            first.setdefault(n, t)
        snaps.append((t, ok, w))
    ok, w, kinds = judge([(n, p) for n, p, _ in fin], ref)
    dur = snaps[-1][0] if snaps else 1.0
    frames = sum(1 for _ in open(run / "sam6d_frames.jsonl"))
    at = lambda t: ([v for v in snaps if v[0] <= t] or [(t, set(), 0)])[-1]
    ft = [first[n] for n in ref if n in first]
    dist = {n: round(min(float(np.linalg.norm(p - ref[n])) for m, p, _ in fin if m == n) * 100, 1)
            for n in ref if any(m == n for m, _, _ in fin)}
    return {"correct": len(ok), "wrong": w, "of": len(ref), "score_pct": round(100 * len(ok) / (N + w), 1),
            "missed": sorted(set(ref) - ok), "wrong_kinds": kinds,
            "wrong_classes": sorted(n for n, p, _ in fin if not (n in ref and np.linalg.norm(p - ref[n]) <= RADIUS)),
            "rate_hz": round(frames / max(dur, 1e-6), 2), "duration_s": round(dur, 1),
            "at": {str(t): [len(at(t)[1]), at(t)[2]] for t in TT},   # [correct, wrong] at t s
            "mean_score_pct": round(100 * float(np.mean([len(o) / (N + x) for _, o, x in snaps])), 1) if snaps else None,
            "time_to_map_median_s": round(float(np.median(ft)), 1) if len(ft) == len(ref) and ft else None,
            "dist_cm": dist, "kf_max_cm": s.get("kf_max_correction_cm"), "loops": s.get("slam_loop_events"),
            "slam_dropped": s.get("slam_dropped"), "sam6d_frames": s.get("sam6d_frames")}


def main():
    out, res = HERE / "results_main_41lan.json", {}
    for ses in sys.argv[1:] or SESSIONS:
        runs = {k: ROOT / f.format(s=ses) for k, f in CONFIGS}
        if not (runs["ref"] / "summary.json").exists():
            res[ses] = None; print(ses, "no reference run"); continue
        ref, notes, others = reference(runs["ref"])
        row = {"reference": {n: [round(float(v), 3) for v in p] for n, p in ref.items()},
               "not_reference": notes, "ref_other_classes": others}
        for k, d in runs.items():
            row[k] = run_one(d, ref) if (d / "summary.json").exists() else None
        res[ses] = row
        f = lambda r: "-" if r is None else (f"{r['correct']}/{r['wrong']} (of {r['of']}) {r['score_pct']}% "
                                              f"{r['rate_hz']}Hz mean {r['mean_score_pct']}% loop {r['kf_max_cm']}cm")
        print(ses, f"ref={len(ref)} not={notes}", " | ".join(f"{k}: {f(row[k])}" for k in runs))
    json.dump(res, open(out, "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
