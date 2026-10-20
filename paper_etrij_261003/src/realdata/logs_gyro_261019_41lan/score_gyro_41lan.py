"""2026-10-19: score the runs of ~/Dataset/bop/ycbv_work/run_gyro_lan41.sh (.41 hub + improvement 3 + cluster-first,
.192 modified ORB-SLAM3 + gyro) with the doc 30 rule (score_gyro.py / score_unused.py: 20 cm, one correct entry per
reference class). Reference positions as in doc 30: eightcircle / bigeightcircle from the deployed ORB-SLAM3
reference runs on this machine (eval_real.REF_BY_SESSION / REF_RUNS); digut / longcircle have no reference run here,
so the laptop's reference positions (ref_<ses>_ours_l5090, ../logs_gyro_261018/results_gyro_l5090.json) are reused.

    python score_gyro_41lan.py [session ...]   -> results_gyro_41lan.json
"""
import json, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "logs_unused_261017")); sys.path.insert(0, str(HERE.parents[1] / "objmap"))
import score_unused as su   # noqa: E402
import real_score           # noqa: E402

SESSIONS = ["260901_cbnu_eightcircle", "260901_cbnu_bigeightcircle", "260901_cbnu_digut", "260901_cbnu_longcircle"]
CONFIGS = [("origL05_orb", "live_261019_{s}_41lan_origL05_orb")]
CONFIGS += [(k + r, f + r) for r in ("", "_r2", "_r3", "_redo")
            for k, f in (("hyb3cf_orb", "live_261019_{s}_41lan_hyb3cf_orb"), ("hyb3cf_orbgyro", "live_261019_{s}_41lan_hyb3cf_orbgyro"))]
# a run whose recogniser fell far below its usual rate (bigeightcircle hyb3cf_orb 17:00: 518 frames, 0 objects while the
# FM-Fusion detector of the ObChange chain started on the same machine) is kept in the json but flagged
LOW_RATE_HZ = 3.0
LAPTOP = json.load(open(HERE.parent / "logs_gyro_261018" / "results_gyro_l5090.json"))
LAPTOP_REF = {"260901_cbnu_digut", "260901_cbnu_longcircle"}


def main():
    res = {}
    for ses in sys.argv[1:] or SESSIONS:
        if ses in LAPTOP_REF:
            ref = {n: np.asarray(p, float) for n, p in LAPTOP[ses]["reference"].items()}
            src = "laptop " + LAPTOP[ses]["reference_from"]
        else:
            er, ref = real_score.reference(ses); ref = {n: np.asarray(p, float) for n, p in ref.items()}
            src = ",".join(er.REF_RUNS)
        row = {"reference_from": src, "reference": {n: [round(float(v), 3) for v in p] for n, p in ref.items()}}
        for k, f in CONFIGS:
            d = su.ROOT / f.format(s=ses)
            row[k] = su.run_one(d, ref) if (d / "summary.json").exists() else None
            if row[k] and row[k]["rate_hz"] < LOW_RATE_HZ:
                row[k]["invalid"] = f"recogniser rate {row[k]['rate_hz']} Hz < {LOW_RATE_HZ} (machine contention)"
        # the laptop's gyro result on the same recording, for the side-by-side table
        lap = LAPTOP.get(ses) or {}
        row["laptop"] = {k: lap.get(k) for k in ("origL05_orb", "hyb3cf_orb", "hyb3cf_orb_r2", "hyb3cf_orb_r3",
                                                  "hyb3cf_orbgyro", "hyb3cf_orbgyro_r2", "hyb3cf_orbgyro_r3")}
        res[ses] = row
        fm = lambda r: "-" if r is None else (f"{r['correct']}/{r['wrong']} {r['rate_hz']}Hz loop {r['kf_max_cm']}cm"
                                               + (" INVALID" if r.get("invalid") else ""))
        print(ses, f"ref={len(ref)}", " | ".join(f"{k}: {fm(row[k])}" for k, _ in CONFIGS if row[k] is not None))
    json.dump(res, open(HERE / "results_gyro_41lan.json", "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
