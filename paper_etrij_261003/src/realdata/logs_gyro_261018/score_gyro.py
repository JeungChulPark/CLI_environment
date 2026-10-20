"""Doc 30: score the runs of run_gyro_l5090.sh with the rule of ../logs_unused_261017/score_unused.py (20 cm, one
correct entry per reference class). Reference positions: eightcircle and bigeightcircle keep the median over their
earlier deployed ORB-SLAM3 runs (eval_real.REF_BY_SESSION / REF_RUNS, as docs 22-24); digut and longcircle have no
such runs, so their reference is the final map of one deployed-recogniser run (ref_<ses>_ours_l5090).

    python score_gyro.py [session ...]   -> results_gyro_l5090.json
"""
import json, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "logs_unused_261017")); sys.path.insert(0, str(HERE.parents[1] / "objmap"))
import score_unused as su   # noqa: E402
import real_score           # noqa: E402

SESSIONS = ["260901_cbnu_eightcircle", "260901_cbnu_bigeightcircle", "260901_cbnu_digut", "260901_cbnu_longcircle"]
CONFIGS = [("origL05_orb", "live_261018_{s}_l5090_origL05_orb"), ("hyb3cf_orb", "live_261018_{s}_l5090_hyb3cf_orb"),
           ("hyb3cf_orbgyro", "live_261018_{s}_l5090_hyb3cf_orbgyro")]
CONFIGS += [(k + r, f + r) for r in ("_r2", "_r3") for k, f in CONFIGS[1:]]   # repeats (SUF=_r2/_r3 HYB_ONLY=1)
# unmodified ORB-SLAM3 V1.0 (run_orbup_l5090.sh)
CONFIGS += [("origL05_orbup", "live_261018_{s}_l5090_origL05_orbup")]
CONFIGS += [("hyb3cf_orbup" + r, "live_261018_{s}_l5090_hyb3cf_orbup" + r) for r in ("", "_r2", "_r3")]
REF_RUN = {"260901_cbnu_digut": "ref_260901_cbnu_digut_ours_l5090",
           "260901_cbnu_longcircle": "ref_260901_cbnu_longcircle_ours_l5090"}


def main():
    res = {}
    for ses in sys.argv[1:] or SESSIONS:
        if ses in REF_RUN:
            if not (su.ROOT / REF_RUN[ses] / "summary.json").exists():
                res[ses] = None; print(ses, "no reference run"); continue
            ref, notes, _ = su.reference(su.ROOT / REF_RUN[ses]); src = REF_RUN[ses]
        else:
            er, ref = real_score.reference(ses); ref = {n: np.asarray(p, float) for n, p in ref.items()}
            notes, src = {}, ",".join(er.REF_RUNS)
        row = {"reference_from": src, "reference": {n: [round(float(v), 3) for v in p] for n, p in ref.items()},
               "not_reference": notes}
        for k, f in CONFIGS:
            d = su.ROOT / f.format(s=ses)
            row[k] = su.run_one(d, ref) if (d / "summary.json").exists() else None
        res[ses] = row
        fm = lambda r: "-" if r is None else (f"{r['correct']}/{r['wrong']} (of {r['of']}) {r['score_pct']}% "
                                               f"{r['rate_hz']}Hz loop {r['kf_max_cm']}cm drop {r['slam_dropped']}")
        print(ses, f"ref={len(ref)}", " | ".join(f"{k}: {fm(row[k])}" for k, _ in CONFIGS))
    json.dump(res, open(HERE / "results_gyro_l5090.json", "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
