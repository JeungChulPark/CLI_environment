#!/usr/bin/env python3
"""eval_reps.py — score every repeat run of the six ORB-SLAM3 variants (run_map.sh / run_variants.sh / run_reps.sh)
against KISS-ICP, the same way build_map_data.py scores run1 (rig fitted on the run's own optimized trajectory,
camera height pinned), and print mean / min / max ATE per variant. Writes reps/<run>.{json,txt} per session and reps.json.

    ~/objpose/lidar/venv/bin/python eval_reps.py [--dir DIR] [sessions...]      (default: digut kiuk)
--dir: another directory with the same layout (../261025_fix: the race-fixed build); reps.json goes there.
--ref pgo: score against the LiDAR pseudo-GT (rt/gt/out/<session>/pgo_tum.txt) instead of KISS; per-run files go
           to reps_pgo/ and the table to reps_pgo.json.
"""
import json, os, sys, contextlib, io
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "261017_new"))
import build_map_data as bmd  # noqa: E402

VARIANTS = [("base", "modified"), ("dep", "modified + depth fix"), ("up", "upstream"), ("upd", "upstream + depth fix"),
            ("gy", "modified + gyro"), ("gyd", "modified + gyro + depth fix")]


def run_file(O, v, r):
    """run1 of base/dep came from run_map.sh with the _tum suffix; everything else is <v>_run<r>.txt"""
    for fn in ({("base", 1): "base_run1_tum.txt", ("dep", 1): "depth_run1_tum.txt"}.get((v, r)), f"{v}_run{r}.txt"):
        if fn and (O / f"{fn}.optimized.txt").exists():
            return O / fn
    return None


def main():
    a = sys.argv[1:]
    base, ref = HERE, "kiss"
    while a[:1] in (["--dir"], ["--ref"]):
        if a[0] == "--dir":
            base = (HERE / a[1]).resolve()
        else:
            ref = a[1]
        a = a[2:]
    tag = "" if ref == "kiss" else f"_{ref}"
    sess = a or ["digut", "kiuk"]
    out = {}
    for s in sess:
        name = f"261022_cbnu_{s}"; O = base / name; R = O / f"reps{tag}"; R.mkdir(exist_ok=True)
        if not (R / "kiss").exists():  # fit_eval looks for <dir>/kiss/kiss_tum.txt
            if ref == "kiss":
                (R / "kiss").symlink_to("../kiss")
            else:
                (R / "kiss").mkdir()
                (R / "kiss" / "kiss_tum.txt").symlink_to(os.path.relpath(HERE.parent / "gt" / "out" / name / "pgo_tum.txt", R / "kiss"))
        out[s] = {}
        for v, label in VARIANTS:
            rows = []
            for r in range(1, 10):
                if not any(O.glob(f"{v}_run{r}*")) and (v, r) not in (("base", 1), ("dep", 1)):
                    continue
                f = run_file(O, v, r)
                if f is None:
                    if (O / f"{v}_run{r}.log").exists() or (O / f"{'depth' if v == 'dep' else v}_run{r}.log").exists():
                        rows.append({"run": r, "crashed": True})
                    continue
                with contextlib.redirect_stdout(io.StringIO()):
                    _, E, _ = bmd.fit_eval(name, R, f, f"_{v}_run{r}", "live", "opt")
                rows.append({"run": r, "crashed": False, **{k: {
                    "ate_cm": round(E["runs"][k]["ate"]["pos_cm"]["rmse"], 2), "ate_max_cm": round(E["runs"][k]["ate"]["pos_cm"]["max"], 2),
                    "final_cm": round(E["runs"][k]["drift"]["pos_cm_final"], 2)} for k in ("live", "opt")}})
            out[s][v] = rows
    (base / f"reps{tag}.json").write_text(json.dumps(out, indent=1))
    print(f"{'session':8} {'variant':30} {'ok':>5}  ATE live cm (mean min..max)   ATE opt cm (mean min..max)")
    for s, V in out.items():
        for v, label in VARIANTS:
            ok = [x for x in V[v] if not x["crashed"]]
            line = f"{s:8} {label:30} {len(ok)}/{len(V[v])}"
            for k in ("live", "opt"):
                a = [x[k]["ate_cm"] for x in ok]
                line += f"   {sum(a)/len(a):6.1f} ({min(a):5.1f}..{max(a):6.1f})" if a else "   -"
            print(line)


if __name__ == "__main__":
    main()
