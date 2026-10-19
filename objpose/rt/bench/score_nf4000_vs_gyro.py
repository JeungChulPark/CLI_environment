#!/usr/bin/env python3
"""score_nf4000_vs_gyro.py — table of run_nf4000_vs_gyro.sh results (out_nf/<session>/map_eval.json + live_tum.txt.json).
Prints a Markdown table (mean ± std over reps; per-run values for max errors; change vs NF4000) and writes
nf4000_vs_gyro.csv next to this script. Systems missing from a session's map_eval.json are skipped."""
import csv, json, statistics as st
from pathlib import Path

HERE = Path(__file__).resolve().parent
SESSIONS = ["eightcircle", "digut", "longcircle", "bigeightcircle"]
SYSTEMS = [("nf4000", "NF4000"), ("nf2000", "NF2000"), ("nf2000_gyro", "NF2000+자이로"), ("nf4000_gyro", "NF4000+자이로")]
METRICS = [("loc_live", "ate_cm", "live ATE cm"), ("loc_live", "ate_max_cm", "live 최대 cm"),
           ("loc_live", "rot_max_deg", "live 최대 회전 °"), ("loc_live", "rpe1_cm", "live RPE1 cm"),
           ("loc_final", "ate_cm", "final ATE cm"), ("loc_final", "ate_max_cm", "final 최대 cm"),
           ("dense", "median_cm", "깊이영상 지도 중앙값 cm"), ("native", "points", "ORB 맵포인트 수")]
RUNINFO = [("track 평균 ms", lambda r: r["track_ms"]["mean"]), ("track p99 ms", lambda r: r["track_ms"]["p99"]),
           ("33 ms 초과 프레임", lambda r: r["track_ms"]["over_33ms"]), ("맵 수", lambda r: len(r["map_segments"])),
           ("OK 비율", lambda r: r["ok_ratio"])]


def fmt(lab, v):
    if "최대" in lab or "초과" in lab:  # per-run values: one jump sets the max, so show every run
        return " / ".join(f"{x:.1f}" if isinstance(x, float) else str(x) for x in v)
    if lab == "ORB 맵포인트 수":
        return f"{st.mean(v):.0f}"
    return f"{st.mean(v):.2f} ± {st.stdev(v):.2f}" if len(v) > 1 else f"{v[0]:.2f}"


def change(lab, base, v):
    f = st.median if "최대" in lab else st.mean
    return f"{(f(v) - f(base)) / f(base) * 100:+.0f}%" if f(base) else "-"


rows, systems = [], None
for s in SESSIONS:
    d = HERE / "out_nf" / f"260901_cbnu_{s}"
    ev = json.loads((d / "map_eval.json").read_text())
    present = [x for x in SYSTEMS if any(k.startswith(x[0] + "_rep") for k in ev)]
    if systems is None:
        systems = present
        others = [x for x in systems if x[0] != "nf4000"]
        print("| 세션 | 지표 | " + " | ".join(n for _, n in systems) + " | "
              + " | ".join(f"{n} 변화" for _, n in others) + " |")
        print("|---" * (2 + len(systems) + len(others)) + "|")
    vals = {}
    for sys, _ in systems:
        reps = sorted(k for k in ev if k.startswith(sys + "_rep"))
        for sec, key, lab in METRICS:
            vals[(sys, lab)] = [ev[k][sec][key] for k in reps]
        runs = [json.loads((d / sys / k.split("_")[-1] / "live_tum.txt.json").read_text()) for k in reps]
        for lab, f in RUNINFO:
            vals[(sys, lab)] = [f(r) for r in runs]
    labels = [m[2] for m in METRICS] + [r[0] for r in RUNINFO]
    for i, lab in enumerate(labels):
        base = vals[("nf4000", lab)]
        cells = [fmt(lab, vals[(sys, lab)]) for sys, _ in systems]
        chg = [change(lab, base, vals[(sys, lab)]) + (" (중앙값)" if "최대" in lab else "") for sys, _ in others]
        print(f"| {s if i == 0 else ''} | {lab} | " + " | ".join(cells) + " | " + " | ".join(chg) + " |")
        row = {"session": s, "metric": lab}
        for sys, _ in systems:
            row[sys] = vals[(sys, lab)]; row[sys + "_mean"] = st.mean(vals[(sys, lab)])
        rows.append(row)
with open(HERE / "nf4000_vs_gyro.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
