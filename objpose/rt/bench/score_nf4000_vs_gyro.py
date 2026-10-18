#!/usr/bin/env python3
"""score_nf4000_vs_gyro.py — table of run_nf4000_vs_gyro.sh results (out_nf/<session>/map_eval.json + live_tum.txt.json).
Prints a Markdown table and writes nf4000_vs_gyro.csv next to this script."""
import csv, json, statistics as st
from pathlib import Path

HERE = Path(__file__).resolve().parent
SESSIONS = ["eightcircle", "digut", "longcircle", "bigeightcircle"]
SYSTEMS = ["nf4000", "nf2000_gyro"]
METRICS = [("loc_live", "ate_cm", "live ATE cm"), ("loc_live", "ate_max_cm", "live 최대 cm"),
           ("loc_live", "rot_max_deg", "live 최대 회전 °"), ("loc_live", "rpe1_cm", "live RPE1 cm"),
           ("loc_final", "ate_cm", "final ATE cm"), ("loc_final", "ate_max_cm", "final 최대 cm"),
           ("dense", "median_cm", "깊이영상 지도 중앙값 cm"), ("native", "points", "ORB 맵포인트 수")]

def ms(v):
    return f"{st.mean(v):.2f} ± {st.stdev(v):.2f}" if len(v) > 1 else f"{v[0]:.2f}"

rows = []
print("| 세션 | 지표 | NF4000 | NF2000+자이로 | 변화 |\n|---|---|---|---|---|")
for s in SESSIONS:
    d = HERE / "out_nf" / f"260901_cbnu_{s}"
    ev = json.loads((d / "map_eval.json").read_text())
    vals = {}
    for sys in SYSTEMS:
        reps = sorted(k for k in ev if k.startswith(sys + "_rep"))
        for sec, key, label in METRICS:
            vals[(sys, label)] = [ev[k][sec][key] for k in reps]
        runs = [json.loads((d / sys / k.split("_")[-1] / "live_tum.txt.json").read_text()) for k in reps]
        vals[(sys, "track 평균 ms")] = [r["track_ms"]["mean"] for r in runs]
        vals[(sys, "track p99 ms")] = [r["track_ms"]["p99"] for r in runs]
        vals[(sys, "33 ms 초과 프레임")] = [r["track_ms"]["over_33ms"] for r in runs]
        vals[(sys, "맵 수")] = [len(r["map_segments"]) for r in runs]
        vals[(sys, "OK 비율")] = [r["ok_ratio"] for r in runs]
    labels = [m[2] for m in METRICS] + ["track 평균 ms", "track p99 ms", "33 ms 초과 프레임", "맵 수", "OK 비율"]
    for i, lab in enumerate(labels):
        a, b = vals[("nf4000", lab)], vals[("nf2000_gyro", lab)]
        ma, mb = st.mean(a), st.mean(b)
        ch = f"{(mb - ma) / ma * 100:+.0f}%" if ma else "-"
        if "최대" in lab:  # per-run values: the max is set by one jump, so show all three
            fa, fb = " / ".join(f"{x:.1f}" for x in a), " / ".join(f"{x:.1f}" for x in b)
            ch = f"{(st.median(b) - st.median(a)) / st.median(a) * 100:+.0f}% (중앙값)"
        elif lab == "ORB 맵포인트 수":
            fa, fb = f"{ma:.0f}", f"{mb:.0f}"
        else:
            fa, fb = ms(a), ms(b)
        print(f"| {s if i == 0 else ''} | {lab} | {fa} | {fb} | {ch} |")
        rows.append({"session": s, "metric": lab, "nf4000": a, "nf2000_gyro": b, "nf4000_mean": ma, "nf2000_gyro_mean": mb})
with open(HERE / "nf4000_vs_gyro.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
