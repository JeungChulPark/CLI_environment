#!/usr/bin/env python3
"""table.py — one line per session from out/<session>/summary.json (+ out/rescore_<session>.json if present)."""
import json, sys
from pathlib import Path
import numpy as np

OUT = Path(__file__).resolve().parent / "out"
print("| 세션 | 경로 m / 시간 s | 루프 수락/후보 | ICP 정·역 불일치 cm | KISS 루프 오차 med/p90/max cm | pseudo-GT 보류 루프 med/p90/max cm | 보류 루프 ° (KISS→PGO) p90 | 시작·끝 보류 cm (KISS→PGO) | PGO−KISS med/max cm | 다른 회차 평면 잔차 med/p90 cm (KISS→PGO, 같은 회차) | 중력 기울기 ° 보정후 med/p90 (KISS→PGO) |")
print("|---|---|---|---|---|---|---|---|---|---|---|")
for s in sorted(OUT.glob("*/summary.json")):
    j = json.loads(s.read_text()); n = s.parent.name
    L, H = j["loops"], j.get("heldout_loops", {})
    se = j.get("start_end_heldout_cm", {})
    po, pp = j.get("planes_odom", {}), j.get("planes_pgo", {})
    tg = j.get("tilt_vs_gravity", {})
    tilt = (f"{tg['odom']['cal']['median_deg']:.2f}/{tg['odom']['cal']['p90_deg']:.2f} → {tg['pgo']['cal']['median_deg']:.2f}/{tg['pgo']['cal']['p90_deg']:.2f}"
            if tg and "cal" in tg.get("odom", {}) else "–")
    print(f"| {n} | {j['path_m']:.1f} / {j['duration_s']:.0f} | {L['accepted']}/{L['candidates']} | {L['fwbw_cm_median']:.2f} | "
          f"{H['odom_cm']['median']:.2f}/{H['odom_cm']['p90']:.2f}/{H['odom_cm']['max']:.1f} | "
          f"{H['pgo_heldout_cm']['median']:.2f}/{H['pgo_heldout_cm']['p90']:.2f}/{H['pgo_heldout_cm']['max']:.1f} | "
          f"{H['odom_deg']['p90']:.2f} → {H['pgo_heldout_deg']['p90']:.2f} | "
          f"{(str(se['odom_median']) + ' → ' + str(se['pgo_median'])) if se else '–'} | "
          f"{j['pgo_minus_odom_cm']['median']:.1f}/{j['pgo_minus_odom_cm']['max']:.1f} | "
          f"{po['other_pass']['median_cm']:.2f}/{po['other_pass']['p90_cm']:.2f} → {pp['other_pass']['median_cm']:.2f}/{pp['other_pass']['p90_cm']:.2f} "
          f"({pp['same_pass']['median_cm']:.2f}/{pp['same_pass']['p90_cm']:.2f}) | {tilt} |")
