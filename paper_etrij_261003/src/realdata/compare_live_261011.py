"""24 문서: 23 문서 제안 방법 실행(objmap/common.REAL_SESSIONS) 대 2026-10-11 재실행(live_261011_*_l3080_hyb3cf)을
objmap/real_score.py 와 같은 기준(eval_real 기준 위치, 반경 20 cm)으로 비교 -> results_real5_live_261011.json."""
import json, sys
from pathlib import Path
import numpy as np
SRC = Path(__file__).resolve().parent.parent / "objmap"
sys.path.insert(0, str(SRC))
from common import REAL_SESSIONS
from real_score import reference, score, REPO
rows = {}
for s, (ses, old) in sorted(REAL_SESSIONS.items()):
    er, ref = reference(ses)
    row = {}
    runs = [("23", old), ("new", f"live_261011_{ses}_l3080_hyb3cf")]
    if s == 1: runs.append(("new_r2", f"live_261011_{ses}_l3080_hyb3cf_r2"))
    for tag, run in runs:
        d = REPO / "objpose/output" / run
        if not (d / "summary.json").exists():
            row[tag] = None; continue
        o = er.run_one(d, ref)
        S = json.load(open(d / "summary.json"))
        fin = [(n.split("#")[0], np.asarray(v["T_w_obj"], float).reshape(4, 4)[:3, 3]) for n, v in S.get("objects", {}).items()]
        r = score(fin, ref, er.judge, er.RADIUS)
        dist = {}
        for n, p in fin:
            if n in ref:
                dd = float(np.linalg.norm(p - ref[n])); dist[n] = round(min(dd, dist.get(n, 9e9)) * 100, 1)
        row[tag] = dict(run=run, correct=r["correct"], wrong=r["wrong"], score_pct=r["score_pct"], missed=r["missed"],
                        wrong_kinds=r["wrong_kinds"], rate_hz=o["rate_hz"], mean_score_pct=o["mean_score_pct"],
                        dist_cm=dist, kf_max_cm=S.get("kf_max_correction_cm"), sam6d_frames=S.get("sam6d_frames"),
                        slam_dropped=S.get("slam_dropped"), loops=S.get("slam_loop_events"), track_ms=S.get("slam_track_ms"))
    rows[ses] = row
    f = lambda r: "없음" if r is None else f"{r['correct']}/{r['wrong']} {r['score_pct']}% {r['rate_hz']}Hz 평균{r['mean_score_pct']}% 루프{r['kf_max_cm']}cm 놓침{r['missed']}"
    print(s, ses, " | ".join(f"{t}: {f(row[t])}" for t in row))
json.dump(rows, open(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).parent / "results_real5_live_261011.json", "w"), indent=1, ensure_ascii=False)
