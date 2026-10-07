import json
from pathlib import Path
HERE = Path(__file__).resolve().parent
exec(open(HERE / "quick_compare.py").read().split("levels = ")[0])
for f in ['pred_ladder_L0', 'pred_ladder_L0_th05', 'pred_combo_L05_verify', 'pred_ladder_L1', 'pred_ladder_L1_th05',
          'pred_ladder_L1_th06', 'pred_ladder_L1_th07', 'pred_combo_S05_verify', 'pred_ladder_L4', 'pred_gate_G123_p']:
    _, m = judge(json.load(open(Q / f"{f}.json")))
    print(f"{f:24s} {m['time_median_ms']:6.0f} ms  found {m['found_pct']:5.1f}%  wrong {m['wrong_per_image']:.3f}  answers {m['answers']}  {m['stage_median_ms']}")
