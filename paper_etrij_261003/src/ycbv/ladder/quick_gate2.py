import json, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
exec(open(HERE / "quick_compare.py").read().split("levels = ")[0])
for k in ["pred_ladder_L1", "pred_ladder_A_noverify", "pred_ladder_A", "pred_gate_A_G13_noverify", "pred_gate_A_G13",
          "pred_ladder_L3", "pred_ladder_L4", "pred_gate_G123_p"]:
    _, m = judge(json.load(open(Q / f"{k}.json"))); print(k, m)
