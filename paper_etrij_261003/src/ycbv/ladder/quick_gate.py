import json, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.argv = [sys.argv[0]]
exec(open(HERE / "quick_compare.py").read().split("levels = ")[0])
out = {}
for k, f in [("L1", "pred_ladder_L1"), ("L4", "pred_ladder_L4"), ("L5", "pred_ladder_L5"), ("A", "pred_ladder_A"),
             ("G1", "pred_gate_G1"), ("G2", "pred_gate_G2"), ("G3", "pred_gate_G3"), ("G123", "pred_gate_G123"),
             ("G123p", "pred_gate_G123_p")]:
    _, m = judge(json.load(open(Q / f"{f}.json")))
    out[k] = m; print(k, m)
json.dump(out, open(HERE / "quick_gate.json", "w"), indent=1)
