"""Choose prompts per object from the validation recall of every candidate (det_recall.py --candidates).

  best.json      method 1 (prompt re-selection): the candidate with the highest validation
                 recall; the paper prompt is kept unless another beats it by more than 1 point.
  ensemble.json  method 2 (prompt ensemble): the 3 best candidates, run together and merged.

    python select_prompts.py out/val_cand_m.json --tag m
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import paths as P  # noqa: E402


def main():
    src, tag = sys.argv[1], sys.argv[sys.argv.index("--tag") + 1] if "--tag" in sys.argv else "m"
    cand = json.load(open(src))["candidates"]
    best, ens, table = {}, {}, {}
    for o in range(1, 22):
        c = {p: r for p, r in cand[str(o)].items() if r is not None}
        orig = P.PROMPTS[o - 1]
        rank = sorted(c, key=lambda p: -c[p])
        b = rank[0] if c[rank[0]] > c.get(orig, -1) + 1.0 else orig
        best[o] = b
        ens[o] = rank[:3]
        table[P.YCB_NAMES[o - 1]] = {"paper": [orig, c.get(orig)], "best": [b, c[b]], "ensemble": ens[o],
                                     "all": c}
    out = HERE / "out"
    json.dump(best, open(out / f"prompts_best_{tag}.json", "w"), indent=1)
    json.dump(ens, open(out / f"prompts_ensemble_{tag}.json", "w"), indent=1)
    json.dump(table, open(out / f"prompt_selection_{tag}.json", "w"), indent=1, ensure_ascii=False)
    for k, v in table.items():
        print(f"{k:24s} {v['paper'][0]!r:26s} {v['paper'][1]:5}  ->  {v['best'][0]!r:26s} {v['best'][1]:5}   ens {v['ensemble']}")


if __name__ == "__main__":
    main()
