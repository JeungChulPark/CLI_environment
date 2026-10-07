"""Pose-verification cost study (det_study/run_verify_cost.sh) on the 36 images.
Each run against its reference: the improved recogniser (pred_gate_G123_p, 300 candidates,
stride 1, fp32) or, for the L4 row, the deployed recogniser (pred_ladder_L4).
lost / gained = (image, object) cases correct in the reference but not here / the other way."""
import json
from pathlib import Path
HERE = Path(__file__).resolve().parent
exec(open(HERE / "quick_compare.py").read().split("levels = ")[0])
runs = [("pred_gate_G123_p", None), ("pred_vc_top100", "pred_gate_G123_p"), ("pred_vc_top50", "pred_gate_G123_p"),
        ("pred_vc_top30", "pred_gate_G123_p"), ("pred_vc_top10", "pred_gate_G123_p"),
        ("pred_vc_stride3", "pred_gate_G123_p"), ("pred_vc_stride10", "pred_gate_G123_p"),
        ("pred_vc_fp16", "pred_gate_G123_p"), ("pred_vc_bf16", "pred_gate_G123_p"),
        ("pred_vc_top30_bf16", "pred_gate_G123_p"), ("pred_vk_cand100", "pred_gate_G123_p"), ("pred_vk_cand50", "pred_gate_G123_p"),
        ("pred_vk_cand30", "pred_gate_G123_p"), ("pred_vk_cand10", "pred_gate_G123_p"),
        ("pred_ladder_L4", None), ("pred_vc_L4_top30", "pred_ladder_L4"), ("pred_vk_L4_cand30", "pred_ladder_L4"),
        ("pred_vk_fp16", "pred_gate_G123_p"), ("pred_vk_bf16", "pred_gate_G123_p")]
cache, out = {}, {}
for f, ref in runs:
    if not (Q / f"{f}.json").exists():
        print(f"{f:22s} (missing)"); continue
    cache[f] = judge(json.load(open(Q / f"{f}.json")))
    ok, m = cache[f]
    if ref and ref in cache:
        rok = cache[ref][0]
        m["lost"] = sum(1 for k in rok if rok[k][1] and not ok.get(k, (0, 0))[1])
        m["gained"] = sum(1 for k in rok if not rok[k][1] and ok.get(k, (0, 0))[1])
    out[f] = m
    print(f"{f:22s} {m['time_median_ms']:6.0f} ms  pem {m['stage_median_ms'].get('pem', 0):6.0f}  found {m['found_pct']:5.1f}%  "
          f"wrong {m['wrong_per_image']:.3f}  answers {m['answers']:3d}  lost {m.get('lost', '-')}  gained {m.get('gained', '-')}")
json.dump(out, open(Q / "quick_vc.json", "w"), indent=1)
