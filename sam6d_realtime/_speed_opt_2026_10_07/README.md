# Verification speed, 2026-10-07 (prototype, not applied to the pipeline)

Measured on 6 live frames (3 objects each, `frames/`), RTX 5090 Laptop on AC, config of
`objpose/output/live_loc-nav_261007_fix2/sam6d_config.yaml` (`fast` preset, verify on).

| per frame | now | with `alt_select.py` |
|---|---|---|
| PEM | 544 ms | 337 ms |
| total | 637 ms | 435 ms |
| `independent_candidate_verify` | 364 ms | 140 ms |
| `sequential_candidate_select` | 236 ms | 7 ms |
| texture `cdist` (1024 x 2048 per candidate) | 79 ms | 79 ms |
| `_candidate_shape_metrics` (2 calls) | 30 ms | 30 ms |

`sequential_candidate_select` builds a 300-row candidate table per object with one `.item()` per value (thousands of
GPU->CPU syncs per frame); `independent_candidate_verify` then drops that table unless analysis / dump_topk is on.
`alt_select.py` moves each tensor to the CPU once and builds the same rows: 24 calls, 116 objects (96 accepted),
selected index and every row field identical to the original (`ALT_COMPARE=1`).
The pipeline itself is not deterministic run to run (2.7 mm / 2.5 deg median between two unmodified runs).
`texture_chunk` 64 / 128 and `projection_chunk` 100 change nothing (compute-bound).

    python prof.py <sam6d_config.yaml> frames "" out.json                 # baseline
    ALT_COMPARE=1 python prof.py <cfg> frames alt_select.py out.json      # run both, compare
    python prof.py <cfg> frames alt_select.py out.json                    # timing with the fast version

## Applied 2026-10-12
`sequential_candidate_select` in `sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py` now is the fast version
(original file: `model_utils_before_261012.py`). `alt_check_applied.py` runs the original function text next to the in-file
one: real frames 24 calls / 116 objects / 34,800 candidate rows identical; 400 random cases reaching all six outcomes
(accepted with one / several cluster members, projection invalid, mask empty, texture empty, convergence insufficient) identical.

    python prof.py <cfg> frames alt_check_applied.py out.json
