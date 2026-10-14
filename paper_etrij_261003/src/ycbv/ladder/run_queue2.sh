#!/bin/bash
# L2 rerun (single-proposal fix), option B (L1 + ISM threshold), option A (FastSAM boxes + our ISM)
set -u
cd "$(dirname "$0")/.."; PY=$HOME/anaconda3/envs/sam6d/bin/python
$PY ladder/run_ladder.py --level L2 || echo "FAIL L2"
$PY ladder/run_ladder.py --level L1 --thresh 0.4 || echo "FAIL B04"
$PY ladder/run_ladder.py --level L1 --thresh 0.5 || echo "FAIL B05"
$PY run_ours.py --mode text --proposer fastsam --top-k 200 --no-verify --out out/pred_ladder_A_noverify.json || echo "FAIL A0"
$PY run_ours.py --mode text --proposer fastsam --top-k 200 --out out/pred_ladder_A.json || echo "FAIL A"
$PY evaluate.py --results ladder/results_ladder.json --methods ladder_L0,ladder_L1,ladder_L2,ladder_L3,ladder_L4,ladder_L5,ladder_L1_th04,ladder_L1_th05,ladder_A_noverify,ladder_A
echo QUEUE2_DONE
