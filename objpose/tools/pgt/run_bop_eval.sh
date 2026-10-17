#!/usr/bin/env bash
# official BOP-19 AR (VSD / MSSD / MSPD) for the 9 estimator-comparison CSVs
set -u
BL="$(cd "$(dirname "$0")/../baselines" && pwd)"
export PYTHONPATH=$BL/bop_toolkit PATH=$HOME/anaconda3/envs/bop/bin:$PATH BOP_PATH=$BL/bop_root BOP_RESULTS_PATH=$BL/bop_results BOP_EVAL_PATH=$BL/bop_eval
cd $BL/bop_toolkit
for ds in ${DSETS:-ycbm real}; do for m in proposed gtbox orig; do
  f=$m-estcmp_$ds-test.csv
  echo "=== $(date +%T) $f"
  python scripts/eval_bop19_pose.py --result_filenames=$f --renderer_type=vispy --num_workers=8 2>&1 | grep -E "bop19_average|Error|Traceback|error" | tail -8
done; done
echo "=== $(date +%T) BOP DONE"
