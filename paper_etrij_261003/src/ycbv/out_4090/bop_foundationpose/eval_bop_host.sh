#!/bin/bash
# host .3: official bop_toolkit (cea62d6, same commit as this PC) scoring of a BOP CSV. usage: eval_bop_host.sh <csv> <name> [num_workers]
set -e
CSV=$1; NAME=$2; NW=${3:-16}
source ~/anaconda3/etc/profile.d/conda.sh; conda activate bopeval
OUT=~/foundationpose/bop_results; TK=~/foundationpose/bop_toolkit
mkdir -p $OUT/eval; cp "$CSV" $OUT/${NAME}_ycbv-test.csv
cd $TK
BOP_PATH=~/foundationpose/bop_root BOP_RESULTS_PATH=$OUT BOP_EVAL_PATH=$OUT/eval PYTHONPATH=$TK PATH=$CONDA_PREFIX/bin:$PATH PYOPENGL_PLATFORM=egl \
  python scripts/eval_bop19_pose.py --renderer_type=vispy --result_filenames=${NAME}_ycbv-test.csv --num_workers=$NW > $OUT/eval_log_${NAME}.txt 2>&1 || { tail -30 $OUT/eval_log_${NAME}.txt; exit 1; }
tail -2 $OUT/eval_log_${NAME}.txt
cat $OUT/eval/${NAME}_ycbv-test/scores_bop19.json
