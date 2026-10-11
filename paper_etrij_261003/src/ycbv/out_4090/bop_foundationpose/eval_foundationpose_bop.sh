#!/bin/bash
# 이 PC: FoundationPose CSV 를 bop_toolkit(bop_venv)으로 채점 (eval_gigapose_bop.sh 와 동일). 사용: eval_foundationpose_bop.sh <csv> [name]
set -e
CSV=$1; NAME=${2:-foundationpose}
OUT=/home/jucpark/DeepLearning/server/CLI_environment/paper_etrij_261003/src/ycbv/out_4090/bop_foundationpose
W=$HOME/Dataset/bop/ycbv_work; TK=$W/bop_toolkit; PY=$W/bop_venv/bin/python
mkdir -p $OUT/eval; [ "$(readlink -f "$CSV")" = "$OUT/${NAME}_ycbv-test.csv" ] || cp "$CSV" $OUT/${NAME}_ycbv-test.csv
cd $TK
CUDA_VISIBLE_DEVICES="" BOP_PATH=$W/bop_root BOP_RESULTS_PATH=$OUT BOP_EVAL_PATH=$OUT/eval PYTHONPATH=$TK PATH=$W/bop_venv/bin:$PATH LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
  $PY scripts/eval_bop19_pose.py --renderer_type=vispy --result_filenames=${NAME}_ycbv-test.csv --num_workers=48 > $OUT/eval_log_${NAME}.txt 2>&1
tail -2 $OUT/eval_log_${NAME}.txt
cat $OUT/eval/${NAME}_ycbv-test/scores_bop19.json
