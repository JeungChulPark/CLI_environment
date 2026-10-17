#!/usr/bin/env bash
# the rest of the original SAM-6D all-frame runs (YCB-M, then YCB-Video), continuing from the frame where the last
# run stopped (run_compare.py --resume). A run that dies (GPU out of memory, ...) is started again, up to 5 times.
#   bash objpose/tools/baselines/run_orig_rest.sh            # until both are finished
#   bash objpose/tools/baselines/run_orig_rest.sh "07:00"    # stop cleanly at that clock time; call again to continue
set -u
cd "$(dirname "$0")/../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
R=objpose/tools/baselines/run_profiled.py
O=objpose/output/allframes_261006
L=$O/logs/main.log
STOP=0; [ -n "${1:-}" ] && STOP=$(date -d "$1" +%s)
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
for ds in ycbm ycbv; do
  total=$(ls -d $($PY -c "import json;print(json.load(open('objpose/rt/bop/${ds}_objects.json'))['scenes_dir'])")/*/ | wc -l)
  for try in 1 2 3 4 5; do
    [ "$(ls $O/${ds}_orig/*/run.json 2>/dev/null | wc -l)" = "$total" ] && break
    [ $STOP != 0 ] && [ $(date +%s) -ge $STOP ] && break
    echo "=== $(date '+%m-%d %T') start orig $ds (try $try, until ${1:-the end})" >> $L
    $PY $R --method orig --dataset $ds --mode still --out $O/${ds}_orig --resume --stop-at $STOP >> $O/logs/${ds}_orig.log 2>&1
    echo "=== $(date '+%m-%d %T') orig $ds exit=$?: $(cat $O/${ds}_orig/*/sam6d_frames.jsonl | wc -l) frames, $(ls $O/${ds}_orig/*/run.json 2>/dev/null | wc -l) / $total scenes finished" >> $L
  done
done
touch $O/DONE
