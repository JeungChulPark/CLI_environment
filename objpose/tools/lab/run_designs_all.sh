#!/usr/bin/env bash
# run_designs_all.sh — 끝난 실제 환경 실행(objpose/output/live_six_261010/c<구성>/<세션>)마다 지도 설계 네 가지(map_designs.py)를 만든다. 이미 만든 것은 건너뛴다(-f 로 다시).
cd "$(dirname "$0")/../../.."; O=objpose/output/live_six_261010; mkdir -p $O/designs; PY=$HOME/anaconda3/envs/sam6d/bin/python; export PYTHONPATH=$(pwd)/objpose/tools/shims_nvml
for d in $O/c[1-8]/26*/; do [ -f $d/summary.json ] || continue; s=$(basename $d); c=$(basename $(dirname $d)); f=$O/designs/${c}_$s.json
  [ -f $f ] && [ "${1:-}" != "-f" ] && continue
  echo "== $c $s"; nice -n 10 $PY objpose/tools/lab/map_designs.py $d --session $s --out $f 2>&1 | grep -E "^V[0-3]:|^\{|Traceback|Error" | cut -c1-200
done
