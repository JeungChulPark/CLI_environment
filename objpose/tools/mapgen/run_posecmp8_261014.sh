#!/usr/bin/env bash
# run_posecmp8_261014.sh — 8 구성 프레임별 6D 포즈 비교(posecmp8.py): YCB-Video BOP19 시험 프레임 900장 → YCB-M 925장(estcmp_260927 과 같은 프레임).
#   결과 objpose/output/posecmp8_261014/{ycbv,ycbm}/<장면>.jsonl, 기록 log.txt, 끝나면 POSECMP8_DONE
set -u
cd "$(dirname "$0")/../../.."; O=objpose/output/posecmp8_261014; L=$O/log.txt
PY=$HOME/anaconda3/envs/sam6d/bin/python; export PYTHONPATH=$PWD/objpose/tools/shims_nvml
for ds in ycbv ycbm; do
  CMD="$PY objpose/tools/mapgen/posecmp8.py --table objpose/rt/bop/${ds}_objects.json --frames $O/frames_$ds.json --out $O/$ds"
  echo "=== $(date '+%F %T') start $ds :: $CMD" >> $L
  timeout 14400 $CMD >> $O/$ds.log 2>&1
  echo "=== $(date '+%F %T') end $ds exit=$? scenes=$(ls $O/$ds/*.done 2>/dev/null | wc -l)" >> $L
done
echo "=== $(date '+%F %T') POSECMP8_DONE" >> $L
