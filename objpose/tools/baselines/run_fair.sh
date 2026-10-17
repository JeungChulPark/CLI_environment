#!/usr/bin/env bash
# 2026-09-25 공정 비교 재실행: 두 방법 모두 정답 상자(GT box)를 받는다 — 검출 조건을 같게 두고 자세 추정·검증·지도만 비교.
#   realtime  YCB-V 12영상 30 fps, 최신 프레임만 (원본 SAM-6D = GT 상자 → MobileSAM → PEM, 제안 = GT 상자 → 후보 검증 → PEM → 자세 검증)
#   bop19     YCB-V · LM-O BOP-19 테스트 이미지, BOP AR (제안은 검증 통과만 답 / 검증을 점수로 두 가지)
# 한 번에 하나씩 GPU를 쓴다 (실시간 처리량이 서로 영향받지 않게).
set -u
cd "$(dirname "$0")/../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
R=objpose/tools/baselines/run_compare.py
O=objpose/output/cmp_260925_fair
mkdir -p $O
run() { echo "=== $(date +%T) $*"; $PY $R --gt-boxes "$@" 2>&1 | grep -E --line-buffered "done:|Traceback|Error"; }
run --method orig     --dataset ycbv --mode realtime --out $O/orig_ycbv_realtime
run --method proposed --dataset ycbv --mode realtime --out $O/proposed_ycbv_realtime
for ds in ycbv lmo; do
  run --method orig           --dataset $ds --mode bop19 --out $O/orig_${ds}_bop19
  run --method proposed       --dataset $ds --mode bop19 --out $O/proposed_${ds}_bop19
  run --method proposed_score --dataset $ds --mode bop19 --out $O/proposedscore_${ds}_bop19
done
echo "=== $(date +%T) ALL DONE"
