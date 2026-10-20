#!/bin/bash
# 2026-10-19 20:05 세 번째 시작(공유메모리 접미사 _g0) (GPU 0 + .192 SLAM). 처음 체인(run_gyro_lan41_all.sh → run_evening_41.sh)은 자이로 스크립트의
# 정리 단계가 머신의 모든 hub 를 죽여 방 체인을 망가뜨리는 것을 발견해 17:55 에 멈췄다(스크립트는 자기 hub 만 끝내도록 고침).
# 끝난 실행은 건너뛰므로 처음부터 다시 적어도 된다: 30 문서 세 구성 1회 → 반복 2·3회 → 미사용 녹화 10개 → bigeightcircle 자이로 끔 재실행.
cd ~/Dataset/bop/ycbv_work
for c in gyro orb orig; do CONFIG=$c bash run_gyro_lan41.sh; done
for s in _r2 _r3; do for c in gyro orb; do SUF=$s CONFIG=$c bash run_gyro_lan41.sh; done; done
echo "[$(date +%T)] CHAIN_ALL_DONE" | tee -a logs/gyro_lan41.log
bash run_unused_lan41.sh
echo "[$(date +%T)] EVENING_ALL_DONE" | tee -a logs/unused_lan41.log
CONFIG=orb SUF=_redo bash run_gyro_lan41.sh 260901_cbnu_bigeightcircle
echo "[$(date +%T)] REDO_DONE" | tee -a logs/gyro_lan41.log
# 19:47 추가: outlong_middle 원본 0.5 의 루프 보정 344.7 cm(3 m 넘음, 29 문서 폐기 규칙) → 실행을 옆으로 치우고 한 번 더
O=$HOME/DeepLearning/CLI_environment/objpose/output/live_261019_260915_outlong_middle_41lan_origL05; mv $O ${O}_bigloop345; mv $O.log ${O}_bigloop345.log
bash run_unused_lan41.sh 260915_outlong_middle
echo "[$(date +%T)] REDO2_DONE" | tee -a logs/unused_lan41.log
