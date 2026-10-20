#!/bin/bash
# 2026-10-19 저녁 체인: 자이로 체인(run_gyro_lan41_all.sh, GPU 0 + .192 SLAM)이 끝나면 같은 자원으로 미사용 녹화 10개 체인을 돈다.
cd ~/Dataset/bop/ycbv_work
until grep -q CHAIN_ALL_DONE logs/gyro_lan41.log 2>/dev/null; do pgrep -f "run_gyro_lan41_al[l].sh" >/dev/null || { echo "gyro chain not running and not done; starting unused anyway $(date +%T)"; break; }; sleep 120; done
bash run_unused_lan41.sh
echo "[$(date +%T)] EVENING_ALL_DONE" | tee -a logs/unused_lan41.log
# 17:10 추가: bigeightcircle 자이로 끔 1회차(17:00~06)가 FM-Fusion 검출 시작과 겹쳐 인식 518프레임·물체 0개로 무효 → 미사용 체인 뒤 다시 돈다
CONFIG=orb SUF=_redo bash run_gyro_lan41.sh 260901_cbnu_bigeightcircle
echo "[$(date +%T)] REDO_DONE" | tee -a logs/gyro_lan41.log
