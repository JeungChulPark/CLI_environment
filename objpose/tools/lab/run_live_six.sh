#!/usr/bin/env bash
# run_live_six.sh — 8개 영상 × 여섯 구성을 실제 환경(노트북 ORB-SLAM3 실시간 + 이 PC 인식)으로 하나씩 돌린다. 끝난 실행은 건너뛴다.
#   결과 objpose/output/live_six_261010/c<구성>/<세션>/ (hub 실행 폴더: slam_poses.jsonl, kf_updates.jsonl, sam6d_frames.jsonl, sam6d/frames.jsonl, summary.json), 기록 log.txt (끝나면 LIVE_DONE)
set -u
cd "$(dirname "$0")/../../.."; O=$(pwd)/objpose/output/live_six_261010; mkdir -p $O; exec >> $O/log.txt 2>&1
A="260901_cbnu_bigeightcircle 260901_cbnu_digut 260901_cbnu_eightcircle 260901_cbnu_longcircle"; B="260826_etri_differentObject 260826_etri_eightcircle_dark 260826_etri_longcircle_dark 260826_etri_similarObject"
for group in "$A" "$B"; do for c in ${CONFIGS:-1 2 3 4 5 6}; do for s in $group; do
  [ -f $O/c$c/$s/summary.json ] && continue
  if [[ $s == 260826_* ]]; then until ssh -o BatchMode=yes samsung "test -f ~/DeepLearning/Dataset/$s/SLAM/SLAM_0.db3 && test -f ~/DeepLearning/Dataset/$s/info.json" 2>/dev/null && ! pgrep -f "rsync.*$s" >/dev/null; do sleep 20; done; fi
  echo "=== $(date '+%F %T') c$c $s"; bash objpose/tools/lab/run_live_one.sh $s $c $O/c$c/$s
  [ -f $O/c$c/$s/summary.json ] || { echo "재시도 c$c $s"; sleep 5; bash objpose/tools/lab/run_live_one.sh $s $c $O/c$c/$s; }
done; done; done
echo "=== $(date "+%F %T") ${DONE_MARK:-LIVE_DONE}"
