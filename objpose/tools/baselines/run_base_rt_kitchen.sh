#!/usr/bin/env bash
# run_base_rt_kitchen.sh — FM-Fusion 과 DualMap 을 ObChange Kitchen(방문 5회)에 실시간 재생으로 돌리고 원래 이름으로 채점한다(run_base_rt_obchange.sh 와 같은 설정, CAD 이름 변형은 뺌).
#   그래픽 카드가 비어야 실시간 조건이 맞으므로 run_rt_obchange.sh 가 끝난 뒤(RT_OB_DONE) 시작한다. 결과 objpose/output/combo_obchange_261009/{fmfusion_rt,dualmap_rt,score_base_kitchen.json}, 기록 base_kitchen_log.txt (끝나면 KITCHEN_DONE)
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); O=$REPO/objpose/output/combo_obchange_261009; exec >> $O/base_kitchen_log.txt 2>&1
DPY=$HOME/anaconda3/envs/dualmap/bin/python; GPY=$HOME/DeepLearning/third_party/gsam_venv/bin/python
OB=$HOME/DeepLearning/Datasets/ObChange; T=objpose/tools/obchange; FD=$HOME/DeepLearning/Datasets/fmfusion_obchange_261005; TAG=combo_rt_261009
OBS=""; for n in 2 3 4 5 6; do OBS="$OBS kitchen_s$n"; done
log() { echo "=== $(date '+%F %T') $*"; }
until grep -aq RT_OB_DONE $REPO/objpose/output/rt_obchange_261011/log.txt 2>/dev/null; do sleep 20; done
export BENCH_ROOT=$OB/bop BENCH_TABLE=$REPO/objpose/rt/bop/obchange_objects.json BENCH_FPS=10 BENCH_SLAM=$OB/bop/poses_tf
log "FM-Fusion 검출 (실시간)"
for sc in $OBS; do rm -rf $FD/scans/$sc/prediction_gsam_rt; done
timeout 5400 $GPY objpose/tools/baselines/fmfusion/detect_gsam_prompts.py --data $FD/scans --gap 3 --all-objects --realtime 10 --out-name prediction_gsam_rt --scenes $OBS 2>&1 | grep -E "^[a-z]+_s[0-9]|Traceback|Error" | cut -c1-110
log "FM-Fusion 지도"
for sc in $OBS; do
  env -i HOME=$HOME PATH=/usr/bin:/bin LD_LIBRARY_PATH=$HOME/DeepLearning/third_party/install/lib timeout 1800 \
    $HOME/DeepLearning/third_party/FM-Fusion/build/src/IntegrateInstanceMap --config $REPO/$T/fmf_obchange_small.yaml --root $FD/scans/$sc \
    --prediction prediction_gsam_rt --frame_gap 3 --output $O/fmfusion_rt --verbose 2 > $O/fmfusion_rt/$sc.log 2>&1
  echo "$sc $(grep -o 'Save [0-9]* semantic instances' $O/fmfusion_rt/$sc.log) / 검출 프레임 $(python3 -c "import json;j=json.load(open('$FD/scans/$sc/prediction_gsam_rt/timing.json'));print(j['frames'],'/',j['offered'])")"
done
log "DualMap (실시간)"
DM_YOLO_CONF=0.05 timeout 7200 $DPY objpose/tools/baselines/dualmap/run_dualmap_bop.py --bop-test $OB/bop/test --table objpose/rt/bop/obchange_objects.json --poses gt --fps 10 --stride 2 \
  --out $O/dualmap_rt --tag $TAG --prompts $REPO/objpose/output/obchange_261005/dualmap_prompt_test/plain_prompts.json --timeout 1500 --realtime --scenes $OBS 2>&1 | grep -v -i warn | cut -c1-160
for sc in $OBS; do echo "$sc $(grep -a -o 'REALTIME processed.*' $O/dualmap_rt/${TAG}_in/$sc.log | tail -1)"; done
log "채점 (원래 이름)"
FMF_DIR=$O/fmfusion_rt DUALMAP_MAPS=$O/dualmap_rt/maps/$TAG timeout 1800 $DPY $T/inst_iou_obchange.py --scenes $OBS --tau 0.02 --out $O/score_base_kitchen.json 2>&1 | grep "FM-Fusion\|DualMap" | cut -c1-200
log "KITCHEN_DONE"
