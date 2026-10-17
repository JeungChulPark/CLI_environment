#!/usr/bin/env bash
# run_base_rt_obchange.sh — FM-Fusion 과 DualMap 을 ObChange 3개 방(방문 15회)에 실시간 재생으로 다시 돌리고, 원래 이름과 CAD 이름(cad_relabel.py) 두 가지로 채점한다.
#   FM-Fusion  검출(GroundingDINO + SAM)이 끝난 순간 도착해 있는 최신 프레임만 처리(detect_gsam_prompts.py --realtime) → 그 프레임들만 지도에 통합
#   DualMap    5 Hz 로 도착하는 프레임 중 처리가 밀리면 최신 것만(run_dualmap_bop.py --realtime). 문구·기준값은 기존과 같음(일반 이름, 0.05)
# run_combo_obchange.sh 가 끝난 뒤(COMBO_DONE) 시작한다. 결과 objpose/output/combo_obchange_261009/{fmfusion_rt,dualmap_rt,cad_rt,score_base_<방>.json,score_cad_<방>.json}, 기록 base_log.txt (끝나면 BASE_DONE)
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); O=$REPO/objpose/output/combo_obchange_261009; mkdir -p $O; exec >> $O/base_log.txt 2>&1
PY=$HOME/anaconda3/envs/sam6d/bin/python; DPY=$HOME/anaconda3/envs/dualmap/bin/python; GPY=$HOME/DeepLearning/third_party/gsam_venv/bin/python
OB=$HOME/DeepLearning/Datasets/ObChange; T=objpose/tools/obchange; FD=$HOME/DeepLearning/Datasets/fmfusion_obchange_261005; TAG=combo_rt_261009
ROOMS="office livingarea smallroom"; OBS=""; for r in $ROOMS; do for n in 2 3 4 5 6; do OBS="$OBS ${r}_s$n"; done; done
log() { echo "=== $(date '+%F %T') $*"; }
until grep -aq COMBO_DONE $O/log.txt 2>/dev/null; do sleep 20; done
export BENCH_ROOT=$OB/bop BENCH_TABLE=$REPO/objpose/rt/bop/obchange_objects.json BENCH_FPS=10 BENCH_SLAM=$OB/bop/poses_tf

log "FM-Fusion 검출 (실시간)"
for sc in $OBS; do rm -rf $FD/scans/$sc/prediction_gsam_rt; done
timeout 5400 $GPY objpose/tools/baselines/fmfusion/detect_gsam_prompts.py --data $FD/scans --gap 3 --all-objects --realtime 10 --out-name prediction_gsam_rt --scenes $OBS 2>&1 | grep -E "^[a-z]+_s[0-9]|Traceback|Error" | cut -c1-110
log "FM-Fusion 지도"
mkdir -p $O/fmfusion_rt
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
for r in $ROOMS; do
  FMF_DIR=$O/fmfusion_rt DUALMAP_MAPS=$O/dualmap_rt/maps/$TAG timeout 1800 $DPY $T/inst_iou_obchange.py --scenes ${r}_s2 ${r}_s3 ${r}_s4 ${r}_s5 ${r}_s6 --tau 0.02 --out $O/score_base_$r.json 2>&1 | grep "FM-Fusion\|DualMap" | cut -c1-200
done
log "CAD 이름 (실시간)"
unset BENCH_ROOT BENCH_TABLE BENCH_FPS BENCH_SLAM
timeout 9000 $PY objpose/tools/analysis/cad_relabel.py --data obchange --out $O/cad_rt --gap FM-Fusion=3 DualMap=2 --fmf-dir $O/fmfusion_rt --dualmap-maps $O/dualmap_rt/maps/$TAG --scenes $OBS 2>&1 | grep -E "^[a-z]+_s[0-9] |Traceback|Error" | cut -c1-260
log "채점 (CAD 이름)"
for r in $ROOMS; do
  FMF_DIR=$O/fmfusion_rt FMF_NAMES=$O/cad_rt/fmf_names.json DUALMAP_MAPS=$O/cad_rt/dualmap timeout 1800 $DPY $T/inst_iou_obchange.py --scenes ${r}_s2 ${r}_s3 ${r}_s4 ${r}_s5 ${r}_s6 --tau 0.02 --out $O/score_cad_$r.json 2>&1 | grep "FM-Fusion\|DualMap" | cut -c1-200
done
log "BASE_DONE"
