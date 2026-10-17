#!/usr/bin/env bash
# run_room.sh <Room> — ObChange 한 방을 처음부터 끝까지: zip → (bag 하나씩 풀어) BOP 장면 → 정답 중심 →
#   제안 방법(실시간 10 fps) · DualMap(5 fps) · FM-Fusion(3.3 fps) 객체 지도 → ORB-SLAM3 자세 비교.
# 디스크를 아끼려고 bag 은 변환 뒤 지운다(zip 은 남긴다). 이미 끝난 단계는 건너뛴다.
#   Room = Kitchen | Office | LivingArea | SmallRoom | BigRoom
set -u
ROOM=${1:?}; room=$(echo "$ROOM" | tr 'A-Z' 'a-z')
REPO=$(cd "$(dirname "$0")/../../.." && pwd); cd "$REPO"
OB=$HOME/DeepLearning/Datasets/ObChange; O=$REPO/objpose/output/obchange_261005; T=objpose/tools/obchange
PY=$HOME/anaconda3/envs/sam6d/bin/python; DPY=$HOME/anaconda3/envs/dualmap/bin/python; GPY=$HOME/DeepLearning/third_party/gsam_venv/bin/python
FD=$HOME/DeepLearning/Datasets/fmfusion_obchange_261005
log() { echo "=== $(date '+%F %T') [$ROOM] $*"; }
mkdir -p $O/logs $O/slam_orb

log "1 bag → BOP"
for member in $(unzip -Z1 $OB/$ROOM.zip | grep '\.bag$' | sort -V); do
  n=$(basename $member .bag | grep -o '[0-9]*'); sc=${room}_s$n
  [ -f $OB/bop/poses_tf/${sc}_frames.txt ] && continue
  (cd $OB/raw && unzip -o -q ../$ROOM.zip "$member")
  python3 $T/make_bop.py --room $ROOM --raw $OB/raw/$(dirname $member) --step 3 --scenes $n 2>&1 | tail -1
  rm -f $OB/raw/$member
done
SCENES=$(ls $OB/bop/test | grep "^${room}_s" | sort -V | tr '\n' ' ')
log "scenes: $SCENES"
$DPY $T/gt_centers.py --room $ROOM 2>&1 | grep -v -i warn | cut -c1-160

log "2 제안 방법 (실시간)"
$PY objpose/tools/baselines/run_compare.py --method proposed --dataset obchange --mode realtime --all-objects --scenes $SCENES --out $O/proposed 2>&1 | grep -E "done:|skip|Traceback|Error"
BENCH_ROOT=$OB/bop BENCH_TABLE=$REPO/objpose/rt/bop/obchange_objects.json BENCH_FPS=10 \
  $PY $T/proposed_maps.py --run $O/proposed --out $O --tag proposed --room $room 2>&1 | grep -v -i "warn\|pynvml" | cut -c1-200

log "3 DualMap"
bash $T/run_dualmap.sh $room $O 2>&1 | cut -c1-200

log "4 FM-Fusion"
export BENCH_ROOT=$OB/bop BENCH_TABLE=$REPO/objpose/rt/bop/obchange_objects.json BENCH_FPS=10 BENCH_SLAM=$OB/bop/poses_tf
TODO=""; for sc in $SCENES; do [ -f $O/fmfusion/$sc/instance_info.txt ] || TODO="$TODO $sc"; done
if [ -n "$TODO" ]; then
  $PY objpose/tools/baselines/fmfusion/make_ycbv_scans.py --out $FD --scenes $TODO 2>&1 | grep -v -i warn | cut -c1-120
  $GPY objpose/tools/baselines/fmfusion/detect_gsam_prompts.py --data $FD/scans --gap 3 --all-objects --scenes $TODO 2>&1 | grep -E "^${room}|Traceback|Error" | cut -c1-110
  for sc in $TODO; do
    env -i HOME=$HOME PATH=/usr/bin:/bin LD_LIBRARY_PATH=$HOME/DeepLearning/third_party/install/lib \
      $HOME/DeepLearning/third_party/FM-Fusion/build/src/IntegrateInstanceMap --config $REPO/$T/fmf_obchange_small.yaml --root $FD/scans/$sc \
      --prediction prediction_gsam --frame_gap 3 --output $O/fmfusion --verbose 2 > $O/fmfusion/$sc.log 2>&1
    echo "$sc $(grep -o 'Save [0-9]* semantic instances' $O/fmfusion/$sc.log)"
  done
fi
python3 $T/fmfusion_to_maps.py $O/fmfusion $O/maps/fmfusion | grep "^${room}" | cut -c1-120

log "5 ORB-SLAM3 자세 비교"
VOC=$HOME/DeepLearning/CLI_environment_server/orbslam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt
for sc in $SCENES; do
  [ -s $O/slam_orb/${sc}_frames.txt ] || echo $sc
done | xargs -r -P 4 -I{} bash -c "n=\$(ls $OB/bop/test/{}/rgb | wc -l); OMP_NUM_THREADS=2 $O/ycbv_rgbd $VOC $T/orb_obchange_f3000.yaml $OB/bop/test/{} \$n $O/slam_orb/{} 10 > $O/slam_orb/{}.log 2>&1"
python3 $T/pose_analysis.py --run $O/proposed --orb $O/slam_orb --room $room --out $O/eval/pose_${room}.json 2>&1 | tail -1 | cut -c1-400
log "ROOM_DONE"
