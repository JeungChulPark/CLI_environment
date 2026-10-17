#!/usr/bin/env bash
# 2026-09-29: FM-Fusion vs Object Pose Map(제안) — YCB-M 전체 새 실행 + YCB-V 인식 정확 가정의 FM-Fusion 입력을 상자 기준으로 통일.
#   인식 정확 가정 = 두 방법 모두 데이터셋 정답 상자(bbox_visib, visib_fract>=0.1) → 각자 분할기(제안 MobileSAM, FM-Fusion SAM ViT-H)
#   자체 검출     = 제안 YOLO-World(+검증), FM-Fusion GroundingDINO + SAM (같은 장면별 물체 문구)
#   YCB-M: 32장면, 6 fps 재생. ORB-SLAM3 RGB-D 3000 특징(ycbm_xtion_f3000.yaml). FM-Fusion 은 2프레임마다 통합
#          (YCB-V 30 fps 의 10프레임 간격과 같은 초당 3회).
# 사용: YCBV_RGBD=<ycbv_rgbd 실행 파일> bash objpose/tools/baselines/fmfusion/rerun_ycbm.sh
set -u
cd "$(dirname "$0")/../../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
GPY=$HOME/DeepLearning/third_party/gsam_venv/bin/python
HERE=objpose/tools/baselines/fmfusion
BIN=${YCBV_RGBD:?}
VOC=/home/jucpark/DeepLearning/CLI_environment_server/orbslam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt
log() { echo "=== $(date +%T) $*"; }

log "A YCB-V: 정답 상자 → SAM (FM-Fusion 인식 정확 가정)"
V=objpose/output/fmf_cmp_260928; VD=$HOME/DeepLearning/Datasets/fmfusion_ycbv_260928
$GPY $HERE/make_gt_box_sam.py --data $VD/scans --gap 10 2>&1 | grep -E "^[0-9]{6}: |Traceback|Error"
FMF_DATA=$VD/scans bash $HERE/run_ycbv.sh $V/fmfusion_gtbox prediction_gtbox $HERE/ycbv_small.yaml 10
BENCH_SLAM=$(pwd)/$V/slam $PY $HERE/eval_maps.py --fmf $V/fmfusion_gtbox --ours $V/proposed_gt --orig $V/orig_gt --out $V/eval_gtbox 2>&1 | grep -v -i warn
BENCH_SLAM=$(pwd)/$V/slam $PY $HERE/render_maps.py --fmf $V/fmfusion_gtbox --ours $V/proposed_gt --orig $V/orig_gt --title "인식 정확 가정" --out $V/maps_gtbox 2>&1 | grep -v -i warn | tr '\n' ' '; echo

log "B YCB-M"
M=objpose/output/fmf_cmp_ycbm_260929; MD=$HOME/DeepLearning/Datasets/fmfusion_ycbm_260929
YCBM=/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_M/xtion_bop
mkdir -p $M/slam
export BENCH_ROOT=$YCBM BENCH_TABLE=$(pwd)/objpose/rt/bop/ycbm_objects.json BENCH_FPS=6 BENCH_SLAM=$(pwd)/$M/slam
log "B1 ORB-SLAM3"
ls $YCBM/test | xargs -P 8 -I{} bash -c "n=\$(ls $YCBM/test/{}/rgb | wc -l); OMP_NUM_THREADS=2 $BIN $VOC objpose/tools/ycbv_slam/ycbm_xtion_f3000.yaml $YCBM/test/{} \$n $M/slam/{} 6 > $M/slam/{}.log 2>&1"
for s in $(ls $YCBM/test); do echo -n "$s $(wc -l < $M/slam/${s}_frames.txt)/$(ls $YCBM/test/$s/rgb | wc -l)  "; done; echo
log "B2 FM-Fusion input"
$PY $HERE/make_ycbv_scans.py --out $MD 2>&1 | grep -v -i warn | tail -3
log "B3 제안 실시간 (6 fps)"
rc() { $PY objpose/tools/baselines/run_compare.py "$@" 2>&1 | grep -E --line-buffered "done|Traceback|Error" ; }
rc --method proposed --dataset ycbm --mode realtime --gt-boxes --out $M/proposed_gt
rc --method proposed --dataset ycbm --mode realtime --out $M/proposed_own
log "B4 FM-Fusion"
$GPY $HERE/make_gt_box_sam.py --data $MD/scans --gap 2 2>&1 | grep -E "Traceback|Error" ; echo gtbox done
$GPY $HERE/detect_gsam_prompts.py --data $MD/scans --gap 2 2>&1 | grep -E "Traceback|Error" ; echo gsam done
FMF_DATA=$MD/scans bash $HERE/run_ycbv.sh $M/fmfusion_gtbox prediction_gtbox $HERE/ycbm_small.yaml 2 | tail -2
FMF_DATA=$MD/scans bash $HERE/run_ycbv.sh $M/fmfusion_own prediction_gsam $HERE/ycbm_small.yaml 2 | tail -2
log "B5 eval + maps"
$PY $HERE/eval_maps.py --fmf $M/fmfusion_gtbox --ours $M/proposed_gt --out $M/eval_gtbox 2>&1 | grep -v -i warn
$PY $HERE/eval_maps.py --fmf $M/fmfusion_own --ours $M/proposed_own --out $M/eval_own 2>&1 | grep -v -i warn
$PY $HERE/render_maps.py --fmf $M/fmfusion_gtbox --ours $M/proposed_gt --title "인식 정확 가정" --out $M/maps_gtbox 2>&1 | grep -v -i warn | wc -l
$PY $HERE/render_maps.py --fmf $M/fmfusion_own --ours $M/proposed_own --title "자체 검출" --out $M/maps_own 2>&1 | grep -v -i warn | wc -l
log "ALL DONE"
