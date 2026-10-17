#!/usr/bin/env bash
# FM-Fusion vs Object Pose Map (제안 / 원본 SAM-6D) YCB-V 비교를 처음부터 새로 돌린다. 2026-09-28.
#   1) ORB-SLAM3 RGB-D (ycbv_uw.yaml, 1500 특징) 12영상 — 세 방법이 같은 카메라 자세를 쓴다
#   2) FM-Fusion 입력 폴더 (새 SLAM 자세 + 데이터셋 물체 영역)
#   3) SAM-6D 실시간 4회: 제안·원본 × (인식 정확 가정 = 정답 상자 / 자체 검출). 벽시계 30 fps 라 한 번에 하나씩
#   4) FM-Fusion 검출(GroundingDINO + SAM, 같은 장면별 물체 문구) + 지도 두 조건
#   5) 채점(eval_maps.py)과 그림(render_maps.py)
# 사용: bash objpose/tools/baselines/fmfusion/rerun_all.sh [OUT]
set -u
cd "$(dirname "$0")/../../../.."
OUT=${1:-objpose/output/fmf_cmp_260928}
DATA=$HOME/DeepLearning/Datasets/fmfusion_ycbv_260928
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
GPY=$HOME/DeepLearning/third_party/gsam_venv/bin/python
HERE=objpose/tools/baselines/fmfusion
BIN=${YCBV_RGBD:?ycbv_rgbd 실행 파일 경로를 YCBV_RGBD 로 준다}
VOC=/home/jucpark/DeepLearning/CLI_environment_server/orbslam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt
YCBV=/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video/test
mkdir -p $OUT/slam
log() { echo "=== $(date +%T) $*"; }

log "1 ORB-SLAM3"
ls $YCBV | xargs -P 4 -I{} bash -c "n=\$(ls $YCBV/{}/rgb | wc -l); OMP_NUM_THREADS=2 $BIN $VOC objpose/tools/ycbv_slam/ycbv_uw.yaml $YCBV/{} \$n $OUT/slam/{} > $OUT/slam/{}.log 2>&1"
for s in $(ls $YCBV); do echo "$s $(wc -l < $OUT/slam/${s}_frames.txt)/$(ls $YCBV/$s/rgb | wc -l) tracked"; done
export BENCH_SLAM=$(pwd)/$OUT/slam

log "2 FM-Fusion input"
$PY $HERE/make_ycbv_scans.py --out $DATA 2>&1 | grep -v -i warn

log "3 SAM-6D realtime"
rc() { $PY objpose/tools/baselines/run_compare.py "$@" 2>&1 | grep -E --line-buffered "done|Traceback|Error" ; }
rc --method proposed --dataset ycbv --mode realtime --gt-boxes --out $OUT/proposed_gt
rc --method orig     --dataset ycbv --mode realtime --gt-boxes --out $OUT/orig_gt
rc --method proposed --dataset ycbv --mode realtime --out $OUT/proposed_own
rc --method orig     --dataset ycbv --mode realtime --out $OUT/orig_own
$PY - <<EOF
import json; from pathlib import Path
src, dst = Path("$OUT/orig_own"), Path("$OUT/orig_own_top1")   # 원본은 물체마다 후보를 여럿 낸다 → 물체별 최고점 1개 (eval_cmp.top1 과 같음)
for sc in sorted(p for p in src.iterdir() if (p / "sam6d_frames.jsonl").exists()):
    rows = []
    for l in (sc / "sam6d_frames.jsonl").read_text().splitlines():
        if not l.strip(): continue
        f = json.loads(l); best = {}
        for d in f["dets"]:
            if d["object"] not in best or d["score"] > best[d["object"]]["score"]: best[d["object"]] = d
        f["dets"] = list(best.values()); rows.append(json.dumps(f))
    (dst / sc.name).mkdir(parents=True, exist_ok=True); (dst / sc.name / "sam6d_frames.jsonl").write_text("\n".join(rows) + "\n")
EOF

log "4 FM-Fusion"
$GPY $HERE/detect_gsam_prompts.py --data $DATA/scans 2>&1 | grep -E "^[0-9]{6}: |Traceback|Error"
FMF_DATA=$DATA/scans bash $HERE/run_ycbv.sh $OUT/fmfusion_gt prediction_gt
FMF_DATA=$DATA/scans bash $HERE/run_ycbv.sh $OUT/fmfusion_own prediction_gsam

log "5 eval + maps"
$PY $HERE/eval_maps.py --fmf $OUT/fmfusion_gt --ours $OUT/proposed_gt --orig $OUT/orig_gt --out $OUT/eval_gt 2>&1 | grep -v -i warn
$PY $HERE/eval_maps.py --fmf $OUT/fmfusion_own --ours $OUT/proposed_own --orig $OUT/orig_own_top1 --out $OUT/eval_own 2>&1 | grep -v -i warn
$PY $HERE/render_maps.py --fmf $OUT/fmfusion_gt --ours $OUT/proposed_gt --orig $OUT/orig_gt --title "인식 정확 가정" --out $OUT/maps_gt 2>&1 | grep -v -i warn | tr '\n' ' '
$PY $HERE/render_maps.py --fmf $OUT/fmfusion_own --ours $OUT/proposed_own --orig $OUT/orig_own_top1 --title "자체 검출" --out $OUT/maps_own 2>&1 | grep -v -i warn | tr '\n' ' '
log "ALL DONE"
