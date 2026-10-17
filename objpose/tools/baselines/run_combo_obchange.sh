#!/usr/bin/env bash
# run_combo_obchange.sh — 조합 실험의 핵심 구성 7개를 ObChange 3개 방(Office, LivingArea, SmallRoom, 정답 있는 방문 15회)에 실시간 재생으로 돌리고,
# 구성마다 객체 지도(방문별)를 만들어 채점한다. 구성 설정은 조합 보고서(objpose/docs/ycb_combo_261008)와 같다(combo_rt.py 가 스크립트를 그대로 부른다).
#   결과 objpose/output/combo_obchange_261009/<구성>/{<방문>/sam6d_frames.jsonl, maps/proposed/<방문>.json, score_<방>.json}, 진행 기록 log.txt (끝나면 COMBO_DONE)
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); O=$REPO/objpose/output/combo_obchange_261009; mkdir -p $O; exec >> $O/log.txt 2>&1
PY=$HOME/anaconda3/envs/sam6d/bin/python; DPY=$HOME/anaconda3/envs/dualmap/bin/python; OB=$HOME/DeepLearning/Datasets/ObChange; T=objpose/tools/obchange
Y=/home/jucpark/DeepLearning/CLI_env_paper/paper_etrij_261003/src/ycbv; PR="text:yolov8m-worldv2.pt:$Y/det_study/out/prompts_best_m.json"
ROOMS="office livingarea smallroom"; OBS=""; for r in $ROOMS; do for n in 2 3 4 5 6; do OBS="$OBS ${r}_s$n"; done; done
log() { echo "=== $(date '+%F %T') $*"; }
one() { id=$1; shift
  [ -f $O/$id/score_smallroom.json ] && { log "$id 이미 끝남"; return; }
  log "$id 시작: $*"; rm -rf $O/$id; mkdir -p $O/$id
  timeout 4500 $PY objpose/tools/baselines/combo_rt.py --out $O/$id --scenes $OBS -- "$@" > $O/$id/run.log 2>&1; echo "exit=$? $(grep -a -c '^\[rt\] .*frames' $O/$id/run.log) 방문"
  for r in $ROOMS; do
    BENCH_ROOT=$OB/bop BENCH_TABLE=$REPO/objpose/rt/bop/obchange_objects.json BENCH_FPS=10 timeout 1800 $PY $T/proposed_maps.py --run $O/$id --out $O/$id --tag proposed --room $r --no-online > $O/$id/maps_$r.log 2>&1
    PROPOSED_MAPS=$O/$id/maps/proposed timeout 1800 $DPY $T/inst_iou_obchange.py --scenes ${r}_s2 ${r}_s3 ${r}_s4 ${r}_s5 ${r}_s6 --tau 0.02 --out $O/$id/score_$r.json 2>&1 | grep "제안 방법" | cut -c1-200
  done
  log "$id 끝"; }
one E_on  run_ours.py --mode text
one E_off run_ours.py --mode text --no-verify
one C_on  run_ours.py --mode text --orig-ism 0.5 --orig-desc dinov2_vits14 --orig-seg text_msam
one C_off run_ours.py --mode text --orig-ism 0.5 --orig-desc dinov2_vits14 --orig-seg text_msam --no-verify
one D     run_ours.py --mode text --proposer "$PR" --gate-sem 0.25 --gate-appe 0.605 --gate-hsv 0.06 --assign-fallback
one B     run_ours.py --mode text --orig-ism 0.5 --orig-desc dinov2_vits14 --orig-seg fastsam_full
one A     run_orig.py
log "COMBO_DONE"
