#!/bin/bash
# 논문 폴더(src/ycbv) 미완 실험을 이 PC 에서: 렌더 완료 대기 → 자산 → L2 900장(노트북에서 오류로 멈춤) → 우리 기술 900장(4090 참고) → 채점
W=$HOME/Dataset/bop/ycbv_work; S=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003/src/ycbv
source $W/env.sh; export YCBV_OUT=$S/out_4090; mkdir -p $YCBV_OUT
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/chain.log
say(){ echo "[$(date +%T)] $*" >> $LOG; }
say "wait render"
until [ -f $W/templates/obj_000021/templates/xyz_41.npy ] && ! pgrep -f "render_templates_bproc" >/dev/null; do sleep 20; done
n=$(ls $W/templates/*/templates/xyz_41.npy | wc -l); say "render done objs=$n"
cd $S
say "build_assets start"; $PY build_assets.py > $W/logs/build_assets.log 2>&1 || { say "FAIL build_assets"; exit 1; }
say "build_assets done"
say "smoke L2"; $PY ladder/run_ladder.py --level L2 --limit 3 --warmup 1 > $W/logs/smoke_L2.log 2>&1 || { say "FAIL smoke L2"; exit 1; }
say "L2 900 start"; $PY ladder/run_ladder.py --level L2 > $W/logs/ladder_L2.log 2>&1 || say "FAIL L2"
say "L2 900 end"
say "ours_text(L4) 900 start"; $PY run_ours.py --mode text > $W/logs/ours_text.log 2>&1 || say "FAIL ours_text"
say "ours_text end"
say "evaluate"; $PY evaluate.py --skip_bop --methods ladder_L2,ours_text --results $S/results_4090.json > $W/logs/evaluate.log 2>&1 || say "FAIL evaluate"
say "ALL DONE"
