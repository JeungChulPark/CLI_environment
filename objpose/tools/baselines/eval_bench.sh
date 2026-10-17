#!/usr/bin/env bash
# 실시간 비교 실행 채점: 처리 통계(eval_cmp rt), 지도 표시 일치율(display_eval), 주행 뒤 지도(build_memory_maps)
#   bash objpose/tools/baselines/eval_bench.sh hope objpose/output/bench_hope_rt_260925
set -u
cd "$(dirname "$0")/../../.."
DS=$1; F=$(realpath $2)
case $DS in
  hope) export BENCH_ROOT=/home/jucpark/DeepLearning/Datasets/paper_benchmarks/HOPE/video_bop BENCH_TABLE=$PWD/objpose/rt/bop/hope_objects.json BENCH_SLAM=$PWD/objpose/output/hope_slam_orb_f3000_260925 BENCH_FPS=6 ;;
  ycbm) export BENCH_ROOT=/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_M/xtion_bop BENCH_TABLE=$PWD/objpose/rt/bop/ycbm_objects.json BENCH_SLAM=$PWD/objpose/output/ycbm_slam_orb_f3000_260926 BENCH_FPS=6 ;;
esac
export BENCH_TAG=$DS CMP_OUT=$F DISPLAY_OUT=$F/display
P=/home/jucpark/anaconda3/envs/sam6d/bin/python
$P objpose/tools/baselines/eval_cmp.py rt > $F/rt_scores.log 2>&1
$P objpose/tools/baselines/display_eval.py orig $F/orig_${DS}_realtime_top1 > $F/display_orig.log 2>&1 &
$P objpose/tools/baselines/display_eval.py proposed $F/proposed_${DS}_realtime > $F/display_proposed.log 2>&1 &
$P objpose/tools/ycbv_sam6d/build_memory_maps.py --run-dir $F/orig_${DS}_realtime_top1 --out $F/memory_orig > $F/memory_orig.log 2>&1 &
$P objpose/tools/ycbv_sam6d/build_memory_maps.py --run-dir $F/proposed_${DS}_realtime --out $F/memory_proposed > $F/memory_proposed.log 2>&1 &
wait
echo EVAL_DONE
