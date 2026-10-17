#!/usr/bin/env bash
cd /home/jucpark/DeepLearning/CLI_environment_server
P=/home/jucpark/anaconda3/envs/sam6d/bin/python; T=objpose/tools/ycbv_sam6d/eval_reassoc.py; O=objpose/output; R=$O/reassoc_260926
mkdir -p $R
run() { $P $T --run-dir $2 --name $1 --out $R/$1.json > $R/$1.log 2>&1; $P $T --run-dir $2 --name $1_noslam --no-slam --out $R/$1_noslam.json > $R/$1_noslam.log 2>&1; $P $T --run-dir $2 --name $1_occl --occl --out $R/$1_occl.json > $R/$1_occl.log 2>&1; }
( run ycbv_orig $O/cmp_260924/orig_ycbv_realtime_top1 ) &
( run ycbv_assume $O/cmp_260925_fair/proposed_ycbv_realtime ) &
( run ycbv_actual $O/ycbv_sam6d_realtime_260921 ) &
( export BENCH_ROOT=/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_M/xtion_bop BENCH_TABLE=$PWD/objpose/rt/bop/ycbm_objects.json BENCH_SLAM=$PWD/objpose/output/ycbm_slam_orb_f3000_260926 BENCH_FPS=6
  run ycbm_orig $O/bench_ycbm_rt_own_260926/orig_ycbm_realtime_top1 &
  run ycbm_assume $O/bench_ycbm_rt_vis50_260926/proposed_ycbm_realtime &
  run ycbm_actual $O/bench_ycbm_rt_own_260926/proposed_ycbm_realtime &
  wait ) &
wait; echo REASSOC_DONE
