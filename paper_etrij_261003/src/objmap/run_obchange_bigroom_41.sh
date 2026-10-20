#!/bin/bash
# GPU 3 chain: BigRoom (visits 1..7, 98k frames) in parallel with the GPU 2 chain, on its own hub ports.
set -u
source $HOME/Dataset/bop/ycbv_work/obchange_rooms_env_41.sh
export CUDA_VISIBLE_DEVICES=3 RT_PORT_ARGS="--http-port 8785 --slam-port 17021" REUSE_DETECT=1 SAM6D_SHM_SUFFIX=_g3   # 19:30 재시작: DualMap 실시간 훅 설치 후, FM-Fusion 검출은 재사용
cd $S
wait_convert BigRoom || { say "SKIP BigRoom: conversion failed"; exit 1; }
say "START BigRoom (last visit 7, GPU $CUDA_VISIBLE_DEVICES, ports 8785/17021)"
bash run_obchange_rt_ports.sh BigRoom 7 > out30/logs_bigroom/rt_chain.log 2>&1; rc=$?
say "DONE BigRoom rc=$rc"; report BigRoom
say "BIGROOM_DONE"
