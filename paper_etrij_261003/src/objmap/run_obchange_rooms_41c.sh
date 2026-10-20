#!/bin/bash
# GPU 2 chain, third start (19:30): Office → LivingArea → SmallRoom with run_obchange_rt_ports.sh (default ports 8775/17011).
# Restarted because .41's DualMap lacked the real-time frame-dropping hook (DM_RT_INTERVAL in applications/runner_dataset.py,
# copied from .192 at 19:27) — the earlier Office DualMap runs processed every frame. REUSE_DETECT keeps the FM-Fusion
# real-time detection already made for Office. Launch pinned: taskset -c 0-13.
set -u
source $HOME/Dataset/bop/ycbv_work/obchange_rooms_env_41.sh
export CUDA_VISIBLE_DEVICES=2 REUSE_DETECT=1 SAM6D_SHM_SUFFIX=_g2
cd $S
for RL in "Office 6" "LivingArea 6" "SmallRoom 6"; do
  set -- $RL; ROOM=$1; LAST=$2; r=$(echo $ROOM | tr A-Z a-z)
  wait_convert $ROOM || { say "SKIP $ROOM: conversion failed"; continue; }
  say "START $ROOM (last visit $LAST, GPU $CUDA_VISIBLE_DEVICES, DualMap real-time hook)"
  bash run_obchange_rt_ports.sh $ROOM $LAST > out30/logs_$r/rt_chain.log 2>&1; rc=$?
  say "DONE $ROOM rc=$rc"; report $ROOM
done
say "ROOMS_GPU2_DONE"
