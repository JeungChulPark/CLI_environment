#!/usr/bin/env bash
# status.sh — 자동 처리 상태 한 화면
cd "$(dirname "$0")/../../.."; O=objpose/output/obchange_261005
date '+%T'
echo "-- chain: $(grep -E '^===' $O/logs/dualmap_chain.log | tail -1 | cut -c1-60) | ycbm $(ls objpose/output/fmf_cmp_ycbm_260929/maps/dualmap_ycbm 2>/dev/null | wc -l)/32 | obchange dualmap: $(ls $O/maps/dualmap 2>/dev/null | sed 's/_s.*//' | sort | uniq -c | tr '\n' ' ')| FAILED $(grep -c FAILED $O/logs/dualmap_chain.log)"
echo "-- watch: $(tail -1 $O/logs/watch.log)"
for r in smallroom bigroom; do [ -f $O/logs/$r.log ] && echo "   $r: $(grep -E '^===' $O/logs/$r.log | tail -1 | cut -c1-70) | 오류 $(grep -c -E 'Traceback|FAILED|No space|Killed' $O/logs/$r.log)"; done
echo "-- fixup: $(tail -1 $O/logs/fixup.log 2>/dev/null | cut -c1-80)"
echo "-- fmf detect: $(for s in $(ls $HOME/DeepLearning/Datasets/fmfusion_obchange_261005/scans | grep -E 'small|big'); do echo -n "$s:$(ls $HOME/DeepLearning/Datasets/fmfusion_obchange_261005/scans/$s/prediction_gsam 2>/dev/null | grep -c label) "; done)"
echo "-- bop: $(ls $HOME/DeepLearning/Datasets/ObChange/bop/test | sed 's/_s.*//' | sort | uniq -c | tr '\n' ' ')"
ps -eo etime,cmd | grep -E "run_room|run_dualmap_bop|runner_dataset|detect_gsam|proposed_maps|make_bop|IntegrateInst|run_compare|ycbv_rgbd|fixup_after|watch_rooms|chain_261005" | grep -v grep | cut -c1-100
nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader; df -h ~ | tail -1 | awk '{print "disk free",$4}'; free -g | awk 'NR==2{print "ram avail",$7"G"}'
