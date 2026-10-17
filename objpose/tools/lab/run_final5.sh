#!/usr/bin/env bash
# run_final5.sh — 2026-10-11 확정 구성 다섯 가지(①=2 원본, ②=9 화면 전체 분할 + 관문, ③=8 제안 방법, ④=7 완화, ⑤=6 총점. 번호는 live_six.py 의 LIVE_SIX)를 실시간으로 돌린다.
#   수집 영상 8개(실제 환경, 2 는 끝남 · 6 7 8 은 run_live_rt.sh 가 돌리는 중 → 끝나면 9) → ObChange SmallRoom 다섯 구성. 지도 설계 조합은 실행 중에 계산한다. 끝나면 rt_obchange_261011/log.txt 에 RT_OB_DONE.
cd "$(dirname "$0")/../../.."
while pgrep -f "tools/lab/run_live_r[t].sh" > /dev/null; do sleep 20; done
CONFIGS="9" bash objpose/tools/lab/run_live_rt.sh
CONFIGS="2 9 8 7 6" bash objpose/tools/obchange/run_rt_obchange.sh smallroom
