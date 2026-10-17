#!/usr/bin/env bash
# run_rt_all.sh — 실행 중 지도 설계 실험(2026-10-11 확정: ViT-S 구성 2 · 6 · 7 · 8 만): 수집 영상(실제 환경, 구성 2 는 이미 끝남) → ObChange SmallRoom → 나머지 방. 그래픽 카드를 하나만 쓰므로 차례로 돌린다.
cd "$(dirname "$0")/../../.."
CONFIGS="6 7 8" bash objpose/tools/lab/run_live_rt.sh
CONFIGS="2 6 7 8" bash objpose/tools/obchange/run_rt_obchange.sh smallroom; CONFIGS="2 6 7 8" bash objpose/tools/obchange/run_rt_obchange.sh office livingarea kitchen
