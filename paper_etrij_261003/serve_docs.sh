#!/usr/bin/env bash
# 논문 문서 폴더를 LAN 에 정적으로 공개 (http://192.168.20.5:8787/)
cd "$(dirname "$0")"
exec python3 -m http.server 8787 --bind 0.0.0.0
