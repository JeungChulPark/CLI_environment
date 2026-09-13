#!/usr/bin/env bash
# check_link.sh — 어느 쪽에서 실행해도 자기 위치를 판단해 점검한다.
#   WSL2(.100) 에서 :  맥으로 나가는 길과 수신기
#   맥(.113) 에서    :  역터널로 들어오는 길
MAC="${MAC_HOST:-192.168.219.113}"
PC="${PC_HOST:-192.168.219.100}"
DATA="${DATA_PORT:-5765}"
CTRL="${CTRL_PORT:-2222}"
ok(){ printf '  \033[32m OK \033[0m %s\n' "$*"; }
ng(){ printf '  \033[31mFAIL\033[0m %s\n' "$*"; }
wn(){ printf '  \033[33mWARN\033[0m %s\n' "$*"; }
tcp(){ timeout 2 bash -c "echo >/dev/tcp/$1/$2" 2>/dev/null; }

if grep -qi microsoft /proc/version 2>/dev/null; then WHERE=wsl
elif [ "$(uname -s)" = "Darwin" ]; then WHERE=mac
else WHERE=other; fi
echo "== 위치: $WHERE =="

if [ "$WHERE" = wsl ]; then
  echo "-- 1. 자기 주소 --"
  ip -4 addr show eth0 | awk '/inet /{print "  WSL2 내부 IP: "$2}'
  echo "  (LAN 주소 $PC 는 Windows 것이다. WSL2 는 NAT 뒤에 있다)"

  echo "-- 2. 맥으로 나가는 길 --"
  ping -c1 -W2 "$MAC" >/dev/null 2>&1 && ok "ping $MAC" || ng "ping $MAC"
  tcp "$MAC" 22 && ok "$MAC:22 (원격 로그인 켜짐)" \
    || ng "$MAC:22 닫힘 — 맥에서 '시스템 설정 > 일반 > 공유 > 원격 로그인' 켤 것"

  echo "-- 3. 키 인증 --"
  if [ -f ~/.ssh/id_ed25519.pub ] || [ -f ~/.ssh/id_rsa.pub ]; then ok "공개키 있음"
  else wn "키 없음 — ssh-keygen -t ed25519"; fi
  if tcp "$MAC" 22; then
    if timeout 8 ssh -o BatchMode=yes -o ConnectTimeout=5 \
         -o StrictHostKeyChecking=accept-new "${MAC_USER:-$(whoami)}@$MAC" true 2>/dev/null
      then ok "비밀번호 없이 ssh 접속"
      else wn "키 인증 안 됨 — 맥의 ~/.ssh/authorized_keys 에 공개키 등록 필요"; fi
  fi

  echo "-- 4. 수신기 --"
  tcp 127.0.0.1 "$DATA" && ok "수신기 127.0.0.1:$DATA 대기 중" \
    || wn "수신기 없음 — receiver/link.sh up"

  echo "-- 5. WSL2 sshd (맥에서 제어하려면 필요) --"
  ss -tln 2>/dev/null | grep -q ':22 ' && ok "sshd 동작" \
    || wn "sshd 없음 — setup/install_sshd.sh (데이터 전송에는 불필요)"

elif [ "$WHERE" = mac ]; then
  echo "-- 1. 원격 로그인 --"
  sudo systemsetup -getremotelogin 2>/dev/null | grep -qi on && ok "켜짐" || ng "꺼짐"
  echo "-- 2. 역터널이 서 있나 (WSL2 가 걸어야 생긴다) --"
  tcp 127.0.0.1 "$DATA" && ok "localhost:$DATA — 데이터 경로 열림" \
    || ng "localhost:$DATA 닫힘 — WSL2 에서 receiver/link.sh up 실행됐는지 확인"
  tcp 127.0.0.1 "$CTRL" && ok "localhost:$CTRL — 제어 경로 열림" \
    || wn "localhost:$CTRL 닫힘 (WSL2 sshd 가 없으면 정상)"
  echo "-- 3. 왕복 --"
  if tcp 127.0.0.1 "$DATA"; then
    printf '{"type":"hello","source":"check_link"}\n' \
      | timeout 3 nc 127.0.0.1 "$DATA" >/dev/null 2>&1 && ok "테스트 메시지 전송" \
      || wn "전송 실패"
  fi
else
  echo "  WSL2 도 맥도 아니다. MAC_HOST/PC_HOST 를 주고 수동 확인할 것."
fi
