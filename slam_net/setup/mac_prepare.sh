#!/usr/bin/env bash
# 맥(192.168.219.113)에서 한 번 실행한다. WSL2 에서 실행하는 것이 아니다.
#
# WSL2 가 맥으로 **나가는** SSH 를 걸어 역터널을 세우는 구조라, 맥에는
# 원격 로그인(sshd)이 켜져 있어야 한다. 이것 하나가 유일한 필수 선행 조건이다.
set -e
echo "== 1. 원격 로그인 =="
if sudo systemsetup -getremotelogin 2>/dev/null | grep -qi on; then
  echo "   이미 켜져 있음"
else
  sudo systemsetup -setremotelogin on
  echo "   켰음"
fi

echo "== 2. WSL2 쪽 공개키 등록 =="
echo "   WSL2 에서 만든 공개키를 ~/.ssh/authorized_keys 에 넣어야"
echo "   터널이 비밀번호 없이 자동 재접속한다."
echo "   WSL2 에서:  cat ~/.ssh/id_ed25519.pub"
echo "   여기서  :  mkdir -p ~/.ssh && chmod 700 ~/.ssh"
echo "              echo '<붙여넣기>' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"

echo "== 3. 역터널 포트가 맥의 localhost 에만 열리는지 =="
echo "   기본값(GatewayPorts no)이면 localhost 전용이라 안전하다. 그대로 두면 된다."

echo "== 4. 확인 =="
echo "   터널이 선 뒤 맥에서:"
echo "     nc -z localhost 5765 && echo '데이터 포트 OK'"
echo "     ssh -p 2222 $(whoami)@localhost 'echo 제어 OK'   # sshd 설치했을 때"
