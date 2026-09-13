#!/usr/bin/env bash
# WSL2 안에 sshd 를 올린다. **sudo 비밀번호가 필요해 자동화하지 않았다** —
# 아래를 그대로 붙여넣어 실행하면 된다.
#
# 이게 있어야 맥에서 역터널로 WSL2 에 로그인해 명령을 돌릴 수 있다.
# 데이터 전송(포즈 스트림)만 쓸 거면 없어도 된다.
set -e
sudo apt-get update -qq
sudo apt-get install -y openssh-server
# WSL2 는 LAN 에서 직접 안 보이므로 루프백만 듣게 한다. 역터널로만 들어온다.
sudo sed -i 's/^#\?ListenAddress .*/ListenAddress 127.0.0.1/' /etc/ssh/sshd_config
sudo sed -i 's/^#\?PasswordAuthentication .*/PasswordAuthentication no/' /etc/ssh/sshd_config
sudo sed -i 's/^#\?PubkeyAuthentication .*/PubkeyAuthentication yes/' /etc/ssh/sshd_config
grep -q '^ListenAddress 127.0.0.1' /etc/ssh/sshd_config || echo 'ListenAddress 127.0.0.1' | sudo tee -a /etc/ssh/sshd_config
sudo ssh-keygen -A
sudo systemctl enable --now ssh
systemctl is-active ssh && ss -tln | grep ':22 '
echo
echo "다음: 맥의 공개키를 ~/.ssh/authorized_keys 에 넣는다."
echo "  맥에서:  ssh-keygen -t ed25519           (없으면)"
echo "  맥에서:  cat ~/.ssh/id_ed25519.pub       (내용 복사)"
echo "  여기서:  mkdir -p ~/.ssh && chmod 700 ~/.ssh"
echo "           echo '<붙여넣기>' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
