#!/usr/bin/env bash
# check_env.sh — V0: OS/ROS2/NIC PHC 환경 점검
#   사용: bash scripts/check_env.sh <iface>   (예: bash scripts/check_env.sh eno1)
set -u
IFACE="${1:-}"

echo "==== OS ===="
lsb_release -a 2>/dev/null || cat /etc/os-release
echo "   ↑ Ubuntu 24.04 (noble) 여야 Jazzy 사용 가능"

echo
echo "==== ROS2 ===="
echo "ROS_DISTRO=${ROS_DISTRO:-<unset: source /opt/ros/jazzy/setup.bash 필요>}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<default: rmw_fastrtps_cpp>}"
command -v ros2 >/dev/null && ros2 pkg list 2>/dev/null | grep -E 'realsense2_camera|velodyne|rosbag2' || \
  echo "  (ros2 미source 또는 패키지 미설치)"

echo
echo "==== NIC PHC (★ chrony-SW vs PTP-HW 분기) ===="
if [ -z "$IFACE" ]; then
  echo "  iface 미지정. 사용 가능한 인터페이스:"; ip -br link
  echo "  → bash scripts/check_env.sh <iface> 로 재실행"
else
  ethtool -T "$IFACE" 2>/dev/null || echo "  ethtool 실패(sudo apt install ethtool / iface명 확인)"
  echo "  → 'PTP Hardware Clock: 1' + hardware-* 있으면 HW, 없으면 chrony-SW 경로"
fi

echo
echo "==== 디스크 (기록 대상) ===="
df -h . | tail -1
echo "  → TLC NVMe 권장. QLC/SD/USB면 장시간 기록 시 병목 주의"
