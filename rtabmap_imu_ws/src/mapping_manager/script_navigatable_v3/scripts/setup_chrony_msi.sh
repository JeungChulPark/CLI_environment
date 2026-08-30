#!/usr/bin/env bash
set -Eeuo pipefail

# Chrony server setup for MSI:
#   br0      = 192.168.131.1/24
#   enp132s0 = physical Ethernet member of br0
#
# Usage:
#   sudo bash setup_chrony_msi.sh setup
#   sudo bash setup_chrony_msi.sh status
#   sudo bash setup_chrony_msi.sh monitor
#   sudo bash setup_chrony_msi.sh rollback

BR_IF="${BR_IF:-br0}"
ETH_IF="${ETH_IF:-enp132s0}"
SERVER_CIDR="${SERVER_CIDR:-192.168.2.1/8765}"
ALLOW_NET="${ALLOW_NET:-192.168.131.0/24}"
CHRONY_CONF="${CHRONY_CONF:-/etc/chrony/chrony.conf}"
BACKUP_ROOT="${BACKUP_ROOT:-/var/backups/chrony-sync-msi}"
ACTION="${1:-setup}"

log()  { printf '\033[1;32m[MSI]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[MSI][WARN]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[MSI][ERROR]\033[0m %s\n' "$*" >&2; exit 1; }

if [[ "${EUID}" -ne 0 ]]; then
  exec sudo -E bash "$0" "$@"
fi

stop_conflicting_time_services() {
  log "Dừng PTP cũ và systemd-timesyncd..."
  pkill ptp4l 2>/dev/null || true
  pkill phc2sys 2>/dev/null || true

  local svc
  for svc in \
    ptp4l.service phc2sys.service \
    ptp4l-master.service ptp4l-client.service \
    phc2sys-master.service phc2sys-client.service; do
    systemctl disable --now "$svc" 2>/dev/null || true
  done

  systemctl disable --now systemd-timesyncd.service 2>/dev/null || true
}

restore_network_management() {
  log "Khôi phục systemd-networkd/Netplan nếu trước đó đã bị dừng hoặc mask..."
  systemctl unmask systemd-networkd.service systemd-networkd.socket 2>/dev/null || true
  systemctl unmask --runtime systemd-networkd.service systemd-networkd.socket 2>/dev/null || true
  systemctl start systemd-networkd.socket systemd-networkd.service 2>/dev/null || true
  command -v netplan >/dev/null 2>&1 && netplan apply || true
}

verify_network() {
  ip link show "$BR_IF" >/dev/null 2>&1 || die "Không tìm thấy bridge $BR_IF."
  ip link show "$ETH_IF" >/dev/null 2>&1 || die "Không tìm thấy interface $ETH_IF."

  if ! ip -4 -o addr show dev "$BR_IF" | grep -Fq " ${SERVER_CIDR}"; then
    die "$BR_IF chưa có IP $SERVER_CIDR. Kiểm tra /etc/netplan trước."
  fi

  if ! bridge link show dev "$ETH_IF" 2>/dev/null | grep -Fq "master $BR_IF"; then
    die "$ETH_IF chưa thuộc $BR_IF. Script không tự sửa để tránh phá cấu hình Husky."
  fi

  ip link show "$ETH_IF" | grep -q LOWER_UP || warn "$ETH_IF chưa có LOWER_UP; kiểm tra dây/switch."
  log "Mạng hợp lệ: $BR_IF=$SERVER_CIDR, $ETH_IF thuộc $BR_IF."
}

install_packages() {
  log "Cài Chrony và công cụ kiểm tra..."
  export DEBIAN_FRONTEND=noninteractive
  apt-get update
  apt-get install -y chrony iproute2 iputils-ping
}

backup_config() {
  mkdir -p "$BACKUP_ROOT"
  local stamp backup
  stamp="$(date +%Y%m%d-%H%M%S)"
  backup="$BACKUP_ROOT/chrony.conf.$stamp"
  cp -a "$CHRONY_CONF" "$backup"
  ln -sfn "$backup" "$BACKUP_ROOT/latest"
  log "Backup: $backup"
}

write_chrony_config() {
  log "Cấu hình MSI làm Chrony server..."
  local tmp
  tmp="$(mktemp)"

  awk -v allow_net="$ALLOW_NET" '
    /^# BEGIN CHRONY-SYNC-MSI$/,/^# END CHRONY-SYNC-MSI$/ { next }
    /^[[:space:]]*makestep[[:space:]]+/ { next }
    /^[[:space:]]*rtcsync([[:space:]]|$)/ { next }
    /^[[:space:]]*local[[:space:]]+stratum[[:space:]]+/ { next }
    $0 ~ "^[[:space:]]*allow[[:space:]]+" allow_net "([[:space:]]|$)" { next }
    { print }
  ' "$CHRONY_CONF" > "$tmp"

  cat >> "$tmp" <<CFG

# BEGIN CHRONY-SYNC-MSI
allow ${ALLOW_NET}
local stratum 8
makestep 0.1 3
rtcsync
# END CHRONY-SYNC-MSI
CFG

  install -m 0644 "$tmp" "$CHRONY_CONF"
  rm -f "$tmp"
}

configure_firewall() {
  if command -v ufw >/dev/null 2>&1 && ufw status 2>/dev/null | grep -q '^Status: active'; then
    log "Mở UDP/123 trên $BR_IF cho $ALLOW_NET..."
    ufw allow in on "$BR_IF" from "$ALLOW_NET" to any port 123 proto udp
  fi
}

start_chrony() {
  log "Khởi động Chrony server..."
  systemctl enable chrony.service >/dev/null
  systemctl restart chrony.service
  chronyc online >/dev/null 2>&1 || true
  chronyc burst 4/4 >/dev/null 2>&1 || true
  sleep 3

  systemctl is-active --quiet chrony.service || die "chrony.service không hoạt động."
  ss -lunp | grep -q ':123' || die "chronyd chưa lắng nghe UDP/123."
}

show_status() {
  printf '\n===== NETWORK =====\n'
  ip -br addr show "$BR_IF" || true
  bridge link show dev "$ETH_IF" || true
  ip route || true

  printf '\n===== SERVICE =====\n'
  systemctl --no-pager --full status chrony.service || true

  printf '\n===== UDP 123 =====\n'
  ss -lunp | grep ':123' || true

  printf '\n===== TRACKING =====\n'
  chronyc tracking || true

  printf '\n===== SOURCES =====\n'
  chronyc sources -v || true

  printf '\n===== CLIENTS =====\n'
  chronyc clients || true
}

rollback() {
  local backup
  backup="$(readlink -f "$BACKUP_ROOT/latest" 2>/dev/null || true)"
  [[ -n "$backup" && -f "$backup" ]] || die "Không tìm thấy backup trong $BACKUP_ROOT."
  cp -a "$backup" "$CHRONY_CONF"
  systemctl restart chrony.service
  log "Đã khôi phục từ $backup."
}

case "$ACTION" in
  setup)
    stop_conflicting_time_services
    restore_network_management
    verify_network
    install_packages
    backup_config
    write_chrony_config
    configure_firewall
    start_chrony
    show_status
    printf '\nHoàn tất MSI. Sau khi cấu hình Laptop B, chạy: sudo chronyc clients\n'
    ;;
  status) show_status ;;
  monitor) exec watch -n 1 "chronyc tracking; echo; chronyc sources -v; echo; chronyc clients" ;;
  rollback) rollback ;;
  *) die "Dùng: setup|status|monitor|rollback" ;;
esac
