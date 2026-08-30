#!/bin/bash

INTERFACE="${LIDAR_INTERFACE:-enx00e04c680136}"
CONNECTION="${LIDAR_CONNECTION:-vlp16-direct}"
HOST_IP="${LIDAR_HOST_IP:-192.168.1.100}"
LIDAR_IP="${LIDAR_IP:-192.168.1.201}"
VIRTUAL_INTERFACE="${LIDAR_L3_INTERFACE:-vlp16-l3}"

set -euo pipefail

for command in ip nmcli ping readlink; do
    if ! command -v "$command" >/dev/null 2>&1; then
        echo "Required command not found: $command"
        exit 1
    fi
done

if [[ ! -d "/sys/class/net/$INTERFACE" ]]; then
    echo "LiDAR Ethernet interface not found: $INTERFACE"
    exit 1
fi

MASTER_PATH="$(readlink -f "/sys/class/net/$INTERFACE/master" 2>/dev/null || true)"
if [[ -n "$MASTER_PATH" && -d "$MASTER_PATH" ]]; then
    PARENT_INTERFACE="$(basename "$MASTER_PATH")"
    L3_INTERFACE="$VIRTUAL_INTERFACE"
    echo "$INTERFACE is a bridge port; creating $L3_INTERFACE on $PARENT_INTERFACE."

    # An Ethernet profile cannot complete while the physical device is a port
    # of an externally managed bridge.  A persistent macvlan gives this host a
    # Layer-3 endpoint on the bridge without taking ownership of br0 or
    # detaching either bridge port.
    if nmcli -t -f NAME connection show | grep -Fxq "$CONNECTION"; then
        nmcli --wait 10 connection down "$CONNECTION" >/dev/null 2>&1 || true
        if [[ "$(nmcli -g connection.type connection show "$CONNECTION")" != "macvlan" ]]
        then
            nmcli connection delete "$CONNECTION" >/dev/null
        fi
    fi
    if ! nmcli -t -f NAME connection show | grep -Fxq "$CONNECTION"; then
        nmcli connection add \
            type macvlan \
            ifname "$L3_INTERFACE" \
            con-name "$CONNECTION" \
            dev "$PARENT_INTERFACE" \
            mode bridge
    fi
    nmcli connection modify "$CONNECTION" \
        connection.interface-name "$L3_INTERFACE" \
        connection.autoconnect yes \
        macvlan.parent "$PARENT_INTERFACE" \
        macvlan.mode bridge \
        ipv4.method manual \
        ipv4.addresses "$HOST_IP/32" \
        ipv4.routes "$LIDAR_IP/32" \
        ipv4.never-default yes \
        ipv6.method disabled
    nmcli --wait 20 connection up "$CONNECTION"
else
    L3_INTERFACE="$INTERFACE"
    if nmcli -t -f NAME connection show | grep -Fxq "$CONNECTION"; then
        nmcli --wait 10 connection down "$CONNECTION" >/dev/null 2>&1 || true
        if [[ "$(nmcli -g connection.type connection show "$CONNECTION")" != "802-3-ethernet" ]]
        then
            nmcli connection delete "$CONNECTION" >/dev/null
        fi
    fi
    if nmcli -t -f NAME connection show | grep -Fxq "$CONNECTION"; then
        echo "Updating NetworkManager profile: $CONNECTION"
    else
        echo "Creating NetworkManager profile: $CONNECTION"
        nmcli connection add \
            type ethernet \
            ifname "$INTERFACE" \
            con-name "$CONNECTION"
    fi

    nmcli connection modify "$CONNECTION" \
        connection.interface-name "$INTERFACE" \
        connection.autoconnect yes \
        ipv4.method manual \
        ipv4.addresses "$HOST_IP/32" \
        ipv4.routes "$LIDAR_IP/32" \
        ipv4.never-default yes \
        ipv6.method disabled

    nmcli --wait 20 connection up "$CONNECTION"
fi

echo "VLP-16 direct link configured:"
echo "  physical:  $INTERFACE"
echo "  Layer-3:   $L3_INTERFACE"
echo "  host:      $HOST_IP/32"
echo "  sensor:    $LIDAR_IP/32"

if ping -I "$HOST_IP" -c 1 -W 1 "$LIDAR_IP" >/dev/null 2>&1; then
    echo "VERIFIED: $LIDAR_IP responds on the direct link."
else
    echo "WARNING: $LIDAR_IP did not answer ping; UDP may still be active."
fi
