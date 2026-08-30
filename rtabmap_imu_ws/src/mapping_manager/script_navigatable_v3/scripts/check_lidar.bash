#!/bin/bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/env.bash"

set -euo pipefail

topic_has_data() {
    local topic="$1"
    local timeout_seconds="${2:-3}"

    ros2 topic echo "$topic" --once --timeout "$timeout_seconds" --no-arr >/dev/null 2>&1
}

publisher_count() {
    local topic="$1"
    local info

    info="$(ros2 topic info "$topic" 2>/dev/null || true)"
    awk -F': ' '/Publisher count/ {print $2; found=1} END {if (!found) print 0}' <<<"$info"
}

transform_ready() {
    local parent="$1"
    local child="$2"
    local timeout_seconds="${3:-6}"
    local pipefail_state
    local status

    # tf2_echo keeps running after it finds a transform, so its timeout status
    # cannot be used as the readiness result. Stop at the first valid sample.
    pipefail_state="$(set -o | awk '$1 == "pipefail" {print $2}')"
    set +o pipefail
    timeout "$timeout_seconds" ros2 run tf2_ros tf2_echo \
        "$parent" "$child" 2>/dev/null | grep -Fq -m 1 "Translation:"
    status=$?
    if [[ "$pipefail_state" == "on" ]]; then
        set -o pipefail
    fi
    return "$status"
}

echo "=== Velodyne VLP-16 topics ==="
for topic in /velodyne_packets /velodyne_points /scan; do
    publishers="$(publisher_count "$topic")"
    if [[ "$publishers" == "0" ]]; then
        echo "MISS $topic (publishers=0)"
    elif (( publishers > 1 )); then
        echo "FAIL $topic has $publishers publishers; stop the stale LiDAR stack."
        exit 1
    elif topic_has_data "$topic" 5; then
        echo "OK   $topic (publishers=$publishers, data=yes)"
    else
        echo "WARN $topic (publishers=$publishers, data=no within 5s)"
    fi
done

echo
echo "=== TF check ==="
if transform_ready base_link velodyne 12; then
    echo "OK   TF base_link -> velodyne is available"
else
    echo "FAIL TF base_link -> velodyne is not available"
    exit 1
fi

echo
echo "=== Body-fixed frame and configured range ==="
timeout 12 python3 "$ROOT/scripts/check_lidar_geometry.py" \
    --config "$ROOT/config/vlp16.yaml" \
    --samples 5 \
    --timeout 10
