#!/bin/bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! source "$ROOT/scripts/timing_profile.bash"; then
    exit 1
fi

DURATION="${DURATION:-120}"
WARMUP="${WARMUP:-10}"
WATCHDOG="${WATCHDOG:-150}"
OUTPUT_JSON="${OUTPUT_JSON:-/tmp/triple_sync_report.json}"

source "$ROOT/scripts/env.bash"

set -euo pipefail

ros_float() {
    local value="$1"
    if [[ "$value" =~ ^[0-9]+$ ]]; then
        printf '%s.0' "$value"
    elif [[ "$value" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        printf '%s' "$value"
    else
        echo "Expected a non-negative number, got: $value" >&2
        return 1
    fi
}

DURATION_ROS="$(ros_float "$DURATION")"
WARMUP_ROS="$(ros_float "$WARMUP")"

echo "Verifying D455f + MTi-630 + VLP-16 triple synchronization"
echo "Collection: ${DURATION}s (warm-up ${WARMUP}s)"
echo "Profile: $TIMING_PROFILE ($PROFILE_ROLE)"
echo "Report: $OUTPUT_JSON"
echo

set +e
timeout --signal=INT "$WATCHDOG" \
    python3 "$ROOT/nodes/triple_sync_verifier.py" \
    --ros-args \
    -p duration_sec:="$DURATION_ROS" \
    -p warmup_sec:="$WARMUP_ROS" \
    -p timing_profile:="$TIMING_PROFILE" \
    -p output_json:="$OUTPUT_JSON"
STATUS=$?
set -e

case "$STATUS" in
    0)
        echo "PASS: triple-sensor timestamp alignment and profile are verified."
        ;;
    1)
        echo "FAIL: data, alignment, or active timestamp configuration is invalid."
        ;;
    2)
        echo "Verifier configuration error."
        ;;
    124|130)
        echo "Verification did not complete before the watchdog or was interrupted."
        ;;
    *)
        echo "Verifier error (exit $STATUS)."
        ;;
esac

exit "$STATUS"
