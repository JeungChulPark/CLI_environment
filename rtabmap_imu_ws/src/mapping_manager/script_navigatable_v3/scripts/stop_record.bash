#!/bin/bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_DIR="${BAG_DIR:-$ROOT/data/rosbags}"
STATE_DIR="$BAG_DIR/.v3_recorder"
PID_FILE="$STATE_DIR/pid"
OUTPUT_FILE="$STATE_DIR/output"

source "$ROOT/scripts/env.bash"


set -euo pipefail

if [[ ! -f "$PID_FILE" || ! -f "$OUTPUT_FILE" ]]; then
    echo "No managed V3 rosbag recorder is running."
    echo "A bag without metadata can be repaired with:"
    echo "  ros2 bag reindex --storage sqlite3 <bag_directory>"
    echo "  ros2 bag reindex --storage mcap <bag_directory>  # legacy MCAP"
    exit 0
fi

RECORDER_PID="$(cat "$PID_FILE")"
OUTPUT="$(cat "$OUTPUT_FILE")"

if [[ ! "$RECORDER_PID" =~ ^[0-9]+$ ]]; then
    echo "ERROR: Invalid recorder PID state: $RECORDER_PID"
    exit 1
fi

if kill -0 "$RECORDER_PID" 2>/dev/null; then
    echo "Stopping V3 rosbag recorder cleanly..."

    # The recorder's Stop service closes the active storage and writes
    # metadata.yaml before the idle CLI process is terminated below.
    timeout 15 ros2 service call \
        /v3_rosbag_recorder/stop \
        rosbag2_interfaces/srv/Stop "{}" >/dev/null 2>&1 || true

    # Stop finalizes the storage but Jazzy's recorder process stays alive and
    # ignores SIGINT once recording has stopped. SIGTERM exits the idle CLI.
    sleep 1
    kill -TERM "$RECORDER_PID" 2>/dev/null || true

    for _ in $(seq 1 180); do
        if ! kill -0 "$RECORDER_PID" 2>/dev/null; then
            break
        fi
        sleep 1
    done
fi

if kill -0 "$RECORDER_PID" 2>/dev/null; then
    echo "ERROR: Recorder is still flushing after 180 seconds."
    exit 1
fi

for _ in $(seq 1 15); do
    if [[ -s "$OUTPUT/metadata.yaml" ]]; then
        break
    fi
    sleep 1
done

if [[ ! -s "$OUTPUT/metadata.yaml" ]]; then
    if find "$OUTPUT" -maxdepth 1 -type f -name '*.db3' -print -quit | grep -q .; then
        echo "Rebuilding missing metadata.yaml from SQLite3 files..."
        ros2 bag reindex --storage sqlite3 "$OUTPUT"
    elif find "$OUTPUT" -maxdepth 1 -type f -name '*.mcap' -print -quit | grep -q .; then
        echo "Rebuilding missing metadata.yaml from MCAP files..."
        ros2 bag reindex --storage mcap "$OUTPUT"
    fi
fi

rm -f "$PID_FILE" "$OUTPUT_FILE"

if [[ -s "$OUTPUT/metadata.yaml" ]]; then
    echo "Rosbag metadata finalized:"
    echo "  $OUTPUT/metadata.yaml"
else
    echo "ERROR: metadata.yaml is still missing from $OUTPUT"
    exit 1
fi
