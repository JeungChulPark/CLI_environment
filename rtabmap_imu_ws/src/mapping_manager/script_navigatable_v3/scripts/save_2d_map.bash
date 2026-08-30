#!/bin/bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_DIR="${MAP_DIR:-$ROOT/data/maps}"
RTABMAP_SERVICE_ROOT="/rtabmap/rtabmap"
PAUSE_SERVICE="$RTABMAP_SERVICE_ROOT/pause"
RESUME_SERVICE="$RTABMAP_SERVICE_ROOT/resume"
GET_MAP_SERVICE="$RTABMAP_SERVICE_ROOT/get_map"

source "$ROOT/scripts/env.bash"


set -euo pipefail

mkdir -p "$MAP_DIR"

# RTAB-Map runs get_map in the same mutually-exclusive callback group as its
# mapping updates. Pause/resume is an optional extra when those services are
# advertised, but get_map remains safe on builds that do not expose them.
echo "Requesting a consistent 2D occupancy grid from RTAB-Map..."
if ! ROS_LOG_DIR=/tmp/roslogs python3 \
    "$ROOT/scripts/save_occupancy_grid.py" \
    --pause-service "$PAUSE_SERVICE" \
    --resume-service "$RESUME_SERVICE" \
    --service "$GET_MAP_SERVICE" \
    --output "$MAP_DIR/map"
then
    echo "ERROR: RTAB-Map did not return a valid 2D occupancy grid."
    echo "Save the map while the RTAB-Map mapping terminal is still active."
    exit 1
fi

echo "Saved static map:"
echo "  $MAP_DIR/map.yaml"
echo "  $MAP_DIR/map.pgm"
