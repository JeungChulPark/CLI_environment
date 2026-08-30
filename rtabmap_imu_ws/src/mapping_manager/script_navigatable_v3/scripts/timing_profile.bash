#!/usr/bin/env bash

# Load one timing profile from config/timestamp_sync.yaml.
# This file is sourced by the V3 entrypoints; it does not start any process.

TIMING_PROFILE_ROOT="$(
    cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd
)"
TIMING_PROFILE_CONFIG="$TIMING_PROFILE_ROOT/config/timestamp_sync.yaml"

TIMING_PROFILE_VALUES="$(
    python3 - \
        "$TIMING_PROFILE_CONFIG" \
        "${TIMING_PROFILE:-}" \
        "${REALSENSE_INTER_CAM_SYNC_MODE:-}" <<'PY'
import sys

import yaml

config_path, requested_profile, requested_mode = sys.argv[1:4]
with open(config_path, encoding="utf-8") as stream:
    policy = yaml.safe_load(stream)

profile_name = requested_profile or str(policy["active_profile"])
profiles = policy.get("profiles", {})
if profile_name not in profiles:
    available = ", ".join(sorted(profiles))
    raise SystemExit(
        f"Unknown TIMING_PROFILE={profile_name!r}; available: {available}"
    )

profile = profiles[profile_name]
camera = profile["realsense"]
xsens = profile["xsens"]
velodyne = profile["velodyne_vlp16"]
allowed_modes = [
    int(value)
    for value in camera.get(
        "allowed_inter_cam_sync_modes",
        [camera["inter_cam_sync_mode"]],
    )
]
selected_mode = (
    int(requested_mode)
    if requested_mode
    else int(camera["inter_cam_sync_mode"])
)
if selected_mode not in allowed_modes:
    allowed = ", ".join(str(value) for value in allowed_modes)
    raise SystemExit(
        f"REALSENSE_INTER_CAM_SYNC_MODE={selected_mode} is invalid for "
        f"{profile_name}; allowed: {allowed}"
    )

def boolean(value):
    return "true" if bool(value) else "false"

values = [
    profile_name,
    str(profile["role"]),
    boolean(profile["require_chrony_sync"]),
    str(profile["camera_fps"]),
    str(selected_mode),
    boolean(camera["global_time_enabled"]),
    boolean(not camera["manual_exposure"]),
    str(xsens["time_option"]),
    boolean(velodyne["gps_time"]),
    str(velodyne["time_offset"]),
    boolean(velodyne["timestamp_first_packet"]),
]
print("\t".join(values))
PY
)" || {
    unset TIMING_PROFILE_ROOT TIMING_PROFILE_CONFIG TIMING_PROFILE_VALUES
    return 1 2>/dev/null || exit 1
}

IFS=$'\t' read -r \
    PROFILE_NAME \
    PROFILE_ROLE \
    PROFILE_REQUIRE_CHRONY_SYNC \
    PROFILE_CAMERA_FPS \
    PROFILE_REALSENSE_INTER_CAM_SYNC_MODE \
    PROFILE_REALSENSE_GLOBAL_TIME_ENABLED \
    PROFILE_REALSENSE_AUTO_EXPOSURE \
    PROFILE_XSENS_TIME_OPTION \
    PROFILE_VLP16_GPS_TIME \
    PROFILE_VLP16_TIME_OFFSET \
    PROFILE_VLP16_TIMESTAMP_FIRST_PACKET \
    <<<"$TIMING_PROFILE_VALUES"

TIMING_PROFILE="${TIMING_PROFILE:-$PROFILE_NAME}"
CAMERA_FPS="${CAMERA_FPS:-$PROFILE_CAMERA_FPS}"
REALSENSE_INTER_CAM_SYNC_MODE="$PROFILE_REALSENSE_INTER_CAM_SYNC_MODE"
REALSENSE_GLOBAL_TIME_ENABLED="$PROFILE_REALSENSE_GLOBAL_TIME_ENABLED"
REALSENSE_AUTO_EXPOSURE="$PROFILE_REALSENSE_AUTO_EXPOSURE"
XSENS_TIME_OPTION="$PROFILE_XSENS_TIME_OPTION"
VLP16_GPS_TIME="$PROFILE_VLP16_GPS_TIME"
VLP16_TIME_OFFSET="$PROFILE_VLP16_TIME_OFFSET"
VLP16_TIMESTAMP_FIRST_PACKET="$PROFILE_VLP16_TIMESTAMP_FIRST_PACKET"

unset TIMING_PROFILE_ROOT TIMING_PROFILE_CONFIG TIMING_PROFILE_VALUES
