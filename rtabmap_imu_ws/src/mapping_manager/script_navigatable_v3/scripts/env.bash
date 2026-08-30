#!/usr/bin/env bash

THIS_FILE="${BASH_SOURCE[0]}"
THIS_DIR="$(cd "$(dirname "$THIS_FILE")" && pwd)"
STACK_ROOT="$(cd "$THIS_DIR/.." && pwd)"
PACKAGE_ROOT="$(cd "$STACK_ROOT/.." && pwd)"
WORKSPACE_ROOT="$(cd "$PACKAGE_ROOT/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

_NOUNSET_WAS_SET=0
case "$-" in
    *u*) _NOUNSET_WAS_SET=1; set +u ;;
esac

if [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
elif [[ -n "${CONDA_PREFIX:-}" && -f "$CONDA_PREFIX/setup.bash" ]]; then
    # Conda (RoboStack) ROS 2: this host has no /opt/ros tree.
    source "$CONDA_PREFIX/setup.bash"
else
    echo "ROS distro not found: /opt/ros/$ROS_DISTRO" >&2
    echo "(no conda ROS in CONDA_PREFIX either)" >&2
    return 1 2>/dev/null || exit 1
fi

# Optional colon-separated colcon workspaces overlaid before this one, for
# packages the ROS installation does not ship (e.g. a source-built rtabmap_ros).
if [[ -n "${EXTRA_ROS_WS:-}" ]]; then
    IFS=':' read -r -a _EXTRA_WS <<< "$EXTRA_ROS_WS"
    for _WS in "${_EXTRA_WS[@]}"; do
        [[ -f "$_WS/install/setup.bash" ]] && source "$_WS/install/setup.bash"
    done
    unset _EXTRA_WS _WS
fi

if [[ -f "$WORKSPACE_ROOT/install/setup.bash" ]]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
fi

if [[ "$_NOUNSET_WAS_SET" == "1" ]]; then
    set -u
fi
unset _NOUNSET_WAS_SET

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslogs}"
mkdir -p "$ROS_LOG_DIR"

TORCH_LIB_DIR="$HOME/.local/lib/python3.12/site-packages/torch/lib"
if [[ -d "$TORCH_LIB_DIR" ]]; then
    export LD_LIBRARY_PATH="$TORCH_LIB_DIR:${LD_LIBRARY_PATH:-}"
fi
