#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

source "$SCRIPT_DIR/env.bash"
cd "$WORKSPACE_ROOT"

# --symlink-install needs setuptools' develop mode, which recent setuptools
# (as shipped by conda) no longer accepts; override on such hosts.
colcon build ${COLCON_BUILD_ARGS---symlink-install}
echo
echo "Build done. Source with:"
echo "  source $WORKSPACE_ROOT/setup/env.bash"
