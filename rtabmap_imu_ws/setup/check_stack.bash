#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.bash"

missing=0

check_pkg() {
    local pkg="$1"
    if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
        printf 'OK      %s\n' "$pkg"
    else
        printf 'MISSING %s\n' "$pkg"
        missing=1
    fi
}

for pkg in "$@"; do
    check_pkg "$pkg"
done

exit "$missing"
