#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CLIENT_WS="${HDL_SERVICE_CLIENT_WS:-/tmp/hdl_service_client_ws}"
MAP_PATH="${1:-$ROOT_DIR/output/hdl_tiers_map.pcd}"
GRAPH_DIR="${2:-$ROOT_DIR/output/graph_dump}"
CLIENT_BIN="$CLIENT_WS/install/hdl_service_client/lib/hdl_service_client/export_hdl_results"
EXPORT_TIMEOUT="${HDL_EXPORT_TIMEOUT:-60s}"

if [[ ! -x "$CLIENT_BIN" ]]; then
  echo "Missing C++ export client: $CLIENT_BIN" >&2
  echo "Rebuild or recreate $CLIENT_WS before exporting HDL results." >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
source "$CLIENT_WS/install/setup.bash"
set -u

exec timeout "$EXPORT_TIMEOUT" "$CLIENT_BIN" "$MAP_PATH" "$GRAPH_DIR"
