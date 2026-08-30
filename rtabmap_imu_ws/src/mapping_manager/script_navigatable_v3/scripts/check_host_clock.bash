#!/usr/bin/env bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! source "$ROOT/scripts/timing_profile.bash"; then
    exit 1
fi

set -euo pipefail

if [[ "$PROFILE_REQUIRE_CHRONY_SYNC" != "true" ]]; then
    echo "Clock preflight: not required by $TIMING_PROFILE."
    exit 0
fi

MAX_OFFSET_MS="${CLOCK_MAX_OFFSET_MS:-5}"
if ! TRACKING="$(chronyc -c tracking 2>/dev/null)"; then
    echo "ERROR: Chrony is required by $TIMING_PROFILE but chronyc tracking failed." >&2
    exit 1
fi

python3 - "$TRACKING" "$MAX_OFFSET_MS" "$TIMING_PROFILE" <<'PY'
import math
import sys

tracking, maximum_ms, profile = sys.argv[1:4]
fields = tracking.strip().split(",")
if len(fields) < 13:
    raise SystemExit(
        f"ERROR: unexpected chronyc tracking output ({len(fields)} fields)."
    )

has_reference_name = len(fields) >= 14
stratum_index = 2 if has_reference_name else 1
system_offset_index = 4 if has_reference_name else 3
last_offset_index = 5 if has_reference_name else 4
leap_index = 13 if has_reference_name else 12

try:
    stratum = int(fields[stratum_index])
    system_offset_ms = float(fields[system_offset_index]) * 1000.0
    last_offset_ms = float(fields[last_offset_index]) * 1000.0
    limit_ms = float(maximum_ms)
except (TypeError, ValueError) as error:
    raise SystemExit(f"ERROR: cannot parse chronyc tracking output: {error}")

leap = fields[leap_index].strip()
valid = (
    0 < stratum <= 15
    and leap.lower() == "normal"
    and math.isfinite(system_offset_ms)
    and math.isfinite(last_offset_ms)
    and abs(system_offset_ms) <= limit_ms
    and abs(last_offset_ms) <= limit_ms
)
print(
    "Clock preflight: "
    f"profile={profile} stratum={stratum} leap={leap} "
    f"system_offset={system_offset_ms:.3f}ms "
    f"last_offset={last_offset_ms:.3f}ms limit={limit_ms:.3f}ms"
)
if not valid:
    raise SystemExit(
        "ERROR: Chrony is not synchronized closely enough for a dual-host bag."
    )
PY
