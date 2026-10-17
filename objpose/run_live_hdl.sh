#!/bin/bash
# run_live_hdl.sh — kept for the first live runs (2026-09-17); now a thin wrapper: LIVE sensors with hdl_graph_slam.
#   bash objpose/run_live_hdl.sh [hub.py options]   ==   bash objpose/run_live.sh hdl [hub.py options]
exec bash "$(dirname "${BASH_SOURCE[0]}")/run_live.sh" hdl "$@"
