#!/bin/bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DB_PATH="${DB_PATH:-$ROOT/data/database/rtabmap.db}"
MAP_DIR="$ROOT/data/maps"

source "$ROOT/scripts/env.bash"

TORCH_LIB_DIR="$HOME/.local/lib/python3.12/site-packages/torch/lib"
if [[ -d "$TORCH_LIB_DIR" ]]; then
    export LD_LIBRARY_PATH="$TORCH_LIB_DIR:${LD_LIBRARY_PATH:-}"
fi

set -euo pipefail

if [[ ! -f "$DB_PATH" ]]; then
    echo "RTAB-Map database not found: $DB_PATH"
    exit 1
fi

mkdir -p "$MAP_DIR"

echo "Exporting 3D RGB point cloud from database..."
rtabmap-export \
    --cloud \
    --output rtabmap \
    --output_dir "$MAP_DIR" \
    --opt 2 \
    --max_range 5.0 \
    --decimation 2 \
    --voxel 0.03 \
    --noise_radius 0.08 \
    --noise_k 5 \
    "$DB_PATH"

if [[ -f "$MAP_DIR/rtabmap_cloud.ply" ]]; then
    :
elif [[ -f "$MAP_DIR/rtabmap_cloud_cloud.ply" ]]; then
    mv "$MAP_DIR/rtabmap_cloud_cloud.ply" "$MAP_DIR/rtabmap_cloud.ply"
elif [[ -f "$MAP_DIR/rtabmap_cloud.pcd" ]]; then
    echo "Point cloud exported as PCD: $MAP_DIR/rtabmap_cloud.pcd"
fi

if [[ ! -f "$MAP_DIR/map.yaml" ]]; then
    echo "WARNING: $MAP_DIR/map.yaml does not exist."
    echo "Run scripts/save_2d_map.bash while mapping is active."
fi

echo "Static 2D map: $MAP_DIR/map.yaml"
echo "3D point cloud: $MAP_DIR/rtabmap_cloud.ply"
