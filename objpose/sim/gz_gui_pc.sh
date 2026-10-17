#!/usr/bin/env bash
# PC: Gazebo GUI window attached to the Gazebo server running on the laptop (sim_up.sh). Needs conda env gz_harmonic
# (conda-forge gz-sim8) and the meshes copied to ~/gz_assets/share (clearpath_*_description, velodyne_description,
# clearpath_gz) plus Fuel models in ~/.gz/fuel.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${CONDA_SH:-$HOME/anaconda3/etc/profile.d/conda.sh}"
conda activate gz_harmonic
export GZ_IP=192.168.2.2 GZ_PARTITION=husky_sim
# same absolute Fuel cache path as the laptop server (it sends absolute mesh paths)
export GZ_FUEL_CACHE_PATH=/tmp/husky_sim_fuel
mkdir -p $GZ_FUEL_CACHE_PATH && rsync -a $HOME/.gz/fuel/ $GZ_FUEL_CACHE_PATH/
A=$HOME/gz_assets/share
export GZ_SIM_RESOURCE_PATH=$A/clearpath_gz/worlds:$A/clearpath_gz/meshes:$A
exec gz sim -g -v 2 --gui-config "$HERE/gui_pc.config" "$@"
