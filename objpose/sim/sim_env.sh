# Source on the laptop: ROS 2 Jazzy + access to the logged-in desktop (Wayland session, XWayland for Gazebo's Qt GUI).
source /opt/ros/jazzy/setup.bash
# domain 5: keeps the simulation apart from the real Husky driver (domain 0, same a200_0881 namespace) on this laptop
export ROS_DOMAIN_ID=5
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export DISPLAY=${DISPLAY:-:0}
export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}
export QT_QPA_PLATFORM=xcb
xa=$(ls -t $XDG_RUNTIME_DIR/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
[ -n "$xa" ] && export XAUTHORITY=$xa
export SIM_DIR=$HOME/clearpath_sim
# Gazebo transport over the wired LAN so the PC (192.168.2.2) can attach a Gazebo GUI (objpose/sim/gz_gui_pc.sh).
export GZ_IP=192.168.2.1
export GZ_PARTITION=husky_sim
# Fuel model cache at a path that exists on both machines, because the server sends absolute mesh paths to the PC GUI.
export GZ_FUEL_CACHE_PATH=/tmp/husky_sim_fuel
mkdir -p $GZ_FUEL_CACHE_PATH && rsync -a $HOME/.gz/fuel/ $GZ_FUEL_CACHE_PATH/
