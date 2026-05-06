#!/usr/bin/env bash
set -e

export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_BASE="$SCRIPT_DIR/install"
[[ -f "$SCRIPT_DIR/install_vla/setup.bash" ]] && INSTALL_BASE="$SCRIPT_DIR/install_vla"

source /opt/ros/humble/setup.bash
source "$INSTALL_BASE/setup.bash"

ros2 launch openarm_bringup openarm.bimanual.launch.py \
	robot_controller:=forward_position_controller \
	use_fake_hardware:=false

