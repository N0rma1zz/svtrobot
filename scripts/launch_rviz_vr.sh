#!/usr/bin/env bash
set -e

# 固定参数：按需要改这里
export ARM_TYPE="v10"                # OpenArm 机械臂型号
export USE_FAKE_HARDWARE="true"      # true=不接真实硬件，只用 fake hardware/RViz
export ROBOT_CONTROLLER="forward_position_controller"  # RViz 验证用控制器
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"  # 避免 FastDDS 可变长度消息 history 报错

# 自动推导路径：通常不用改
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS2_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
export INSTALL_BASE="$ROS2_WS/install"
[[ -f "$ROS2_WS/install_vla/setup.bash" ]] && export INSTALL_BASE="$ROS2_WS/install_vla"
export URDF_PATH="$ROS2_WS/openarm_bimanual_control.urdf"
export ROS_SETUP="export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION && source /opt/ros/humble/setup.bash && source $INSTALL_BASE/setup.bash"
export KEEP_OPEN='; echo; echo "按 Ctrl+D 或关闭窗口退出。"; exec bash'

# 传入 --kill 时，停止 RViz fake hardware 和 VR 相关进程
if [[ "${1:-}" == "--kill" ]]; then
	pkill -f 'teleop_openarm|openarm\.bimanual\.launch\.py' || true
	exit 0
fi

# 检查 ROS workspace 和 OpenArm URDF 是否存在
[[ -f "$INSTALL_BASE/setup.bash" ]] || { echo "找不到 ROS setup: $INSTALL_BASE/setup.bash"; exit 1; }
[[ -f "$URDF_PATH" ]] || { echo "找不到 URDF: $URDF_PATH"; exit 1; }

echo "启动 RViz fake hardware + VR 检查"

# 启动 fake hardware/RViz 侧的 OpenArm bringup
gnome-terminal --title="rviz-openarm" -- bash -lc "$ROS_SETUP && ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=$ARM_TYPE use_fake_hardware:=$USE_FAKE_HARDWARE robot_controller:=$ROBOT_CONTROLLER$KEEP_OPEN"

# 等 RViz/控制器启动后，再启动 VR 节点
sleep 4

# 启动 VR 遥操作节点，验证控制话题是否能驱动 fake hardware
gnome-terminal --title="rviz-vr" -- bash -lc "$ROS_SETUP && ros2 run vr_teleop teleop_openarm --urdf $URDF_PATH$KEEP_OPEN"

echo "已启动。RViz 显示 fake hardware 后，VR 会驱动手臂话题。"