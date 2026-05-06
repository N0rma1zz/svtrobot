#!/usr/bin/env bash
set -e

# 固定参数：按需要改这里
export LEFT_PORT="/dev/ttyUSB1"      # 左外骨骼串口，不是相机
export RIGHT_PORT="/dev/ttyUSB0"     # 右外骨骼串口，不是相机
export CALIBRATION_FILE="$HOME/.openarm_exo_calibration.json"  # 外骨骼标定文件
export PUBLISH_RATE="50.0"           # 外骨骼控制话题发布频率
export RECALIBRATE_ON_START="false"  # true=启动时重新标定
export CALIBRATION_ONLY="false"      # true=只标定，不进入遥操作
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"  # 避免 FastDDS 可变长度消息 history 报错

# 自动推导路径：通常不用改
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS2_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
export INSTALL_BASE="$ROS2_WS/install"
[[ -f "$ROS2_WS/install_vla/setup.bash" ]] && export INSTALL_BASE="$ROS2_WS/install_vla"

# 检查 ROS workspace 是否已编译
[[ -f "$INSTALL_BASE/setup.bash" ]] || { echo "找不到 ROS setup: $INSTALL_BASE/setup.bash"; exit 1; }

echo "启动外骨骼遥操作"

# 加载 ROS 环境，然后启动外骨骼 teleop launch
source /opt/ros/humble/setup.bash
source "$INSTALL_BASE/setup.bash"

# 从左右外骨骼串口读取数据，并发布 OpenArm 控制命令
ros2 launch openarm_exoskeleton exo_teleop.launch.py \
    left_port:="$LEFT_PORT" \
    right_port:="$RIGHT_PORT" \
    calibration_file:="$CALIBRATION_FILE" \
    publish_rate:="$PUBLISH_RATE" \
    recalibrate_on_start:="$RECALIBRATE_ON_START" \
    calibration_only:="$CALIBRATION_ONLY"