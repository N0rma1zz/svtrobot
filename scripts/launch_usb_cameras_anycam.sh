#!/usr/bin/env bash
set -e

# 固定参数：相机设备、三路分辨率、FPS、QoS 都写在 JSON 里
export CAMERA_CONFIG=""              # 为空时自动找 src/openarm_cameras/config/openarm_cameras.json
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"  # 避免 FastDDS 可变长度消息 history 报错

# 自动推导路径：通常不用改
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS2_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
export INSTALL_BASE="$ROS2_WS/install"
[[ -f "$ROS2_WS/install_vla/setup.bash" ]] && export INSTALL_BASE="$ROS2_WS/install_vla"
[[ -z "$CAMERA_CONFIG" && -f "$ROS2_WS/src/openarm_cameras/config/openarm_cameras.json" ]] && export CAMERA_CONFIG="$ROS2_WS/src/openarm_cameras/config/openarm_cameras.json"

# 检查 ROS workspace 和相机 JSON 配置
[[ -f "$INSTALL_BASE/setup.bash" ]] || { echo "找不到 ROS setup: $INSTALL_BASE/setup.bash"; exit 1; }
[[ -f "$CAMERA_CONFIG" ]] || { echo "找不到 CAMERA_CONFIG: $CAMERA_CONFIG"; exit 1; }

echo "启动三路 USB 相机"

# 加载 ROS 环境，然后启动三路 USB 相机 launch
source /opt/ros/humble/setup.bash
source "$INSTALL_BASE/setup.bash"

# 发布 /cam/head、/cam/left_wrist、/cam/right_wrist 三路图像话题
ros2 launch openarm_cameras usb_cameras.launch.py config_file:="$CAMERA_CONFIG"