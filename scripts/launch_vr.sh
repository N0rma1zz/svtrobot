#!/usr/bin/env bash
set -e

# 固定参数：按需要改这里
export VR_VIDEO="true"              # true=启动三路视频推流, false=只启动 VR 控制
export VIDEO_LISTEN="0.0.0.0:13579" # VR 视频推流监听地址
export HEAD_TOPIC="/cam/head/color/image_raw"
export LEFT_TOPIC="/cam/left_wrist/color/image_raw"
export RIGHT_TOPIC="/cam/right_wrist/color/image_raw"
export URDF_PATH=""                 # 为空时自动使用 ros2_ws/openarm_bimanual_control.urdf
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"  # 避免 FastDDS 可变长度消息 history 报错

# 自动推导路径：通常不用改
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS2_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
export INSTALL_BASE="$ROS2_WS/install"
[[ -f "$ROS2_WS/install_vla/setup.bash" ]] && export INSTALL_BASE="$ROS2_WS/install_vla"
[[ -z "$URDF_PATH" ]] && export URDF_PATH="$ROS2_WS/openarm_bimanual_control.urdf"
export ROS_SETUP="export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION && source /opt/ros/humble/setup.bash && source $INSTALL_BASE/setup.bash"
export KEEP_OPEN='; echo; echo "按 Ctrl+D 或关闭窗口退出。"; exec bash'

# 传入 --kill 时，停止 VR 控制和视频推流
if [[ "${1:-}" == "--kill" ]]; then
    pkill -f 'teleop_openarm|video_sender' || true
    exit 0
fi

# 检查 ROS workspace 和 OpenArm URDF 是否存在
[[ -f "$INSTALL_BASE/setup.bash" ]] || { echo "找不到 ROS setup: $INSTALL_BASE/setup.bash"; exit 1; }
[[ -f "$URDF_PATH" ]] || { echo "找不到 URDF: $URDF_PATH"; exit 1; }

echo "启动 VR 控制: VIDEO=$VR_VIDEO LISTEN=$VIDEO_LISTEN"

# 启动 VR 遥操作节点，向 OpenArm 控制话题发布动作
gnome-terminal --title="openarm-vr" -- bash -lc "$ROS_SETUP && ros2 run vr_teleop teleop_openarm --urdf '$URDF_PATH'$KEEP_OPEN"

# 可选：把 ROS2 的三路相机图像推给 VR 端显示
[[ "$VR_VIDEO" == "true" ]] && gnome-terminal --title="openarm-vr-video" -- bash -lc "$ROS_SETUP && ros2 run vr_teleop video_sender --listen $VIDEO_LISTEN --ros --topic '$HEAD_TOPIC' --topic2 '$LEFT_TOPIC' --topic3 '$RIGHT_TOPIC'$KEEP_OPEN"

echo "已启动。需要先保证实机/仿真和相机话题已经在运行。"