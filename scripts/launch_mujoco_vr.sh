#!/usr/bin/env bash
set -e

# 固定参数：按需要改这里
export RECORD="true"                # true=启动 LeRobot 录制, false=只跑仿真和VR
export SHOW_VIEWER="false"          # true=显示MuJoCo窗口, false=后台渲染
export CTRL_RATE=50                  # MuJoCo 控制循环频率
export CAM_RATE=30                   # MuJoCo 相机发布频率
export FPS=15                        # 数据集录制频率
export REPO_ID="local/mujoco_vr_dataset"  # LeRobot 数据集保存名
export NUM_EPISODES=100              # 录制 episode 数量
export TASK_DESC="pick up the object"  # 当前采集任务描述
export MUJOCO_VENV="$HOME/openarm_ws/mujoco_venv"  # MuJoCo Python 虚拟环境
export LEROBOT_VENV=""               # LeRobot 虚拟环境；为空则使用当前环境
export VIDEO_LISTEN="0.0.0.0:13579"  # VR 视频推流监听地址
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"  # 避免 FastDDS 可变长度消息 history 报错

# 自动推导路径：通常不用改
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS2_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
export INSTALL_BASE="$ROS2_WS/install"
[[ -f "$ROS2_WS/install_vla/setup.bash" ]] && export INSTALL_BASE="$ROS2_WS/install_vla"
export URDF_PATH="$ROS2_WS/openarm_bimanual_control.urdf"
export SCENE_XML="$ROS2_WS/src/openarm_mujoco/v1/openarm_scene_groot.xml"
export ROS_SETUP="export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION && source /opt/ros/humble/setup.bash && source $INSTALL_BASE/setup.bash"
export KEEP_OPEN='; echo; echo "按 Ctrl+D 或关闭窗口退出。"; exec bash'

# 传入 --kill 时，停止 MuJoCo + VR 相关进程
if [[ "${1:-}" == "--kill" ]]; then
	pkill -f 'mujoco_ros2_bridge.py|teleop_openarm|video_sender|lerobot-record' || true
	exit 0
fi

# 检查 ROS、MuJoCo 虚拟环境和场景文件是否存在
[[ -f "$INSTALL_BASE/setup.bash" ]] || { echo "找不到 ROS setup: $INSTALL_BASE/setup.bash"; exit 1; }
[[ -d "$MUJOCO_VENV" ]] || { echo "找不到 MUJOCO_VENV: $MUJOCO_VENV"; exit 1; }
[[ -f "$SCENE_XML" ]] || { echo "找不到 MuJoCo 场景文件: $SCENE_XML"; exit 1; }

# 默认用 egl 后台渲染；需要窗口时切到 glx 并加 --viewer
export MUJOCO_GL="egl"
export VIEWER_ARG=""
[[ "$SHOW_VIEWER" == "true" ]] && export MUJOCO_GL="glx" && export VIEWER_ARG="--viewer"

echo "启动 MuJoCo + VR 采集"

# 启动 MuJoCo bridge，发布关节状态和三路仿真相机话题
gnome-terminal --title="mujoco-bridge" -- bash -lc "$ROS_SETUP && source $MUJOCO_VENV/bin/activate && export MUJOCO_GL=$MUJOCO_GL && python3 $ROS2_WS/tools/mujoco_ros2_bridge.py --xml $SCENE_XML --ctrl-rate $CTRL_RATE --cam-rate $CAM_RATE $VIEWER_ARG$KEEP_OPEN"

# 等 bridge 先启动，再启动 VR 控制和视频推流
sleep 3

# 启动 VR 遥操作节点，向 OpenArm 控制话题发布动作
gnome-terminal --title="vr-teleop" -- bash -lc "$ROS_SETUP && ros2 run vr_teleop teleop_openarm --urdf $URDF_PATH$KEEP_OPEN"

# 把 MuJoCo/ROS2 的三路相机图像推给 VR 端显示
gnome-terminal --title="vr-video" -- bash -lc "$ROS_SETUP && ros2 run vr_teleop video_sender --listen $VIDEO_LISTEN --ros --topic /cam/head/color/image_raw --topic2 /cam/left_wrist/color/image_raw --topic3 /cam/right_wrist/color/image_raw$KEEP_OPEN"

# 可选：启动 LeRobot 录制
[[ "$RECORD" == "true" ]] && gnome-terminal --title="lerobot-record" -- bash -lc "ROS2_WS_INSTALL_BASE=$INSTALL_BASE LEROBOT_VENV=$LEROBOT_VENV REPO_ID=$REPO_ID NUM_EPISODES=$NUM_EPISODES FPS=$FPS TASK_DESC='$TASK_DESC' $ROS2_WS/scripts/record_lerobot_from_ros2.sh$KEEP_OPEN"

echo "已启动。数据集名、任务描述和 viewer 开关在脚本顶部修改。"