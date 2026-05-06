#!/usr/bin/env bash
set -e

# 固定参数：按需要改这里
export BACKEND="real"                 # real=实机, mujoco=仿真
export CONTROL="vr"                  # vr=VR遥操作, exo=外骨骼
export RECORD="true"                 # true=录制数据集, false=不录制
export SHOW_VIEWER="false"           # true=显示MuJoCo窗口, false=后台渲染
export VR_VIDEO="true"               # true=给VR推三路相机视频
export ARM_TYPE="v10"                # OpenArm 机械臂型号，当前用 v10
export USE_FAKE_HARDWARE="false"     # true=假硬件/RViz验证, false=真实硬件

# 相机 JSON 配置；设备、三路分辨率、FPS、QoS 都写在这个文件里
export CAMERA_CONFIG=""              # 为空时自动找 src/openarm_cameras/config/openarm_cameras.json

# 外骨骼串口；这两个不是相机 port
export LEFT_PORT="/dev/ttyUSB1"      # 左外骨骼串口
export RIGHT_PORT="/dev/ttyUSB0"     # 右外骨骼串口
export CALIBRATION_FILE="$HOME/.openarm_exo_calibration.json"  # 外骨骼标定文件
export RECALIBRATE_ON_START="false"  # true=启动时重新标定外骨骼
export REPO_ID="local/openarm_collection"  # LeRobot 数据集保存名
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
export ROS_SETUP="export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION && source /opt/ros/humble/setup.bash && source $INSTALL_BASE/setup.bash"
export KEEP_OPEN='; echo; echo "按 Ctrl+D 或关闭窗口退出。"; exec bash'
[[ -z "$CAMERA_CONFIG" && -f "$ROS2_WS/src/openarm_cameras/config/openarm_cameras.json" ]] && export CAMERA_CONFIG="$ROS2_WS/src/openarm_cameras/config/openarm_cameras.json"

# 传入 --kill 时，停止本脚本可能拉起的采集相关进程
if [[ "${1:-}" == "--kill" ]]; then
    pkill -f 'mujoco_ros2_bridge.py|teleop_openarm|video_sender|openarm\.bimanual\.launch\.py|usb_cameras\.launch\.py|exo_teleop\.launch\.py|lerobot-record' || true
    exit 0
fi

# 确认 ROS2 workspace 已经编译并存在 setup.bash
[[ -f "$INSTALL_BASE/setup.bash" ]] || { echo "找不到 ROS setup: $INSTALL_BASE/setup.bash"; exit 1; }

echo "OpenArm 采集启动: BACKEND=$BACKEND CONTROL=$CONTROL RECORD=$RECORD"

if [[ "$BACKEND" == "real" ]]; then
    # 实机模式需要相机 JSON
    [[ -f "$CAMERA_CONFIG" ]] || { echo "找不到 CAMERA_CONFIG: $CAMERA_CONFIG"; exit 1; }

    # 启动真实 OpenArm 双臂控制和 ros2_control 控制器
    gnome-terminal --title="openarm-robot" -- bash -lc "$ROS_SETUP && ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=$ARM_TYPE use_fake_hardware:=$USE_FAKE_HARDWARE$KEEP_OPEN"

    # 等机械臂控制器先起来，再启动相机节点
    sleep 3

    # 启动三路 USB 相机，并发布到 /cam/head、/cam/left_wrist、/cam/right_wrist
    gnome-terminal --title="openarm-cameras" -- bash -lc "$ROS_SETUP && ros2 launch openarm_cameras usb_cameras.launch.py config_file:='$CAMERA_CONFIG'$KEEP_OPEN"
elif [[ "$BACKEND" == "mujoco" ]]; then
    # 仿真模式需要先准备 MuJoCo 虚拟环境
    [[ -d "$MUJOCO_VENV" ]] || { echo "找不到 MUJOCO_VENV: $MUJOCO_VENV"; exit 1; }

    # 默认用 egl 后台渲染；需要窗口时切到 glx 并加 --viewer
    export MUJOCO_GL="egl"
    export VIEWER_ARG=""
    [[ "$SHOW_VIEWER" == "true" ]] && export MUJOCO_GL="glx" && export VIEWER_ARG="--viewer"

    # 启动 MuJoCo bridge，发布 /joint_states 和三路仿真相机话题
    gnome-terminal --title="openarm-mujoco" -- bash -lc "$ROS_SETUP && source $MUJOCO_VENV/bin/activate && export MUJOCO_GL=$MUJOCO_GL && python3 $ROS2_WS/tools/mujoco_ros2_bridge.py --xml $ROS2_WS/src/openarm_mujoco/v1/openarm_scene_groot.xml --ctrl-rate 50 --cam-rate 30 $VIEWER_ARG$KEEP_OPEN"

    # 等 MuJoCo bridge 开始发布话题后，再启动控制源
    sleep 3
else
    echo "BACKEND 只能是 real 或 mujoco"
    exit 1
fi

if [[ "$CONTROL" == "vr" ]]; then
    # 启动 VR 遥操作节点，读取手柄/头显姿态并发布 OpenArm 控制命令
    gnome-terminal --title="openarm-vr" -- bash -lc "$ROS_SETUP && ros2 run vr_teleop teleop_openarm --urdf $ROS2_WS/openarm_bimanual_control.urdf$KEEP_OPEN"

    # 可选：把三路 ROS2 相机图像推给 VR 端显示
    [[ "$VR_VIDEO" == "true" ]] && gnome-terminal --title="openarm-video" -- bash -lc "$ROS_SETUP && ros2 run vr_teleop video_sender --listen $VIDEO_LISTEN --ros --topic /cam/head/color/image_raw --topic2 /cam/left_wrist/color/image_raw --topic3 /cam/right_wrist/color/image_raw$KEEP_OPEN"
elif [[ "$CONTROL" == "exo" ]]; then
    # 启动外骨骼遥操作节点，通过串口读取左右外骨骼并发布 OpenArm 控制命令
    gnome-terminal --title="openarm-exo" -- bash -lc "$ROS_SETUP && ros2 launch openarm_exoskeleton exo_teleop.launch.py left_port:='$LEFT_PORT' right_port:='$RIGHT_PORT' calibration_file:='$CALIBRATION_FILE' recalibrate_on_start:=$RECALIBRATE_ON_START$KEEP_OPEN"
else
    echo "CONTROL 只能是 vr 或 exo"
    exit 1
fi

# 可选：启动 LeRobot 录制，从 ROS2 topic 读取状态、动作和三路图像
[[ "$RECORD" == "true" ]] && gnome-terminal --title="openarm-record" -- bash -lc "ROS2_WS_INSTALL_BASE=$INSTALL_BASE CAMERA_CONFIG='$CAMERA_CONFIG' LEROBOT_VENV=$LEROBOT_VENV REPO_ID=$REPO_ID NUM_EPISODES=$NUM_EPISODES TASK_DESC='$TASK_DESC' $ROS2_WS/scripts/record_lerobot_from_ros2.sh$KEEP_OPEN"

echo "已启动。下次运行前直接修改脚本顶部固定参数。"