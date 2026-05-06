#!/usr/bin/env bash
set -e

# 固定参数：可在这里改，也可由 launch_collect.sh 临时传入
export ROBOT_TYPE="${ROBOT_TYPE:-openarm_follower_ros2}"  # LeRobot 机器人接口类型
export TELEOP_TYPE="${TELEOP_TYPE:-openarm_leader_ros2}"  # LeRobot 遥操作接口类型
export REPO_ID="${REPO_ID:-local/openarm_ros2_dataset}"  # 数据集保存名
export NUM_EPISODES="${NUM_EPISODES:-100}"  # 录制 episode 数量
export TASK_DESC="${TASK_DESC:-pick up the object}"  # 当前任务描述
export FPS="${FPS:-}"               # 为空时读取相机 JSON defaults.fps
export CAMERA_CONFIG="${CAMERA_CONFIG:-}"  # 相机 JSON 配置文件
export HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"  # 1=离线保存到本地，不访问 Hugging Face
export LEROBOT_VENV="${LEROBOT_VENV:-}"  # LeRobot 虚拟环境；为空则使用当前环境
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"  # 避免 FastDDS 可变长度消息 history 报错

# 自动推导路径：通常不用改
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS2_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
export INSTALL_BASE="${ROS2_WS_INSTALL_BASE:-$ROS2_WS/install}"
[[ -z "${ROS2_WS_INSTALL_BASE:-}" && -f "$ROS2_WS/install_vla/setup.bash" ]] && export INSTALL_BASE="$ROS2_WS/install_vla"
[[ -z "$CAMERA_CONFIG" && -f "$ROS2_WS/src/openarm_cameras/config/openarm_cameras.json" ]] && export CAMERA_CONFIG="$ROS2_WS/src/openarm_cameras/config/openarm_cameras.json"

# 检查 ROS workspace 和 LeRobot 虚拟环境
[[ -f "$INSTALL_BASE/setup.bash" ]] || { echo "找不到 ROS setup: $INSTALL_BASE/setup.bash"; exit 1; }
[[ -f "$CAMERA_CONFIG" ]] || { echo "找不到 CAMERA_CONFIG: $CAMERA_CONFIG"; exit 1; }
[[ -z "$LEROBOT_VENV" || -d "$LEROBOT_VENV" ]] || { echo "找不到 LEROBOT_VENV: $LEROBOT_VENV"; exit 1; }

mapfile -t CAMERA_INFO < <(python3 - "$CAMERA_CONFIG" <<'PY'
import json
import sys

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as file_obj:
    config = json.load(file_obj)

defaults = config.get("defaults", {}) if isinstance(config, dict) else {}
cameras = config.get("cameras", {}) if isinstance(config, dict) else {}
slots = [
    ("head", "cam_head", "/cam/head/color/image_raw"),
    ("left", "cam_left_wrist", "/cam/left_wrist/color/image_raw"),
    ("right", "cam_right_wrist", "/cam/right_wrist/color/image_raw"),
]


def entry_for(slot):
    if isinstance(cameras, dict):
        return cameras.get(slot, {})
    if isinstance(cameras, list):
        for camera in cameras:
            if camera.get("slot") == slot or camera.get("name") == slot:
                return camera
    return {}


def value(entry, key, fallback):
    if isinstance(entry, dict) and key in entry:
        return entry[key]
    if key in defaults:
        return defaults[key]
    return fallback

dataset_fps = defaults.get("fps", 30)
items = []
for slot, name, topic in slots:
    entry = entry_for(slot)
    width = value(entry, "width", 640)
    height = value(entry, "height", 480)
    fps = value(entry, "fps", dataset_fps)
    items.append(f"{name}: {{type: openarm_ros2_topic, image_topic: {topic}, width: {width}, height: {height}, fps: {fps}}}")

print(dataset_fps)
print("{" + ", ".join(items) + "}")
PY
)
export FPS="${FPS:-${CAMERA_INFO[0]}}"
export LEROBOT_CAMERA_CONFIG="${CAMERA_INFO[1]}"

echo "录制 LeRobot 数据集: $REPO_ID"

# 加载 ROS 环境；如果提供 LEROBOT_VENV，也加载 LeRobot 虚拟环境
source /opt/ros/humble/setup.bash
source "$INSTALL_BASE/setup.bash"
[[ -n "$LEROBOT_VENV" ]] && source "$LEROBOT_VENV/bin/activate"

# 从 ROS2 topic 读取 OpenArm 状态、动作和三路相机，写成 LeRobot 数据集
HF_HUB_OFFLINE="$HF_HUB_OFFLINE" lerobot-record \
    --robot.type="$ROBOT_TYPE" \
    --teleop.type="$TELEOP_TYPE" \
    --robot.cameras="$LEROBOT_CAMERA_CONFIG" \
    --dataset.repo_id="$REPO_ID" \
    --dataset.num_episodes="$NUM_EPISODES" \
    --dataset.fps="$FPS" \
    --dataset.single_task="$TASK_DESC"