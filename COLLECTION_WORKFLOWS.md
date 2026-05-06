# OpenArm 采集工作流

这套脚本的使用方式统一为：先打开脚本，改顶部的少量 `export` 参数，再运行脚本。脚本底部就是实际执行的 `ros2 launch`、`ros2 run` 或 `lerobot-record` 命令，便于直接看清楚启动了什么。

本地开发时进入：

```bash
cd /home/normal/git_ws/vla/vla_remote/ros2_ws
```

同步到远端后，在远端进入：

```bash
cd ~/vla/ros2_ws
```

## 推荐主入口

优先使用 [scripts/launch_collect.sh](scripts/launch_collect.sh)。它负责组合三件事：实机还是仿真、VR 还是外骨骼、是否录制数据集。

运行：

```bash
./scripts/launch_collect.sh
```

停止它拉起的相关进程：

```bash
./scripts/launch_collect.sh --kill
```

主开关在脚本顶部：

```bash
export BACKEND="real"      # real | mujoco
export CONTROL="vr"       # vr | exo
export RECORD="true"      # true | false
```

含义：

- `BACKEND="real"`：启动真实 OpenArm 和 USB 相机。
- `BACKEND="mujoco"`：启动 MuJoCo bridge，由仿真发布关节状态和相机话题。
- `CONTROL="vr"`：启动 `vr_teleop teleop_openarm`，用 VR 控制手臂。
- `CONTROL="exo"`：启动 `openarm_exoskeleton exo_teleop.launch.py`，用外骨骼控制手臂。
- `RECORD="true"`：额外启动 [scripts/record_lerobot_from_ros2.sh](scripts/record_lerobot_from_ros2.sh) 录制 LeRobot 数据集。
- `RECORD="false"`：只运行机器人/仿真和控制源，不录数据。

注意：相机没有单独的 `HEAD_PORT`。三路 USB 相机的设备路径、分辨率、FPS 和 QoS 都统一写到 `CAMERA_CONFIG` 指向的 JSON 文件里。`LEFT_PORT` 和 `RIGHT_PORT` 是外骨骼串口，不是相机。

## 实机 + VR

适合真实 OpenArm、三路 USB 相机、VR 遥操作采集。

在 [scripts/launch_collect.sh](scripts/launch_collect.sh) 顶部配置：

```bash
export BACKEND="real"
export CONTROL="vr"
export RECORD="true"

export ARM_TYPE="v10"
export USE_FAKE_HARDWARE="false"
export VR_VIDEO="true"
```

然后运行：

```bash
./scripts/launch_collect.sh
```

这个模式会拉起：

- `openarm_bringup openarm.bimanual.launch.py`
- `openarm_cameras usb_cameras.launch.py`
- `vr_teleop teleop_openarm`
- 可选的 `vr_teleop video_sender`
- 可选的 `lerobot-record`

如果只想先确认三路相机，可以先改 [src/openarm_cameras/config/openarm_cameras.json](src/openarm_cameras/config/openarm_cameras.json)，然后单独运行 [scripts/launch_usb_cameras_anycam.sh](scripts/launch_usb_cameras_anycam.sh)。

如果真实机械臂和相机已经单独启动，只想启动 VR 控制和视频推流，运行 [scripts/launch_vr.sh](scripts/launch_vr.sh)：

```bash
./scripts/launch_vr.sh
```

## 实机 + 外骨骼

适合真实 OpenArm、三路 USB 相机、外骨骼遥操作采集。

在 [scripts/launch_collect.sh](scripts/launch_collect.sh) 顶部配置：

```bash
export BACKEND="real"
export CONTROL="exo"
export RECORD="true"

export LEFT_PORT="/dev/ttyUSB1"
export RIGHT_PORT="/dev/ttyUSB0"
export CALIBRATION_FILE="$HOME/.openarm_exo_calibration.json"
export RECALIBRATE_ON_START="false"
```

然后运行：

```bash
./scripts/launch_collect.sh
```

这个模式会拉起真实机械臂、相机、外骨骼 teleop，以及可选的数据录制。外骨骼串口如果不确定，可以先看：

```bash
ls /dev/ttyUSB*
```

如果只想单独调外骨骼，不启动机械臂和相机，运行 [scripts/run_exo_servo_zero.sh](scripts/run_exo_servo_zero.sh)：

```bash
./scripts/run_exo_servo_zero.sh
```

## 仿真 + VR

适合先在 MuJoCo 中验证 VR 控制、相机话题和录制流程，不接真实机械臂。

方式一：用主入口 [scripts/launch_collect.sh](scripts/launch_collect.sh)：

```bash
export BACKEND="mujoco"
export CONTROL="vr"
export RECORD="true"
export MUJOCO_VENV="$HOME/openarm_ws/mujoco_venv"
export SHOW_VIEWER="false"
export VR_VIDEO="true"
```

然后运行：

```bash
./scripts/launch_collect.sh
```

方式二：用专门入口 [scripts/launch_mujoco_vr.sh](scripts/launch_mujoco_vr.sh)：

```bash
./scripts/launch_mujoco_vr.sh
```

[scripts/launch_mujoco_vr.sh](scripts/launch_mujoco_vr.sh) 只负责 MuJoCo + VR 这条链路，顶部常改参数是：

```bash
export RECORD="true"
export SHOW_VIEWER="false"
export CTRL_RATE=50
export CAM_RATE=30
export MUJOCO_VENV="$HOME/openarm_ws/mujoco_venv"
export REPO_ID="local/mujoco_vr_dataset"
export NUM_EPISODES=100
export TASK_DESC="pick up the object"
```

`SHOW_VIEWER="false"` 更适合无显示器或远端运行；如果本机有图形界面并想看 MuJoCo 窗口，改成 `SHOW_VIEWER="true"`。

## 仿真 + 外骨骼

适合用外骨骼控制 MuJoCo，先验证外骨骼控制链路，不接真实机械臂。

在 [scripts/launch_collect.sh](scripts/launch_collect.sh) 顶部配置：

```bash
export BACKEND="mujoco"
export CONTROL="exo"
export RECORD="true"
export MUJOCO_VENV="$HOME/openarm_ws/mujoco_venv"

export LEFT_PORT="/dev/ttyUSB1"
export RIGHT_PORT="/dev/ttyUSB0"
export CALIBRATION_FILE="$HOME/.openarm_exo_calibration.json"
```

然后运行：

```bash
./scripts/launch_collect.sh
```

这个模式会拉起 MuJoCo bridge、外骨骼 teleop，以及可选的数据录制。外骨骼输出仍然走同一套 OpenArm ROS2 控制话题，所以录制脚本不需要区分实机或仿真。

## RViz + VR 检查

[scripts/launch_rviz_vr.sh](scripts/launch_rviz_vr.sh) 用 fake hardware 和 RViz 检查 VR 控制链路，不适合正式采集，但适合快速确认 VR 节点能否发控制命令。

运行：

```bash
./scripts/launch_rviz_vr.sh
```

顶部常改参数：

```bash
export ARM_TYPE="v10"
export USE_FAKE_HARDWARE="true"
export ROBOT_CONTROLLER="forward_position_controller"
```

停止：

```bash
./scripts/launch_rviz_vr.sh --kill
```

## 单独 VR + 视频推流

[scripts/launch_vr.sh](scripts/launch_vr.sh) 只启动 VR 遥操作和三路视频推流，不负责启动真实机械臂、MuJoCo 或相机。适合下面两种情况：

- 实机 OpenArm 和 USB 相机已经启动，只想接入 VR。
- MuJoCo bridge 已经启动，只想接入 VR 和 VR 端画面。

运行：

```bash
./scripts/launch_vr.sh
```

停止：

```bash
./scripts/launch_vr.sh --kill
```

顶部常改参数：

```bash
export VR_VIDEO="true"
export VIDEO_LISTEN="0.0.0.0:13579"
export HEAD_TOPIC="/cam/head/color/image_raw"
export LEFT_TOPIC="/cam/left_wrist/color/image_raw"
export RIGHT_TOPIC="/cam/right_wrist/color/image_raw"
```

`VR_VIDEO="true"` 会同时启动 `vr_teleop video_sender`，把三路 ROS2 图像话题推给 VR 端；改成 `false` 时只启动 `teleop_openarm`。

## 单独脚本说明

- [scripts/launch_collect.sh](scripts/launch_collect.sh)：推荐主入口，组合实机/仿真、VR/外骨骼、录制/不录制。
- [scripts/launch_mujoco_vr.sh](scripts/launch_mujoco_vr.sh)：只启动 MuJoCo + VR，适合仿真调试。
- [scripts/launch_vr.sh](scripts/launch_vr.sh)：只启动 VR 控制 + 三路视频推流，适合实机/仿真和相机已经运行时接入 VR。
- [scripts/launch_rviz_vr.sh](scripts/launch_rviz_vr.sh)：只启动 fake hardware/RViz + VR，适合控制链路检查。
- [scripts/launch_usb_cameras_anycam.sh](scripts/launch_usb_cameras_anycam.sh)：只启动三路 USB 相机，适合检查设备路径、分辨率和帧率。
- [scripts/run_exo_servo_zero.sh](scripts/run_exo_servo_zero.sh)：只启动外骨骼 teleop，适合检查串口和标定。
- [scripts/record_lerobot_from_ros2.sh](scripts/record_lerobot_from_ros2.sh)：只从已有 ROS2 话题录制 LeRobot 数据集。

根目录下的 [launch_usb_cameras_anycam.sh](launch_usb_cameras_anycam.sh) 和 [run_exo_servo_zero.sh](run_exo_servo_zero.sh) 是兼容入口，会转到 `scripts/` 里的同名脚本。

## 数据集录制参数

数据集参数通常在 [scripts/launch_collect.sh](scripts/launch_collect.sh) 或 [scripts/launch_mujoco_vr.sh](scripts/launch_mujoco_vr.sh) 顶部改：

```bash
export REPO_ID="local/openarm_collection"
export NUM_EPISODES=100
export TASK_DESC="pick up the object"
export LEROBOT_VENV=""
```

含义：

- `REPO_ID`：数据集保存名，例如 `local/openarm_pick_cube`。
- `NUM_EPISODES`：录制 episode 数量。
- `TASK_DESC`：本次任务描述，会写入数据集。
- `FPS`：数据集录制频率；为空时 [scripts/record_lerobot_from_ros2.sh](scripts/record_lerobot_from_ros2.sh) 会读取相机 JSON 的 `defaults.fps`。
- 相机图像尺寸和图像话题 FPS：统一在 [src/openarm_cameras/config/openarm_cameras.json](src/openarm_cameras/config/openarm_cameras.json) 中配置，不在录制脚本里改。
- `LEROBOT_VENV`：如果 LeRobot 安装在单独虚拟环境中，填 venv 路径；为空则使用当前环境。

如果系统已经有机器人/仿真、控制源和相机话题在运行，也可以单独启动录制：

```bash
./scripts/record_lerobot_from_ros2.sh
```

单独录制时，可直接改 [scripts/record_lerobot_from_ros2.sh](scripts/record_lerobot_from_ros2.sh) 顶部参数，或临时在命令前覆盖：

```bash
REPO_ID="local/test_record" NUM_EPISODES=10 TASK_DESC="pick up the object" ./scripts/record_lerobot_from_ros2.sh
```

## 相机参数

实机相机设备优先用 `/dev/v4l/by-id/...` 或 `/dev/v4l/by-path/...`，不要只用 `/dev/video0` 这类会变化的编号。

相机参数只在 [src/openarm_cameras/config/openarm_cameras.json](src/openarm_cameras/config/openarm_cameras.json) 里改。因为 USB 相机在 ROS2 里按 video device 打开，所以不再额外写 `HEAD_PORT`，也不再把三路设备路径、分辨率和 FPS 分散写到脚本里。

更推荐的方式是用 JSON 绑定稳定硬件路径：

```bash
cp src/openarm_cameras/config/openarm_cameras.example.json src/openarm_cameras/config/openarm_cameras.json
```

然后参考 [src/openarm_cameras/config/openarm_cameras.example.json](src/openarm_cameras/config/openarm_cameras.example.json) 编辑新生成的 `src/openarm_cameras/config/openarm_cameras.json`，把三路相机写进去。

你这台 `robot_20_3060` 当前枚举结果里，三台相机都是同型号 `2M`，只有一台出现在 `/dev/v4l/by-id/`，所以更适合用 `/dev/v4l/by-path/` 绑定 USB 物理口。当前可用的三个主视频节点是：

- `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0`，当前指向 `/dev/video4`
- `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:5:1.0-video-index0`，当前指向 `/dev/video0`
- `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:6:1.0-video-index0`，当前指向 `/dev/video2`

对应的 `src/openarm_cameras/config/openarm_cameras.json` 已经可以写成：

```json
{
	"defaults": {
		"width": 640,
		"height": 480,
		"fps": 30,
		"raw_fps": 30,
		"compressed_fps": 30,
		"pixel_format": "",
		"raw_reliability": "reliable",
		"compressed_reliability": "best_effort",
		"history": "keep_last",
		"depth": 1,
		"jpeg_quality": 80,
		"auto_exposure": -1,
		"exposure_time": -1
	},
	"cameras": {
		"head": {
			"device_path": "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0",
			"width": 640,
			"height": 480,
			"fps": 30
		},
		"left": {
			"device_path": "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:5:1.0-video-index0",
			"width": 640,
			"height": 480,
			"fps": 30
		},
		"right": {
			"device_path": "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:6:1.0-video-index0",
			"width": 640,
			"height": 480,
			"fps": 30
		}
	}
}
```

`defaults` 是三路相机的默认参数；某一路相机里写了同名字段，就覆盖默认值。例如只想把头部相机改成 `1280x720@30`，只改 `head` 里的 `width` 和 `height` 即可。

这里先按 USB 口顺序临时对应为 `head`、`left`、`right`。如果实际画面发现头部/左腕/右腕对不上，不要改成 `/dev/videoN`，只需要交换 JSON 里这三条 `device_path` 的位置。

脚本顶部只保留：

```bash
export CAMERA_CONFIG=""
```

如果 `CAMERA_CONFIG` 为空，脚本会自动寻找 `src/openarm_cameras/config/openarm_cameras.json`；如果这个文件存在，就用 JSON 里的设备路径和采集参数。你也可以显式指定：

```bash
export CAMERA_CONFIG="$HOME/vla/ros2_ws/src/openarm_cameras/config/openarm_cameras.json"
```

查看稳定设备路径：

```bash
ls -l /dev/v4l/by-id/
ls -l /dev/v4l/by-path/
```

如果要看每个 `/dev/video*` 属于哪只相机，先装工具：

```bash
sudo apt install v4l-utils
```

然后运行：

```bash
v4l2-ctl --list-devices
```

典型流程是：

```bash
ls -l /dev/v4l/by-id/
ls -l /dev/v4l/by-path/
v4l2-ctl --list-devices
```

把输出里带 `video-index0`、并且指向实际图像采集节点的路径填到 JSON 的 `device_path`。有些 USB 相机会同时出现 `video-index0` 和 `video-index1`，通常 `video-index0` 才是图像流，`video-index1` 可能只是 metadata 节点。你这次输出里的 `/dev/video1`、`/dev/video3`、`/dev/video5` 就属于每个相机对应的 `video-index1`，采集时先不用它们。

选择规则：

- `/dev/v4l/by-id/...` 更偏向绑定相机硬件身份。
- `/dev/v4l/by-path/...` 更偏向绑定插在哪个 USB 物理口。
- 如果三只相机是同型号且没有唯一序列号，`by-id` 可能不好区分，这时更建议用 `by-path`。

主入口 [scripts/launch_collect.sh](scripts/launch_collect.sh) 中只配置 JSON 路径：

```bash
export CAMERA_CONFIG=""
```

单独相机脚本 [scripts/launch_usb_cameras_anycam.sh](scripts/launch_usb_cameras_anycam.sh) 中也只配置 JSON 路径：

```bash
export CAMERA_CONFIG=""
```

查看当前相机设备：

```bash
ls -l /dev/v4l/by-id/
```

## 外骨骼参数

主入口 [scripts/launch_collect.sh](scripts/launch_collect.sh) 和单独脚本 [scripts/run_exo_servo_zero.sh](scripts/run_exo_servo_zero.sh) 都使用这些核心参数：

```bash
export LEFT_PORT="/dev/ttyUSB1"
export RIGHT_PORT="/dev/ttyUSB0"
export CALIBRATION_FILE="$HOME/.openarm_exo_calibration.json"
export RECALIBRATE_ON_START="false"
```

[scripts/run_exo_servo_zero.sh](scripts/run_exo_servo_zero.sh) 还可以改：

```bash
export PUBLISH_RATE="50.0"
export CALIBRATION_ONLY="false"
```

如果更换外骨骼或标定不准，可以把 `RECALIBRATE_ON_START` 改为 `true`，重新启动后会重新标定。

## ROS2 话题约定

录制脚本和控制源默认使用下面的话题。只要实机、MuJoCo、VR、外骨骼和相机都保持这个约定，同一个录制脚本就可以跨模式使用。

- 状态：`/joint_states`
- 左臂命令：`/left_forward_position_controller/commands`
- 右臂命令：`/right_forward_position_controller/commands`
- 左夹爪：`/left_gripper_target`
- 右夹爪：`/right_gripper_target`
- 头部相机：`/cam/head/color/image_raw`
- 左腕相机：`/cam/left_wrist/color/image_raw`
- 右腕相机：`/cam/right_wrist/color/image_raw`

## DDS 运行时

脚本里默认设置：

```bash
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
```

这是为了避免 FastDDS 在订阅 `Float64MultiArray` 这类可变长度控制消息时刷出下面的日志：

```text
RTPS_READER_HISTORY Error: Change payload size ... is larger than the history payload size ...
```

如果远程机器提示找不到 `rmw_cyclonedds_cpp`，先安装：

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

## 常用组合速查

真实机械臂 + VR + 录制：

```bash
export BACKEND="real"
export CONTROL="vr"
export RECORD="true"
```

真实机械臂 + 外骨骼 + 录制：

```bash
export BACKEND="real"
export CONTROL="exo"
export RECORD="true"
```

MuJoCo + VR + 不录制：

```bash
export BACKEND="mujoco"
export CONTROL="vr"
export RECORD="false"
```

MuJoCo + 外骨骼 + 录制：

```bash
export BACKEND="mujoco"
export CONTROL="exo"
export RECORD="true"
```