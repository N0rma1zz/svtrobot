# openarm_vla

这个包把 OpenArm 的 VLA 数据采集最小能力收口进当前 ros2_ws：

1. LeRobot Robot 插件：`openarm_follower_ros2`
2. LeRobot Teleoperator 插件：`openarm_leader_ros2`
3. 三路 RealSense 相机发布 launch：`camera_publisher.launch.py`
4. 三路 USB 相机包：`openarm_cameras`
5. 外骨骼控制包：`openarm_exoskeleton`

默认 ROS2 契约：

- 机械臂状态：`/joint_states`
- 控制命令：`/left_forward_position_controller/commands`、`/right_forward_position_controller/commands`
- 图像：`/cam/head/color/image_raw`、`/cam/left_wrist/color/image_raw`、`/cam/right_wrist/color/image_raw`

说明：

- `camera_publisher.launch.py` 只适合 RealSense 相机。
- 如果你用 USB/UVC 相机，优先改 `src/openarm_cameras/config/openarm_cameras.json`，然后运行 `scripts/launch_usb_cameras_anycam.sh`。
- 如果你用外骨骼作为控制源，优先走 `ros2 launch openarm_exoskeleton exo_teleop.launch.py ...`。

常用运行方式：

```bash
cd ~/vla/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select openarm_vla
source install/setup.bash

ros2 launch openarm_vla camera_publisher.launch.py \
  width:=640 height:=480 fps:=15 \
  cam_left_serial:=<left_serial> cam_left_type:=D405 \
  cam_right_serial:=<right_serial> cam_right_type:=D405 \
  cam_head_serial:=<head_serial> cam_head_type:=D435
```

USB/UVC 相机的设备路径、三路分辨率和 FPS 统一写在 `src/openarm_cameras/config/openarm_cameras.json`，不要在脚本里分散改：

```bash
./scripts/launch_usb_cameras_anycam.sh
```

LeRobot 录制建议直接走 [ros2_ws/scripts/record_lerobot_from_ros2.sh](../../scripts/record_lerobot_from_ros2.sh)。

更完整的真机 / MuJoCo / VR / 外骨骼采集顺序见 [ros2_ws/COLLECTION_WORKFLOWS.md](../../COLLECTION_WORKFLOWS.md)。