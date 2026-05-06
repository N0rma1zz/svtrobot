# vr_teleop

OpenArm VR remote-teleop ROS2 package.
Consolidated from upstream:

- `XRoboToolkit-Teleop-Sample-Cpp/main_openarm.cpp` -> `src/teleop_openarm.cpp`
- `XRoboToolkit-Orin-Video-Sender/video_sender_pc.cpp` -> `src/video_sender.cpp`
- `XRoboToolkit-Teleop-Sample-Cpp/lib/{PXREARobotSDK.h, libPXREARobotSDK.so}`
  -> `include/PXREARobotSDK.h`, `lib/libPXREARobotSDK.so`

The UR5 / ZED / Windows / asio variants are dropped on purpose.

## Executables

| Binary           | Purpose                                                    |
|------------------|------------------------------------------------------------|
| `teleop_openarm` | VR controller pose -> KDL IK -> /<side>_forward_position_controller/commands + /<side>_gripper_target |
| `video_sender`   | Subscribes 3 `sensor_msgs/Image` topics, encodes H264, listens on TCP for the Pico headset |

## Build

```bash
cd ~/vla/ros2_ws
colcon build --packages-select vr_teleop --symlink-install
source install/setup.bash
```

System deps (Ubuntu 22.04):

```bash
sudo apt install -y \
  libeigen3-dev nlohmann-json3-dev \
  ros-humble-orocos-kdl-vendor ros-humble-kdl-parser ros-humble-urdf \
  ros-humble-control-msgs ros-humble-cv-bridge \
  libopencv-dev \
  libavcodec-dev libavutil-dev libswscale-dev pkg-config
```

## Run

```bash
# Terminal 1 - VR teleop
ros2 run vr_teleop teleop_openarm --urdf $HOME/vla/ros2_ws/openarm_bimanual_control.urdf

# Terminal 2 - Video stream to Pico
ros2 run vr_teleop video_sender -- --listen 0.0.0.0:13579 --ros \
  --topic  /cam/head/color/image_raw \
  --topic2 /cam/left_wrist/color/image_raw \
  --topic3 /cam/right_wrist/color/image_raw
```

The full data-collection orchestration is in
`ros2_ws/scripts/launch_mujoco_vr.sh`.
