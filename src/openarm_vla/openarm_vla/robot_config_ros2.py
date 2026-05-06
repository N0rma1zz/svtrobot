from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.robots import RobotConfig

from .ros2_camera import Ros2CameraConfig


@dataclass
class OpenArmRos2InterfaceConfig:
    namespace: str = ""
    joint_states_topic: str = "/joint_states"
    left_command_topic: str = "/left_forward_position_controller/commands"
    right_command_topic: str = "/right_forward_position_controller/commands"
    left_joint_names: list[str] = field(
        default_factory=lambda: [
            "openarm_left_joint1",
            "openarm_left_joint2",
            "openarm_left_joint3",
            "openarm_left_joint4",
            "openarm_left_joint5",
            "openarm_left_joint6",
            "openarm_left_joint7",
            "openarm_left_finger_joint1",
        ]
    )
    right_joint_names: list[str] = field(
        default_factory=lambda: [
            "openarm_right_joint1",
            "openarm_right_joint2",
            "openarm_right_joint3",
            "openarm_right_joint4",
            "openarm_right_joint5",
            "openarm_right_joint6",
            "openarm_right_joint7",
            "openarm_right_finger_joint1",
        ]
    )


@RobotConfig.register_subclass("openarm_follower_ros2")
@dataclass
class OpenArmRos2Config(RobotConfig):
    max_relative_target: float | dict[str, float] | None = None
    skip_send_action: bool = True
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "cam_head": Ros2CameraConfig(
                image_topic="/cam/head/color/image_raw",
                fps=15,
                width=640,
                height=480,
                color_mode=ColorMode.RGB,
                use_depth=False,
                rotation=Cv2Rotation.NO_ROTATION,
                qos_reliability="reliable",
                queue_size=1,
            ),
            "cam_left_wrist": Ros2CameraConfig(
                image_topic="/cam/left_wrist/color/image_raw",
                fps=15,
                width=640,
                height=480,
                color_mode=ColorMode.RGB,
                use_depth=False,
                rotation=Cv2Rotation.NO_ROTATION,
                qos_reliability="reliable",
                queue_size=1,
            ),
            "cam_right_wrist": Ros2CameraConfig(
                image_topic="/cam/right_wrist/color/image_raw",
                fps=15,
                width=640,
                height=480,
                color_mode=ColorMode.RGB,
                use_depth=False,
                rotation=Cv2Rotation.NO_ROTATION,
                qos_reliability="reliable",
                queue_size=1,
            ),
        }
    )
    ros2: OpenArmRos2InterfaceConfig = field(default_factory=OpenArmRos2InterfaceConfig)


from .robot_ros2 import OpenArmRos2  # noqa: E402,F401