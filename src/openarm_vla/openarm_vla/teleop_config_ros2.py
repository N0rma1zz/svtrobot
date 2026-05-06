from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig


@dataclass
class OpenArmRos2TeleopInterfaceConfig:
    namespace: str = ""
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


@TeleoperatorConfig.register_subclass("openarm_leader_ros2")
@dataclass
class OpenArmRos2TeleopConfig(TeleoperatorConfig):
    ros2: OpenArmRos2TeleopInterfaceConfig = field(default_factory=OpenArmRos2TeleopInterfaceConfig)


from .teleop_ros2 import OpenArmRos2Teleop  # noqa: E402,F401