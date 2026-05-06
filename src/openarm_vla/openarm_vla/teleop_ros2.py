from __future__ import annotations

import logging
import time
from typing import Any

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.teleoperators.utils import TeleopEvents
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .teleop_config_ros2 import OpenArmRos2TeleopConfig
from .teleop_ros2_interface import OpenArmRos2TeleopInterface


logger = logging.getLogger(__name__)


class OpenArmRos2Teleop(Teleoperator):
    config_class = OpenArmRos2TeleopConfig
    name = "openarm_ros2"

    def __init__(self, config: OpenArmRos2TeleopConfig):
        super().__init__(config)
        self.config = config
        self.ros2 = OpenArmRos2TeleopInterface(config.ros2)
        self._all_joint_names = self.config.ros2.left_joint_names + self.config.ros2.right_joint_names

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{joint_name}.pos": float for joint_name in self._all_joint_names}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.ros2.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.ros2.connect()
        self.configure()

        t0 = time.time()
        while self.ros2.get_latest_joint_positions() is None:
            if time.time() - t0 > 3.0:
                break
            time.sleep(0.05)

    def calibrate(self) -> None:
        return

    @property
    def is_calibrated(self) -> bool:
        return True

    def configure(self) -> None:
        return

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        latest = self.ros2.get_latest_joint_positions()
        if latest is None:
            raise ValueError(
                "No action data available. Ensure robot is publishing /joint_states "
                "or teleop node is publishing /commands."
            )

        left, right = latest["left"], latest["right"]
        expected_left = len(self.config.ros2.left_joint_names)
        expected_right = len(self.config.ros2.right_joint_names)
        if len(left) != expected_left or len(right) != expected_right:
            raise ValueError(
                f"Teleop command length mismatch (left {len(left)}/{expected_left}, right {len(right)}/{expected_right})"
            )

        action: dict[str, float] = {}
        for name, value in zip(self.config.ros2.left_joint_names, left):
            action[f"{name}.pos"] = float(value)
        for name, value in zip(self.config.ros2.right_joint_names, right):
            action[f"{name}.pos"] = float(value)
        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.ros2.disconnect()

    def get_teleop_events(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        left_grip = self.ros2.get_grip_value("left")
        right_grip = self.ros2.get_grip_value("right")
        is_intervention = (left_grip > 0.5) or (right_grip > 0.5)
        return {
            TeleopEvents.IS_INTERVENTION: is_intervention,
            TeleopEvents.TERMINATE_EPISODE: False,
            TeleopEvents.SUCCESS: False,
            TeleopEvents.RERECORD_EPISODE: False,
        }