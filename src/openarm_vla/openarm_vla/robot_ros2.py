from __future__ import annotations

import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras.camera import Camera
from lerobot.robots import Robot
from lerobot.robots.utils import ensure_safe_goal_position
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .robot_config_ros2 import OpenArmRos2Config
from .robot_ros2_interface import OpenArmRos2Interface
from .ros2_camera import Ros2Camera, Ros2CameraConfig


logger = logging.getLogger(__name__)


def make_cameras(camera_configs: dict[str, Any]) -> dict[str, Camera]:
    cameras: dict[str, Camera] = {}
    for key, cfg in camera_configs.items():
        if isinstance(cfg, Ros2CameraConfig):
            cameras[key] = Ros2Camera(cfg)
        else:
            from lerobot.cameras.utils import make_cameras_from_configs

            cameras[key] = make_cameras_from_configs({key: cfg})[key]
    return cameras


class OpenArmRos2(Robot):
    config_class = OpenArmRos2Config
    name = "openarm_ros2"

    def __init__(self, config: OpenArmRos2Config):
        super().__init__(config)
        self.config = config
        self.ros2 = OpenArmRos2Interface(config.ros2)
        self.cameras = make_cameras(config.cameras)
        self._all_joint_names = self.config.ros2.left_joint_names + self.config.ros2.right_joint_names

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        motor_state_ft = {f"{joint_name}.pos": float for joint_name in self._all_joint_names}
        cams_ft = {
            cam_key: (self.config.cameras[cam_key].height, self.config.cameras[cam_key].width, 3)
            for cam_key in self.cameras
        }
        return {**motor_state_ft, **cams_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{joint_name}.pos": float for joint_name in self._all_joint_names}

    @property
    def is_connected(self) -> bool:
        return self.ros2.is_connected and all(camera.is_connected for camera in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        for camera in self.cameras.values():
            camera.connect()
        self.ros2.connect()
        self.configure()

        t0 = time.time()
        while self.ros2.get_joint_positions(self._all_joint_names) is None:
            if time.time() - t0 > 5.0:
                break
            time.sleep(0.05)

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        observation: dict[str, Any] = {}
        positions = self.ros2.get_joint_positions(self._all_joint_names)
        if positions is None:
            raise ValueError("Joint state is not available yet.")

        missing = [joint_name for joint_name in self._all_joint_names if joint_name not in positions]
        if missing:
            raise ValueError(f"Missing joints in joint_states: {missing}")

        observation.update({f"{joint_name}.pos": positions[joint_name] for joint_name in self._all_joint_names})

        for cam_key, camera in self.cameras.items():
            try:
                observation[cam_key] = camera.async_read(timeout_ms=300)
            except Exception as exc:
                logger.error("Failed to read camera %s: %s", cam_key, exc)
                observation[cam_key] = None

        return observation

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        goal = {key.removesuffix(".pos"): float(value) for key, value in action.items() if key.endswith(".pos")}

        if self.config.skip_send_action:
            return {f"{joint_name}.pos": goal[joint_name] for joint_name in self._all_joint_names}

        if self.config.max_relative_target is not None:
            present = self.ros2.get_joint_positions(self._all_joint_names)
            if present is None:
                raise ValueError("Joint state is not available yet.")
            goal_present = {f"{joint_name}.pos": (goal[joint_name], present[joint_name]) for joint_name in self._all_joint_names}
            clipped = ensure_safe_goal_position(goal_present, self.config.max_relative_target)
            goal = {key.removesuffix(".pos"): value for key, value in clipped.items()}

        left_vec = [goal[joint_name] for joint_name in self.config.ros2.left_joint_names]
        right_vec = [goal[joint_name] for joint_name in self.config.ros2.right_joint_names]
        self.ros2.send_left_positions(left_vec)
        self.ros2.send_right_positions(right_vec)
        return {f"{joint_name}.pos": goal[joint_name] for joint_name in self._all_joint_names}

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        for camera in self.cameras.values():
            camera.disconnect()
        self.ros2.disconnect()