from __future__ import annotations

import logging
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray

from .teleop_config_ros2 import OpenArmRos2TeleopInterfaceConfig


logger = logging.getLogger(__name__)


class OpenArmRos2TeleopInterface:
    def __init__(self, config: OpenArmRos2TeleopInterfaceConfig):
        self.config = config
        self._node: Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._left_sub = None
        self._right_sub = None
        self._joint_state_sub = None
        self._left_grip_sub = None
        self._right_grip_sub = None
        self._lock = threading.Lock()
        self._last_left: list[float] | None = None
        self._last_right: list[float] | None = None
        self._joint_states: dict[str, float] = {}
        self._left_grip_value: float = 0.0
        self._right_grip_value: float = 0.0
        self._warned_left_len = False
        self._warned_right_len = False
        self.is_connected = False

    def connect(self) -> None:
        if self.is_connected:
            return

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("openarm_lerobot_teleop_interface", namespace=self.config.namespace)
        self._left_sub = self._node.create_subscription(
            Float64MultiArray, self.config.left_command_topic, self._left_cb, 5
        )
        self._right_sub = self._node.create_subscription(
            Float64MultiArray, self.config.right_command_topic, self._right_cb, 5
        )
        self._joint_state_sub = self._node.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 5
        )
        self._left_grip_sub = self._node.create_subscription(
            Float32, "/pico_left_controller/grip", self._left_grip_cb, 5
        )
        self._right_grip_sub = self._node.create_subscription(
            Float32, "/pico_right_controller/grip", self._right_grip_cb, 5
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        time.sleep(0.2)
        self.is_connected = True

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        if self._left_sub is not None:
            self._left_sub.destroy()
            self._left_sub = None
        if self._right_sub is not None:
            self._right_sub.destroy()
            self._right_sub = None
        if self._joint_state_sub is not None:
            self._joint_state_sub.destroy()
            self._joint_state_sub = None
        if self._left_grip_sub is not None:
            self._left_grip_sub.destroy()
            self._left_grip_sub = None
        if self._right_grip_sub is not None:
            self._right_grip_sub.destroy()
            self._right_grip_sub = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._executor is not None:
            self._executor.shutdown()
            self._executor = None
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            self._spin_thread = None

        self.is_connected = False

    def _left_cb(self, msg: Float64MultiArray) -> None:
        data = [float(value) for value in msg.data]
        expected = len(self.config.left_joint_names)
        if expected and len(data) != expected:
            if len(data) < expected:
                missing_names = self.config.left_joint_names[len(data):]
                for name in missing_names:
                    if name in self._joint_states:
                        data.append(self._joint_states[name])
                    else:
                        if not self._warned_left_len:
                            logger.warning(
                                "Left command length %s < expected %s, and joint_state '%s' not available for padding",
                                len(msg.data),
                                expected,
                                name,
                            )
                            self._warned_left_len = True
                        return
            else:
                if not self._warned_left_len:
                    logger.warning(
                        "Left command length %s does not match expected joint count %s; ignoring message",
                        len(data),
                        expected,
                    )
                    self._warned_left_len = True
                return
        with self._lock:
            self._last_left = data

    def _right_cb(self, msg: Float64MultiArray) -> None:
        data = [float(value) for value in msg.data]
        expected = len(self.config.right_joint_names)
        if expected and len(data) != expected:
            if len(data) < expected:
                missing_names = self.config.right_joint_names[len(data):]
                for name in missing_names:
                    if name in self._joint_states:
                        data.append(self._joint_states[name])
                    else:
                        if not self._warned_right_len:
                            logger.warning(
                                "Right command length %s < expected %s, and joint_state '%s' not available for padding",
                                len(msg.data),
                                expected,
                                name,
                            )
                            self._warned_right_len = True
                        return
            else:
                if not self._warned_right_len:
                    logger.warning(
                        "Right command length %s does not match expected joint count %s; ignoring message",
                        len(data),
                        expected,
                    )
                    self._warned_right_len = True
                return
        with self._lock:
            self._last_right = data

    def _joint_state_cb(self, msg: JointState) -> None:
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self._joint_states[name] = float(position)

    def _left_grip_cb(self, msg: Float32) -> None:
        with self._lock:
            self._left_grip_value = float(msg.data)

    def _right_grip_cb(self, msg: Float32) -> None:
        with self._lock:
            self._right_grip_value = float(msg.data)

    def get_grip_value(self, side: str) -> float:
        with self._lock:
            if side == "left":
                return self._left_grip_value
            if side == "right":
                return self._right_grip_value
            raise ValueError(f"Unknown side: {side}")

    def _get_fallback_from_joint_states(self, joint_names: list[str]) -> list[float] | None:
        with self._lock:
            if not self._joint_states:
                return None
            result = []
            for name in joint_names:
                if name not in self._joint_states:
                    return None
                result.append(self._joint_states[name])
            return result

    def get_latest_joint_positions(self) -> dict[str, list[float]] | None:
        with self._lock:
            left = None if self._last_left is None else list(self._last_left)
            right = None if self._last_right is None else list(self._last_right)

        if left is not None and right is not None:
            return {"left": left, "right": right}

        if left is None:
            left = self._get_fallback_from_joint_states(self.config.left_joint_names)
        if right is None:
            right = self._get_fallback_from_joint_states(self.config.right_joint_names)

        if left is None or right is None:
            return None

        return {"left": left, "right": right}