from __future__ import annotations

import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from lerobot.utils.errors import DeviceNotConnectedError

from .robot_config_ros2 import OpenArmRos2InterfaceConfig


class OpenArmRos2Interface:
    def __init__(self, config: OpenArmRos2InterfaceConfig):
        self.config = config
        self._node: Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._left_pub = None
        self._right_pub = None
        self._joint_state_sub = None
        self._lock = threading.Lock()
        self._last_joint_state: JointState | None = None
        self.is_connected = False

    def connect(self) -> None:
        if self.is_connected:
            return

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("openarm_lerobot_interface", namespace=self.config.namespace)
        self._left_pub = self._node.create_publisher(Float64MultiArray, self.config.left_command_topic, 10)
        self._right_pub = self._node.create_publisher(Float64MultiArray, self.config.right_command_topic, 10)
        self._joint_state_sub = self._node.create_subscription(
            JointState, self.config.joint_states_topic, self._joint_state_cb, 50
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        time.sleep(0.5)
        self.is_connected = True

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        if self._joint_state_sub is not None:
            self._joint_state_sub.destroy()
            self._joint_state_sub = None
        if self._left_pub is not None:
            self._left_pub.destroy()
            self._left_pub = None
        if self._right_pub is not None:
            self._right_pub.destroy()
            self._right_pub = None
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

    def _joint_state_cb(self, msg: JointState) -> None:
        with self._lock:
            self._last_joint_state = msg

    def get_joint_positions(self, joint_names: list[str]) -> dict[str, float] | None:
        with self._lock:
            msg = self._last_joint_state
        if msg is None:
            return None

        name_to_idx = {name: index for index, name in enumerate(msg.name)}
        out: dict[str, float] = {}
        for joint_name in joint_names:
            index = name_to_idx.get(joint_name)
            if index is None:
                continue
            out[joint_name] = float(msg.position[index])
        return out

    def send_left_positions(self, positions: list[float]) -> None:
        if not self.is_connected or self._node is None or self._left_pub is None:
            raise DeviceNotConnectedError("OpenArmRos2Interface not connected")
        msg = Float64MultiArray()
        msg.data = [float(value) for value in positions]
        self._left_pub.publish(msg)

    def send_right_positions(self, positions: list[float]) -> None:
        if not self.is_connected or self._node is None or self._right_pub is None:
            raise DeviceNotConnectedError("OpenArmRos2Interface not connected")
        msg = Float64MultiArray()
        msg.data = [float(value) for value in positions]
        self._right_pub.publish(msg)