from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np
import rclpy
from numpy.typing import NDArray
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from lerobot.cameras.camera import Camera
from lerobot.cameras.configs import CameraConfig, ColorMode, Cv2Rotation
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError


logger = logging.getLogger(__name__)


def get_cv2_rotation(rotation: Cv2Rotation) -> int | None:
    if rotation == Cv2Rotation.NO_ROTATION:
        return None
    if rotation == Cv2Rotation.ROTATE_90:
        return cv2.ROTATE_90_CLOCKWISE
    if rotation == Cv2Rotation.ROTATE_180:
        return cv2.ROTATE_180
    if rotation == Cv2Rotation.ROTATE_270:
        return cv2.ROTATE_90_COUNTERCLOCKWISE
    return None


@CameraConfig.register_subclass("openarm_ros2_topic")
@dataclass(kw_only=True)
class Ros2CameraConfig(CameraConfig):
    image_topic: str = "/camera/color/image_raw"
    color_mode: ColorMode = ColorMode.RGB
    rotation: Cv2Rotation = Cv2Rotation.NO_ROTATION
    use_depth: bool = False
    depth_topic: str = "/camera/depth/image_raw"
    qos_reliability: str = "best_effort"
    queue_size: int = 1


class Ros2Camera(Camera):
    def __init__(self, config: Ros2CameraConfig):
        super().__init__(config)
        self.config = config
        self._node: Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._image_sub = None
        self._depth_sub = None
        self._lock = threading.Lock()
        self._latest_frame: NDArray[Any] | None = None
        self._latest_depth: NDArray[Any] | None = None
        self._new_frame_event = threading.Event()
        self._is_connected = False
        self.rotation: int | None = get_cv2_rotation(config.rotation)

    def __str__(self) -> str:
        return f"Ros2Camera({self.config.image_topic})"

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self, warmup: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")

        if not rclpy.ok():
            rclpy.init()

        node_name = f"ros2_camera_{self.config.image_topic.replace('/', '_').strip('_')}"
        self._node = Node(node_name)

        reliability = ReliabilityPolicy.RELIABLE
        if self.config.qos_reliability != "reliable":
            reliability = ReliabilityPolicy.BEST_EFFORT
        qos_profile = QoSProfile(
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.config.queue_size,
        )

        self._image_sub = self._node.create_subscription(
            Image,
            self.config.image_topic,
            self._image_callback,
            qos_profile,
        )
        if self.config.use_depth:
            self._depth_sub = self._node.create_subscription(
                Image,
                self.config.depth_topic,
                self._depth_callback,
                qos_profile,
            )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._is_connected = True

        if warmup:
            t0 = time.time()
            while self._latest_frame is None:
                if time.time() - t0 > 10.0:
                    logger.warning("%s timeout waiting for first frame.", self)
                    break
                time.sleep(0.1)

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self._image_sub is not None:
            self._node.destroy_subscription(self._image_sub)
            self._image_sub = None
        if self._depth_sub is not None:
            self._node.destroy_subscription(self._depth_sub)
            self._depth_sub = None
        if self._executor is not None:
            self._executor.shutdown()
            self._executor = None
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            self._spin_thread = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None

        self._is_connected = False
        self._latest_frame = None
        self._latest_depth = None

    def _image_callback(self, msg: Image) -> None:
        frame = self._ros_image_to_numpy(msg)
        frame = self._postprocess_image(frame)
        with self._lock:
            self._latest_frame = frame
            self._new_frame_event.set()

    def _depth_callback(self, msg: Image) -> None:
        with self._lock:
            self._latest_depth = self._ros_image_to_numpy(msg)

    def _ros_image_to_numpy(self, msg: Image) -> NDArray[Any]:
        if msg.encoding in ("rgb8", "bgr8"):
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif msg.encoding in ("mono8", "8UC1"):
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        elif msg.encoding in ("16UC1", "mono16"):
            frame = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        else:
            raise ValueError(f"Unsupported ROS image encoding: {msg.encoding}")
        return frame.copy()

    def _postprocess_image(self, frame: NDArray[Any]) -> NDArray[Any]:
        if frame.ndim == 3 and self.config.color_mode == ColorMode.RGB:
            if frame.shape[2] == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if self.rotation is not None:
            frame = cv2.rotate(frame, self.rotation)
        return frame

    def async_read(self, timeout_ms: int = 0) -> NDArray[Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if timeout_ms > 0 and self._latest_frame is None:
            self._new_frame_event.wait(timeout_ms / 1000.0)
        with self._lock:
            if self._latest_frame is None:
                raise TimeoutError(f"No frame available from {self}")
            return self._latest_frame.copy()