#!/usr/bin/env python3
"""Display MuJoCo bridge camera images in real-time OpenCV windows.

Subscribes to 3 camera ROS2 topics and shows them in a tiled window.

Usage:
    source /opt/ros/humble/setup.bash
    source .venv/bin/activate
    python examples/OpenArm/debug/camera_viewer.py
"""

import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraViewer(Node):
    TOPICS = [
        "/cam/head/color/image_raw",
        "/cam/left_wrist/color/image_raw",
        "/cam/right_wrist/color/image_raw",
    ]
    LABELS = ["Head", "Left Wrist", "Right Wrist"]

    def __init__(self):
        super().__init__("camera_viewer")
        self._lock = threading.Lock()
        self._images = [None] * len(self.TOPICS)

        for i, topic in enumerate(self.TOPICS):
            self.create_subscription(
                Image, topic,
                lambda msg, idx=i: self._on_image(msg, idx),
                1,
            )
        self.get_logger().info(f"Subscribed to {len(self.TOPICS)} camera topics")

    def _on_image(self, msg: Image, idx: int):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )
        with self._lock:
            self._images[idx] = img

    def get_tiled(self) -> np.ndarray | None:
        with self._lock:
            imgs = list(self._images)
        if all(im is None for im in imgs):
            return None
        h, w = 480, 640
        for i, im in enumerate(imgs):
            if im is None:
                imgs[i] = np.zeros((h, w, 3), dtype=np.uint8)
        # Add labels
        labeled = []
        for im, label in zip(imgs, self.LABELS):
            im_bgr = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
            cv2.putText(im_bgr, label, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            labeled.append(im_bgr)
        # Tile: head on top, left+right on bottom
        bottom = np.hstack([labeled[1], labeled[2]])
        top = cv2.resize(labeled[0], (bottom.shape[1], labeled[0].shape[0]))
        return np.vstack([top, bottom])


def main():
    rclpy.init()
    node = CameraViewer()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("Camera Viewer — press 'q' to quit")
    while rclpy.ok():
        tiled = node.get_tiled()
        if tiled is not None:
            cv2.imshow("OpenArm Cameras", tiled)
        key = cv2.waitKey(33)
        if key == ord("q"):
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
