#!/usr/bin/env python3

import argparse
import array
import shutil
import subprocess
import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CompressedImage, Image


def build_qos(reliability, history, depth):
    reliability_policy = {
        "best_effort": ReliabilityPolicy.BEST_EFFORT,
        "reliable": ReliabilityPolicy.RELIABLE,
    }[reliability]
    history_policy = {
        "keep_last": HistoryPolicy.KEEP_LAST,
        "keep_all": HistoryPolicy.KEEP_ALL,
    }[history]

    qos = QoSProfile(depth=max(depth, 1))
    qos.reliability = reliability_policy
    qos.history = history_policy
    return qos


def open_capture(device_path, device_id):
    candidates = []
    if device_path:
        candidates.append(device_path)
    if device_id not in {None, "", "None"}:
        try:
            candidates.append(int(device_id))
        except ValueError:
            pass

    last_error = None
    for candidate in candidates:
        capture = cv2.VideoCapture(candidate, cv2.CAP_V4L2)
        if capture.isOpened():
            return capture
        last_error = candidate
        capture.release()

    raise RuntimeError(f"Could not open camera using {last_error!r}")


class OpenCVCameraPublisher(Node):
    def __init__(self, args):
        super().__init__(f"{args.camera_name}_opencv_pub", namespace=args.namespace)
        self.frame_id = args.frame_id
        self.capture_fps = args.fps
        self.frame_period = 1.0 / args.fps if args.fps > 0 else 0.0
        self.jpeg_quality = max(1, min(args.jpeg_quality, 100))
        self.raw_fps = args.raw_fps
        self.compressed_fps = args.compressed_fps
        if self.raw_fps <= 0:
            raise ValueError("raw_fps must be > 0")
        if self.compressed_fps <= 0:
            raise ValueError("compressed_fps must be > 0")

        raw_qos = build_qos(args.raw_reliability, args.history, args.depth)
        compressed_qos = build_qos(args.compressed_reliability, args.history, args.depth)

        self.publisher = self.create_publisher(Image, args.image_topic, raw_qos)
        self.compressed_publisher = self.create_publisher(
            CompressedImage,
            f"{args.image_topic}/compressed",
            compressed_qos,
        )

        self.capture = open_capture(args.device_path, args.device_id)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        if args.pixel_format:
            fourcc = cv2.VideoWriter_fourcc(*args.pixel_format[:4])
            self.capture.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        self.capture.set(cv2.CAP_PROP_FPS, args.fps)
        self._apply_exposure(args)

        self.raw_publish_count = 0
        self.compressed_publish_count = 0
        self.last_log_time = time.monotonic()
        self.next_frame_time = time.monotonic()
        self.next_raw_publish_time = self.next_frame_time
        self.next_compressed_publish_time = self.next_frame_time

        self.get_logger().info(
            "OpenCV publisher started on %s/%s (%sx%s, capture %.2f fps, raw %.2f fps, compressed %.2f fps)"
            % (
                args.namespace.rstrip("/"),
                args.image_topic,
                args.width,
                args.height,
                args.fps,
                args.raw_fps,
                args.compressed_fps,
            )
        )

    def _apply_exposure(self, args):
        if args.auto_exposure < 0 and args.exposure_time < 0:
            return
        if not shutil.which("v4l2-ctl"):
            if args.auto_exposure >= 0:
                self.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, args.auto_exposure)
            if args.exposure_time >= 0:
                self.capture.set(cv2.CAP_PROP_EXPOSURE, args.exposure_time)
            return
        device = args.device_path or f"/dev/video{args.device_id}"
        ctrls = []
        if args.auto_exposure >= 0:
            ctrls.append(f"auto_exposure={args.auto_exposure}")
        if args.exposure_time >= 0:
            ctrls.append(f"exposure_time_absolute={args.exposure_time}")
        try:
            subprocess.run(
                ["v4l2-ctl", f"--device={device}", f"--set-ctrl={','.join(ctrls)}"],
                check=True,
                capture_output=True,
                timeout=5,
            )
            self.get_logger().info(f"Set exposure via v4l2-ctl: {', '.join(ctrls)}")
        except Exception as exc:
            self.get_logger().warning(f"v4l2-ctl exposure failed: {exc}")

    def publish_once(self):
        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warning("Failed to read frame from camera")
            time.sleep(0.05)
            return

        stamp = self.get_clock().now().to_msg()
        height, width = frame.shape[:2]
        now = time.monotonic()

        if now >= self.next_raw_publish_time:
            image_msg = Image()
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            image_msg.height = height
            image_msg.width = width
            image_msg.encoding = "bgr8"
            image_msg.is_bigendian = False
            image_msg.step = width * 3
            image_msg.data = array.array("B", frame.tobytes())
            self.publisher.publish(image_msg)
            self.raw_publish_count += 1
            self.next_raw_publish_time = max(
                self.next_raw_publish_time + (1.0 / self.raw_fps),
                now,
            )

        if now >= self.next_compressed_publish_time:
            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            if ok:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = stamp
                compressed_msg.header.frame_id = self.frame_id
                compressed_msg.format = "jpeg"
                compressed_msg.data = array.array("B", encoded.tobytes())
                self.compressed_publisher.publish(compressed_msg)
                self.compressed_publish_count += 1
                self.next_compressed_publish_time = max(
                    self.next_compressed_publish_time + (1.0 / self.compressed_fps),
                    now,
                )

        if now - self.last_log_time >= 2.0:
            elapsed = now - self.last_log_time
            raw_rate = self.raw_publish_count / elapsed if elapsed > 0 else 0.0
            compressed_rate = self.compressed_publish_count / elapsed if elapsed > 0 else 0.0
            self.get_logger().info(
                f"Publishing raw={raw_rate:.2f} fps compressed={compressed_rate:.2f} fps"
            )
            self.raw_publish_count = 0
            self.compressed_publish_count = 0
            self.last_log_time = now

    def run(self):
        try:
            while rclpy.ok():
                now = time.monotonic()
                if self.frame_period > 0 and now < self.next_frame_time:
                    time.sleep(self.next_frame_time - now)
                try:
                    self.publish_once()
                except Exception:
                    if not rclpy.ok():
                        break
                    raise
                if self.frame_period > 0:
                    self.next_frame_time = max(self.next_frame_time + self.frame_period, time.monotonic())
                rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            self.capture.release()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="OpenCV-based ROS2 image publisher")
    parser.add_argument("--camera-name", required=True)
    parser.add_argument("--device-path", required=True)
    parser.add_argument("--device-id", default="")
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--fps", type=float, required=True)
    parser.add_argument("--frame-id", required=True)
    parser.add_argument("--namespace", required=True)
    parser.add_argument("--image-topic", required=True)
    parser.add_argument("--pixel-format", default="")
    parser.add_argument("--reliability", choices=["best_effort", "reliable"], default=None)
    parser.add_argument("--raw-reliability", choices=["best_effort", "reliable"], default=None)
    parser.add_argument("--compressed-reliability", choices=["best_effort", "reliable"], default=None)
    parser.add_argument("--history", choices=["keep_last", "keep_all"], default="keep_last")
    parser.add_argument("--depth", type=int, default=1)
    parser.add_argument("--raw-fps", type=float, default=10.0)
    parser.add_argument("--compressed-fps", type=float, default=30.0)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--auto-exposure", type=int, default=-1)
    parser.add_argument("--exposure-time", type=int, default=-1)
    args = parser.parse_args(argv)
    if args.raw_reliability is None:
        args.raw_reliability = args.reliability or "reliable"
    if args.compressed_reliability is None:
        args.compressed_reliability = args.reliability or "best_effort"
    return args


def main(args=None):
    cli_args = remove_ros_args(args=sys.argv if args is None else args)[1:]
    parsed_args = parse_args(cli_args)
    rclpy.init(args=args)
    node = None
    try:
        node = OpenCVCameraPublisher(parsed_args)
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"OpenCV publisher failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())