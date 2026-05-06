#!/usr/bin/env python3

import json
import os
import re
import threading
import time

import numpy as np
import rclpy
import serial
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


DEFAULT_JOINT_BIAS_DEG = np.zeros(7, dtype=float)
DEFAULT_LEFT_JOINT_MAP_MATRIX = np.eye(7, dtype=float)
DEFAULT_LEFT_JOINT_MAP_MATRIX[3, 3] = -1.0
DEFAULT_RIGHT_JOINT_MAP_MATRIX = np.eye(7, dtype=float)
DEFAULT_RIGHT_JOINT_MAP_MATRIX[1, 1] = -1.0

ARM_JOINT_COUNT = 7
GRIPPER_OPEN_POSITION = 0.044


def pwm_to_angle(response_str, pwm_min=500, pwm_max=2500, angle_range=270.0):
    match = re.search(r"P(\d{4})", response_str)
    if not match:
        return None
    pwm_val = int(match.group(1))
    pwm_span = pwm_max - pwm_min
    return (pwm_val - pwm_min) / pwm_span * angle_range


def is_valid_prad_response(response_str):
    if not response_str:
        return False
    return pwm_to_angle(response_str.strip()) is not None


def send_serial_command(serial_handle, cmd, command_settle_sec, response_retries):
    response = ""
    retry_count = max(1, int(response_retries))
    expects_prad = cmd.endswith("PRAD!")

    for _ in range(retry_count):
        serial_handle.reset_input_buffer()
        serial_handle.write(cmd.encode("ascii"))
        serial_handle.flush()
        if command_settle_sec > 0.0:
            time.sleep(command_settle_sec)
        response = serial_handle.read_until(b"!").decode("ascii", errors="ignore")
        if response and (not expects_prad or is_valid_prad_response(response)):
            return response

    return response


class ServoArmReader:
    def __init__(
        self,
        serial_handle,
        servo_ids,
        name,
        logger,
        max_angle_jump_deg,
        command_settle_sec,
        response_retries,
    ):
        self.ser = serial_handle
        self.servo_ids = list(servo_ids)
        self.name = name
        self.logger = logger
        self.max_angle_jump_deg = float(max_angle_jump_deg)
        self.command_settle_sec = float(command_settle_sec)
        self.response_retries = int(response_retries)
        self.zero_angles = np.zeros(ARM_JOINT_COUNT, dtype=float)
        self.angle_pos_deg = np.zeros(ARM_JOINT_COUNT, dtype=float)
        self.angle_offset_deg = np.zeros(ARM_JOINT_COUNT, dtype=float)
        self.has_valid_reading = [False] * ARM_JOINT_COUNT

    def send_command(self, cmd):
        return send_serial_command(
            self.ser,
            cmd,
            self.command_settle_sec,
            self.response_retries,
        )

    def initialize(self):
        response = self.send_command("#000PVER!")
        for servo_id in self.servo_ids:
            release_response = self.send_command(f"#{servo_id:03d}PULK!")
            self.logger.info(
                f"{self.name} servo {servo_id} torque released: {release_response.strip()}"
            )
        self.logger.info(f"{self.name} version response: {response.strip()}")

    def calibrate_zero_angles(self):
        self.logger.info(f"{self.name} 正在读取初始零点 {self.servo_ids}...")
        calibrated_angles = np.zeros(ARM_JOINT_COUNT, dtype=float)

        for local_index, servo_id in enumerate(self.servo_ids):
            response = self.send_command(f"#{servo_id:03d}PRAD!")
            angle = pwm_to_angle(response.strip())

            if angle is not None:
                calibrated_angles[local_index] = angle
            else:
                calibrated_angles[local_index] = 0.0
                self.logger.warn(
                    f"{self.name} servo {servo_id} 标定读取失败，使用 0.0 作为零点"
                )

        self.zero_angles = calibrated_angles
        self.logger.info(f"{self.name} 零点校准完成: {self.zero_angles.tolist()}")

    def set_zero_angles(self, zero_angles):
        self.zero_angles = np.array(zero_angles, dtype=float)
        self.logger.info(f"{self.name} 已加载零点: {self.zero_angles.tolist()}")

    def read_arm_positions(self):
        for local_index, servo_id in enumerate(self.servo_ids):
            response = self.send_command(f"#{servo_id:03d}PRAD!")
            angle = pwm_to_angle(response.strip())
            if angle is None:
                self.logger.warn(
                    f"{self.name} servo {servo_id} PRAD response incomplete, keep last value",
                    throttle_duration_sec=2.0,
                )
                continue

            if self.has_valid_reading[local_index]:
                angle_jump = abs(angle - self.angle_pos_deg[local_index])
                if angle_jump > self.max_angle_jump_deg:
                    self.logger.warn(
                        f"{self.name} servo {servo_id} angle jump too large: {self.angle_pos_deg[local_index]:.3f} -> {angle:.3f} deg, drop this sample"
                    )
                    continue

            self.angle_pos_deg[local_index] = angle
            self.has_valid_reading[local_index] = True
            self.angle_offset_deg[local_index] = angle - self.zero_angles[local_index]

        return self.angle_offset_deg.copy(), self.angle_pos_deg.copy()


class ServoGripperReader:
    def __init__(
        self,
        serial_handle,
        servo_id,
        name,
        logger,
        max_angle_jump_deg,
        command_settle_sec,
        response_retries,
    ):
        self.ser = serial_handle
        self.servo_id = int(servo_id)
        self.name = name
        self.logger = logger
        self.max_angle_jump_deg = float(max_angle_jump_deg)
        self.command_settle_sec = float(command_settle_sec)
        self.response_retries = int(response_retries)
        self.zero_angle_deg = 0.0
        self.current_angle_deg = 0.0
        self.has_valid_reading = False

    def send_command(self, cmd):
        return send_serial_command(
            self.ser,
            cmd,
            self.command_settle_sec,
            self.response_retries,
        )

    def initialize(self):
        response = self.send_command(f"#{self.servo_id:03d}PULK!")
        self.logger.info(
            f"{self.name} gripper servo {self.servo_id} torque released: {response.strip()}"
        )

    def calibrate_zero_angle(self):
        response = self.send_command(f"#{self.servo_id:03d}PRAD!")
        angle = pwm_to_angle(response.strip())
        if angle is not None:
            self.zero_angle_deg = angle
            self.current_angle_deg = angle
            self.has_valid_reading = True
        else:
            self.zero_angle_deg = 0.0
            self.logger.warn(
                f"{self.name} gripper servo {self.servo_id} 标定读取失败，使用 0.0 作为零点"
            )
        self.logger.info(
            f"{self.name} gripper servo {self.servo_id} 零点校准完成: {self.zero_angle_deg:.3f}"
        )

    def set_zero_angle(self, zero_angle_deg):
        self.zero_angle_deg = float(zero_angle_deg)
        self.current_angle_deg = float(zero_angle_deg)
        self.has_valid_reading = True
        self.logger.info(
            f"{self.name} gripper servo {self.servo_id} 已加载零点: {self.zero_angle_deg:.3f}"
        )

    def read_angle_deg(self):
        response = self.send_command(f"#{self.servo_id:03d}PRAD!")
        angle = pwm_to_angle(response.strip())
        if angle is None:
            self.logger.warn(
                f"{self.name} gripper servo {self.servo_id} PRAD response incomplete, keep last value",
                throttle_duration_sec=2.0,
            )
            return self.current_angle_deg, self.current_angle_deg - self.zero_angle_deg

        if self.has_valid_reading:
            angle_jump = abs(angle - self.current_angle_deg)
            if angle_jump > self.max_angle_jump_deg:
                self.logger.warn(
                    f"{self.name} gripper servo {self.servo_id} angle jump too large: {self.current_angle_deg:.3f} -> {angle:.3f} deg, drop this sample"
                )
                return self.current_angle_deg, self.current_angle_deg - self.zero_angle_deg

        self.current_angle_deg = angle
        self.has_valid_reading = True
        return angle, angle - self.zero_angle_deg


class ExoSideReader:
    def __init__(
        self,
        serial_handle,
        servo_ids,
        gripper_servo_id,
        name,
        logger,
        max_angle_jump_deg,
        command_settle_sec,
        poll_interval_sec,
        response_retries,
    ):
        self.name = name
        self.logger = logger
        self.poll_interval_sec = float(poll_interval_sec)
        self.arm_reader = ServoArmReader(
            serial_handle=serial_handle,
            servo_ids=servo_ids,
            name=name,
            logger=logger,
            max_angle_jump_deg=max_angle_jump_deg,
            command_settle_sec=command_settle_sec,
            response_retries=response_retries,
        )
        self.gripper_reader = ServoGripperReader(
            serial_handle=serial_handle,
            servo_id=gripper_servo_id,
            name=name,
            logger=logger,
            max_angle_jump_deg=max_angle_jump_deg,
            command_settle_sec=command_settle_sec,
            response_retries=response_retries,
        )
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._has_sample = False
        self._arm_offset_deg = np.zeros(ARM_JOINT_COUNT, dtype=float)
        self._arm_raw_deg = np.zeros(ARM_JOINT_COUNT, dtype=float)
        self._gripper_raw_deg = 0.0
        self._gripper_offset_deg = 0.0

    def initialize(self):
        self.arm_reader.initialize()
        self.gripper_reader.initialize()

    def calibrate_zero(self):
        self.arm_reader.calibrate_zero_angles()
        self.gripper_reader.calibrate_zero_angle()

    def set_zero(self, arm_zero_angles, gripper_zero_angle):
        self.arm_reader.set_zero_angles(arm_zero_angles)
        self.gripper_reader.set_zero_angle(gripper_zero_angle)

    def start(self):
        if self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def get_latest(self):
        with self._lock:
            return {
                "has_sample": self._has_sample,
                "arm_offset_deg": self._arm_offset_deg.copy(),
                "arm_raw_deg": self._arm_raw_deg.copy(),
                "gripper_raw_deg": float(self._gripper_raw_deg),
                "gripper_offset_deg": float(self._gripper_offset_deg),
            }

    def _poll_loop(self):
        while not self._stop_event.is_set():
            arm_offset_deg, arm_raw_deg = self.arm_reader.read_arm_positions()
            gripper_raw_deg, gripper_offset_deg = self.gripper_reader.read_angle_deg()

            with self._lock:
                self._arm_offset_deg = arm_offset_deg
                self._arm_raw_deg = arm_raw_deg
                self._gripper_raw_deg = gripper_raw_deg
                self._gripper_offset_deg = gripper_offset_deg
                self._has_sample = True

            if self.poll_interval_sec > 0.0:
                self._stop_event.wait(self.poll_interval_sec)


class DualArmTeleopNode(Node):
    def __init__(self):
        super().__init__("dual_arm_servo_teleop")

        self.declare_parameter("left_port", "/dev/ttyUSB1")
        self.declare_parameter("right_port", "/dev/ttyUSB0")
        self.declare_parameter("calibration_file", os.path.expanduser("~/.openarm_exo_calibration.json"))
        self.declare_parameter("recalibrate_on_start", False)
        self.declare_parameter("calibration_only", False)
        self.declare_parameter("calibration_wait_sec", 3.0)
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("timeout", 0.01)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("serial_command_settle_sec", 0.01)
        self.declare_parameter("serial_poll_interval_sec", 0.005)
        self.declare_parameter("serial_response_retries", 3)
        self.declare_parameter("arm_smoothing_alpha", 0.2)
        self.declare_parameter("left_topic", "/left_forward_position_controller/commands")
        self.declare_parameter("right_topic", "/right_forward_position_controller/commands")
        self.declare_parameter("left_joint_bias_deg", DEFAULT_JOINT_BIAS_DEG.tolist())
        self.declare_parameter("right_joint_bias_deg", DEFAULT_JOINT_BIAS_DEG.tolist())
        self.declare_parameter("left_joint_map_matrix", DEFAULT_LEFT_JOINT_MAP_MATRIX.reshape(-1).tolist())
        self.declare_parameter("right_joint_map_matrix", DEFAULT_RIGHT_JOINT_MAP_MATRIX.reshape(-1).tolist())
        self.declare_parameter("left_servo_ids", list(range(0, 7)))
        self.declare_parameter("right_servo_ids", list(range(0, 7)))
        self.declare_parameter("left_gripper_servo_id", 7)
        self.declare_parameter("right_gripper_servo_id", 7)
        self.declare_parameter("left_gripper_input_min_deg", 0.0)
        self.declare_parameter("left_gripper_input_max_deg", 50.0)
        self.declare_parameter("right_gripper_input_min_deg", 0.0)
        self.declare_parameter("right_gripper_input_max_deg", 50.0)
        self.declare_parameter("left_gripper_sign", 1.0)
        self.declare_parameter("right_gripper_sign", -1.0)
        self.declare_parameter("gripper_delta_threshold", 0.0002)
        self.declare_parameter("max_angle_jump_deg", 60.0)

        baudrate = self.get_parameter("baudrate").value
        timeout = self.get_parameter("timeout").value
        publish_rate = self.get_parameter("publish_rate").value
        serial_command_settle_sec = float(self.get_parameter("serial_command_settle_sec").value)
        serial_poll_interval_sec = float(self.get_parameter("serial_poll_interval_sec").value)
        serial_response_retries = int(self.get_parameter("serial_response_retries").value)
        self.arm_smoothing_alpha = float(self.get_parameter("arm_smoothing_alpha").value)
        left_topic = self.get_parameter("left_topic").value
        right_topic = self.get_parameter("right_topic").value
        left_port = self.get_parameter("left_port").value
        right_port = self.get_parameter("right_port").value
        self.calibration_file = self.get_parameter("calibration_file").value
        recalibrate_on_start = bool(self.get_parameter("recalibrate_on_start").value)
        self.calibration_only = bool(self.get_parameter("calibration_only").value)
        calibration_wait_sec = float(self.get_parameter("calibration_wait_sec").value)
        max_angle_jump_deg = self.get_parameter("max_angle_jump_deg").value
        self.gripper_delta_threshold = self.get_parameter("gripper_delta_threshold").value

        self.left_joint_bias_deg = self._get_vector_parameter("left_joint_bias_deg")
        self.right_joint_bias_deg = self._get_vector_parameter("right_joint_bias_deg")
        self.left_joint_map_matrix = self._get_matrix_parameter("left_joint_map_matrix")
        self.right_joint_map_matrix = self._get_matrix_parameter("right_joint_map_matrix")
        left_servo_ids = self._get_id_parameter("left_servo_ids")
        right_servo_ids = self._get_id_parameter("right_servo_ids")
        left_gripper_servo_id = int(self.get_parameter("left_gripper_servo_id").value)
        right_gripper_servo_id = int(self.get_parameter("right_gripper_servo_id").value)
        self.left_gripper_input_min_deg = float(self.get_parameter("left_gripper_input_min_deg").value)
        self.left_gripper_input_max_deg = float(self.get_parameter("left_gripper_input_max_deg").value)
        self.right_gripper_input_min_deg = float(self.get_parameter("right_gripper_input_min_deg").value)
        self.right_gripper_input_max_deg = float(self.get_parameter("right_gripper_input_max_deg").value)
        self.left_gripper_sign = float(self.get_parameter("left_gripper_sign").value)
        self.right_gripper_sign = float(self.get_parameter("right_gripper_sign").value)

        self.left_serial_handle = serial.Serial(left_port, baudrate, timeout=timeout)
        self.right_serial_handle = serial.Serial(right_port, baudrate, timeout=timeout)

        self.left_side = ExoSideReader(
            serial_handle=self.left_serial_handle,
            servo_ids=left_servo_ids,
            gripper_servo_id=left_gripper_servo_id,
            name="left_arm",
            logger=self.get_logger(),
            max_angle_jump_deg=max_angle_jump_deg,
            command_settle_sec=serial_command_settle_sec,
            poll_interval_sec=serial_poll_interval_sec,
            response_retries=serial_response_retries,
        )
        self.right_side = ExoSideReader(
            serial_handle=self.right_serial_handle,
            servo_ids=right_servo_ids,
            gripper_servo_id=right_gripper_servo_id,
            name="right_arm",
            logger=self.get_logger(),
            max_angle_jump_deg=max_angle_jump_deg,
            command_settle_sec=serial_command_settle_sec,
            poll_interval_sec=serial_poll_interval_sec,
            response_retries=serial_response_retries,
        )

        self.left_side.initialize()
        self.right_side.initialize()
        self._setup_calibration(recalibrate_on_start, calibration_wait_sec)

        if self.calibration_only:
            self.get_logger().info("calibration_only=true，零点保存完成后不启动发布循环")
            return

        self.left_side.start()
        self.right_side.start()

        self.left_pub = self.create_publisher(Float64MultiArray, left_topic, 10)
        self.right_pub = self.create_publisher(Float64MultiArray, right_topic, 10)
        self.left_gripper_client = ActionClient(self, GripperCommand, "/left_gripper_controller/gripper_cmd")
        self.right_gripper_client = ActionClient(self, GripperCommand, "/right_gripper_controller/gripper_cmd")
        self.left_gripper_target_pub = self.create_publisher(Float64, "/left_gripper_target", 10)
        self.right_gripper_target_pub = self.create_publisher(Float64, "/right_gripper_target", 10)
        self.last_left_gripper_position = None
        self.last_right_gripper_position = None
        self.left_smoothed_cmd_rad = None
        self.right_smoothed_cmd_rad = None
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_commands)

        self.get_logger().info(
            f"Publishing left commands to {left_topic}, right commands to {right_topic}"
        )
        self.get_logger().info(
            f"Using ports left={left_port}, right={right_port}, ids left={left_servo_ids}, right={right_servo_ids}"
        )

    def _setup_calibration(self, recalibrate_on_start, calibration_wait_sec):
        if recalibrate_on_start or not os.path.exists(self.calibration_file):
            self.get_logger().info(
                f"请将外骨骼摆到初始姿态，{calibration_wait_sec:.1f} 秒后开始零点标定..."
            )
            time.sleep(calibration_wait_sec)
            self._calibrate_and_save()
            return

        try:
            self._load_calibration()
        except Exception as exc:
            self.get_logger().warn(f"加载标定文件失败，重新标定: {exc}")
            time.sleep(calibration_wait_sec)
            self._calibrate_and_save()

    def _calibrate_and_save(self):
        self.left_side.calibrate_zero()
        self.right_side.calibrate_zero()
        self._save_calibration()

    def _save_calibration(self):
        calibration = {
            "left_zero_angles_deg": self.left_side.arm_reader.zero_angles.tolist(),
            "right_zero_angles_deg": self.right_side.arm_reader.zero_angles.tolist(),
            "left_gripper_zero_deg": self.left_side.gripper_reader.zero_angle_deg,
            "right_gripper_zero_deg": self.right_side.gripper_reader.zero_angle_deg,
        }
        with open(self.calibration_file, "w", encoding="utf-8") as calibration_handle:
            json.dump(calibration, calibration_handle, indent=2)
        self.get_logger().info(f"零点标定已保存到 {self.calibration_file}")

    def _load_calibration(self):
        with open(self.calibration_file, "r", encoding="utf-8") as calibration_handle:
            calibration = json.load(calibration_handle)

        self.left_side.set_zero(
            calibration["left_zero_angles_deg"],
            calibration["left_gripper_zero_deg"],
        )
        self.right_side.set_zero(
            calibration["right_zero_angles_deg"],
            calibration["right_gripper_zero_deg"],
        )
        self.get_logger().info(f"已从 {self.calibration_file} 加载零点标定")

    def _get_vector_parameter(self, name):
        values = np.array(self.get_parameter(name).value, dtype=float)
        if values.shape[0] != ARM_JOINT_COUNT:
            raise ValueError(f"Parameter {name} must contain exactly {ARM_JOINT_COUNT} elements")
        return values

    def _get_id_parameter(self, name):
        values = [int(value) for value in self.get_parameter(name).value]
        if len(values) != ARM_JOINT_COUNT:
            raise ValueError(f"Parameter {name} must contain exactly {ARM_JOINT_COUNT} IDs")
        return values

    def _get_matrix_parameter(self, name):
        values = np.array(self.get_parameter(name).value, dtype=float)
        if values.shape[0] != ARM_JOINT_COUNT * ARM_JOINT_COUNT:
            raise ValueError(
                f"Parameter {name} must contain exactly {ARM_JOINT_COUNT * ARM_JOINT_COUNT} elements"
            )
        return values.reshape((ARM_JOINT_COUNT, ARM_JOINT_COUNT))

    def _compute_joint_command_rad(self, offset_deg, joint_map_matrix, joint_bias_deg):
        target_deg = joint_map_matrix.dot(offset_deg) + joint_bias_deg
        return np.radians(target_deg)

    def _map_gripper_position(self, offset_deg, min_deg, max_deg, sign):
        adjusted_deg = offset_deg * sign
        normalized = (adjusted_deg - min_deg) / max(1e-6, max_deg - min_deg)
        normalized = float(np.clip(normalized, 0.0, 1.0))
        return normalized * GRIPPER_OPEN_POSITION

    def _send_gripper_goal(self, action_client, position, effort=50.0):
        if not action_client.server_is_ready():
            action_client.wait_for_server(timeout_sec=0.1)
        if not action_client.server_is_ready():
            return False

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(effort)
        action_client.send_goal_async(goal)
        return True

    def _smooth_joint_command(self, previous_cmd, target_cmd):
        alpha = float(np.clip(self.arm_smoothing_alpha, 0.0, 1.0))
        if previous_cmd is None or alpha >= 1.0:
            return target_cmd.copy()
        return previous_cmd + alpha * (target_cmd - previous_cmd)

    def publish_commands(self):
        left_sample = self.left_side.get_latest()
        right_sample = self.right_side.get_latest()

        if not left_sample["has_sample"] or not right_sample["has_sample"]:
            return

        left_cmd_target_rad = self._compute_joint_command_rad(
            left_sample["arm_offset_deg"], self.left_joint_map_matrix, self.left_joint_bias_deg
        )
        right_cmd_target_rad = self._compute_joint_command_rad(
            right_sample["arm_offset_deg"], self.right_joint_map_matrix, self.right_joint_bias_deg
        )
        self.left_smoothed_cmd_rad = self._smooth_joint_command(
            self.left_smoothed_cmd_rad, left_cmd_target_rad
        )
        self.right_smoothed_cmd_rad = self._smooth_joint_command(
            self.right_smoothed_cmd_rad, right_cmd_target_rad
        )

        left_gripper_position = self._map_gripper_position(
            left_sample["gripper_offset_deg"],
            self.left_gripper_input_min_deg,
            self.left_gripper_input_max_deg,
            self.left_gripper_sign,
        )
        right_gripper_position = self._map_gripper_position(
            right_sample["gripper_offset_deg"],
            self.right_gripper_input_min_deg,
            self.right_gripper_input_max_deg,
            self.right_gripper_sign,
        )

        self.left_pub.publish(Float64MultiArray(data=self.left_smoothed_cmd_rad.tolist()))
        self.right_pub.publish(Float64MultiArray(data=self.right_smoothed_cmd_rad.tolist()))

        self.left_gripper_target_pub.publish(Float64(data=left_gripper_position))
        self.right_gripper_target_pub.publish(Float64(data=right_gripper_position))

        if self.last_left_gripper_position is None or abs(left_gripper_position - self.last_left_gripper_position) >= self.gripper_delta_threshold:
            if self._send_gripper_goal(self.left_gripper_client, left_gripper_position):
                self.last_left_gripper_position = left_gripper_position

        if self.last_right_gripper_position is None or abs(right_gripper_position - self.last_right_gripper_position) >= self.gripper_delta_threshold:
            if self._send_gripper_goal(self.right_gripper_client, right_gripper_position):
                self.last_right_gripper_position = right_gripper_position

    def destroy_node(self):
        if hasattr(self, "left_side"):
            self.left_side.stop()
        if hasattr(self, "right_side"):
            self.right_side.stop()
        if hasattr(self, "left_serial_handle") and self.left_serial_handle.is_open:
            self.left_serial_handle.close()
        if hasattr(self, "right_serial_handle") and self.right_serial_handle.is_open:
            self.right_serial_handle.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DualArmTeleopNode()
        if node.calibration_only:
            return
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()