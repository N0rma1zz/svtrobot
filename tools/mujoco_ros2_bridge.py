 #!/usr/bin/env python3
"""
MuJoCo ROS2 Bridge for VR Teleoperation Data Collection.

Bridges VR teleoperation C++ node and MuJoCo physics simulation:
  - Subscribes to joint commands from teleop_demo_openarm
  - Steps MuJoCo simulation with position servo actuators
  - Publishes joint states for IK feedback
  - Renders and publishes camera images for VR visual feedback

System architecture (3 processes):
  1. teleop_demo_openarm (C++) : VR → IK → /left|right_forward_position_controller/commands
  2. THIS NODE                 : ROS2 commands → MuJoCo sim → /joint_states + camera images
  3. VideoSenderPC (C++)       : camera image topics → H264 → TCP → VR headset

Startup order:
  1. Start this bridge first (publishes /joint_states at home pose)
  2. Start teleop_demo_openarm (receives /joint_states, initializes IK)
  3. Start VideoSenderPC --ros (subscribes to camera image topics)

Usage:
    # Set EGL for headless GPU rendering (if no display)
    export MUJOCO_GL=egl

    # Terminal 1 - MuJoCo bridge
    python3 mujoco_ros2_bridge.py [--xml /path/to/scene.xml] [--viewer]

    # Terminal 2 - VR teleop IK (do NOT use --no-hardware)
    ./teleop_demo_openarm --urdf /path/to/openarm.urdf

    # Terminal 3 - Video sender to VR headset
    ./VideoSenderPC --listen 0.0.0.0:13579 --ros \\
        --topic  /cam/head/color/image_raw \\
        --topic2 /cam/left_wrist/color/image_raw \\
        --topic3 /cam/right_wrist/color/image_raw
"""

import os
import argparse
import threading
import numpy as np
import mujoco

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState, Image


# ============================================================
# Constants
# ============================================================

# Joint names matching teleop_demo_openarm expectations
LEFT_ARM_JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)]
RIGHT_ARM_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]
LEFT_GRIPPER_JOINTS = ["openarm_left_finger_joint1", "openarm_left_finger_joint2"]
RIGHT_GRIPPER_JOINTS = ["openarm_right_finger_joint1", "openarm_right_finger_joint2"]

ALL_JOINT_NAMES = (LEFT_ARM_JOINTS + LEFT_GRIPPER_JOINTS +
                   RIGHT_ARM_JOINTS + RIGHT_GRIPPER_JOINTS)

# MuJoCo actuator names
LEFT_ARM_ACTUATORS = [f"left_joint{i}_ctrl" for i in range(1, 8)]
LEFT_FINGER_ACTUATORS = ["left_finger1_ctrl", "left_finger2_ctrl"]
RIGHT_ARM_ACTUATORS = [f"right_joint{i}_ctrl" for i in range(1, 8)]
RIGHT_FINGER_ACTUATORS = ["right_finger1_ctrl", "right_finger2_ctrl"]

# Camera name mapping: ROS topic → MuJoCo camera name
CAMERA_MAP = {
    "/cam/head/color/image_raw": "front_camera",
    "/cam/left_wrist/color/image_raw": "left_wrist_camera",
    "/cam/right_wrist/color/image_raw": "right_wrist_camera",
}

# Default home pose (training data mean)
DEFAULT_LEFT_ARM = [0.246, 0.021, -0.176, 1.561, 0.406, -0.320, 0.110]
DEFAULT_RIGHT_ARM = [-0.156, 0.014, 0.103, 1.413, -0.236, 0.285, -0.087]
DEFAULT_GRIPPER = 0.044  # Fully open


# ============================================================
# MuJoCo ROS2 Bridge Node
# ============================================================

class MujocoROS2Bridge(Node):
    def __init__(self, xml_path, ctrl_rate=50.0, cam_rate=30.0,
                 image_size=(480, 640), show_viewer=False):
        super().__init__("mujoco_ros2_bridge")

        # ---- MuJoCo setup ----
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.sim_dt = self.model.opt.timestep
        self.ctrl_dt = 1.0 / ctrl_rate
        self.n_substeps = max(1, int(self.ctrl_dt / self.sim_dt))

        self.image_height, self.image_width = image_size
        self._renderer = mujoco.Renderer(
            self.model, height=self.image_height, width=self.image_width)

        # ---- Resolve MuJoCo actuator IDs ----
        def _act_id(name):
            aid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if aid < 0:
                raise ValueError(f"Actuator '{name}' not found in model")
            return aid

        self._left_arm_act = [_act_id(n) for n in LEFT_ARM_ACTUATORS]
        self._left_finger_act = [_act_id(n) for n in LEFT_FINGER_ACTUATORS]
        self._right_arm_act = [_act_id(n) for n in RIGHT_ARM_ACTUATORS]
        self._right_finger_act = [_act_id(n) for n in RIGHT_FINGER_ACTUATORS]

        # Joint qpos addresses for state readout
        self._joint_qadr = []
        for name in ALL_JOINT_NAMES:
            jid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"Joint '{name}' not found in model")
            self._joint_qadr.append(self.model.jnt_qposadr[jid])

        # Camera IDs
        self._cam_ids = {}
        for topic, cam_name in CAMERA_MAP.items():
            cid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
            if cid < 0:
                self.get_logger().warn(f"Camera '{cam_name}' not found, "
                                       f"topic {topic} disabled")
            else:
                self._cam_ids[topic] = cid

        # ---- Command buffer (protected by lock) ----
        self._lock = threading.Lock()
        self._left_arm_cmd = np.array(DEFAULT_LEFT_ARM)
        self._right_arm_cmd = np.array(DEFAULT_RIGHT_ARM)
        self._left_gripper_cmd = DEFAULT_GRIPPER
        self._right_gripper_cmd = DEFAULT_GRIPPER

        # ---- Initialize sim to home pose ----
        self._reset_to_home()

        # ---- ROS2 Subscribers ----
        # Arm joint commands from teleop_demo_openarm
        self.create_subscription(
            Float64MultiArray,
            "/left_forward_position_controller/commands",
            self._left_arm_cb, 10)
        self.create_subscription(
            Float64MultiArray,
            "/right_forward_position_controller/commands",
            self._right_arm_cb, 10)
        # Gripper targets (Float64 published alongside GripperCommand action)
        self.create_subscription(
            Float64, "/left_gripper_target",
            self._left_gripper_cb, 10)
        self.create_subscription(
            Float64, "/right_gripper_target",
            self._right_gripper_cb, 10)

        # ---- ROS2 Publishers ----
        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._cam_pubs = {
            topic: self.create_publisher(Image, topic, 10)
            for topic in self._cam_ids
        }

        # ---- Timer loops ----
        self._ctrl_timer = self.create_timer(self.ctrl_dt, self._ctrl_step)
        self._cam_timer = self.create_timer(1.0 / cam_rate, self._cam_step)

        # ---- Optional MuJoCo viewer (requires display) ----
        self._viewer = None
        if show_viewer:
            from mujoco.viewer import launch_passive
            self._viewer = launch_passive(self.model, self.data)

        self.get_logger().info(
            f"Bridge ready: ctrl={ctrl_rate}Hz cam={cam_rate}Hz "
            f"substeps={self.n_substeps} "
            f"img={self.image_width}x{self.image_height} "
            f"cameras={list(self._cam_ids.keys())}")

    # ================================================================
    # Home pose initialization
    # ================================================================
    def _reset_to_home(self):
        mujoco.mj_resetData(self.model, self.data)

        # Set joint positions
        for i, name in enumerate(LEFT_ARM_JOINTS):
            jid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.data.qpos[self.model.jnt_qposadr[jid]] = DEFAULT_LEFT_ARM[i]
        for i, name in enumerate(RIGHT_ARM_JOINTS):
            jid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.data.qpos[self.model.jnt_qposadr[jid]] = DEFAULT_RIGHT_ARM[i]
        for name in LEFT_GRIPPER_JOINTS + RIGHT_GRIPPER_JOINTS:
            jid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.data.qpos[self.model.jnt_qposadr[jid]] = DEFAULT_GRIPPER

        # Set initial actuator targets to match
        for i, aid in enumerate(self._left_arm_act):
            self.data.ctrl[aid] = DEFAULT_LEFT_ARM[i]
        for i, aid in enumerate(self._right_arm_act):
            self.data.ctrl[aid] = DEFAULT_RIGHT_ARM[i]
        for aid in self._left_finger_act + self._right_finger_act:
            self.data.ctrl[aid] = DEFAULT_GRIPPER

        mujoco.mj_forward(self.model, self.data)

    # ================================================================
    # Subscriber callbacks
    # ================================================================
    def _left_arm_cb(self, msg):
        if len(msg.data) >= 7:
            with self._lock:
                self._left_arm_cmd = np.array(msg.data[:7])

    def _right_arm_cb(self, msg):
        if len(msg.data) >= 7:
            with self._lock:
                self._right_arm_cmd = np.array(msg.data[:7])

    def _left_gripper_cb(self, msg):
        with self._lock:
            self._left_gripper_cmd = float(np.clip(msg.data, 0.0, 0.044))

    def _right_gripper_cb(self, msg):
        with self._lock:
            self._right_gripper_cmd = float(np.clip(msg.data, 0.0, 0.044))

    # ================================================================
    # Control step (timer callback @ ctrl_rate Hz)
    # ================================================================
    def _ctrl_step(self):
        # Snapshot commands
        with self._lock:
            la = self._left_arm_cmd.copy()
            ra = self._right_arm_cmd.copy()
            lg = self._left_gripper_cmd
            rg = self._right_gripper_cmd

        # Set actuator targets
        for i, aid in enumerate(self._left_arm_act):
            self.data.ctrl[aid] = la[i]
        for i, aid in enumerate(self._right_arm_act):
            self.data.ctrl[aid] = ra[i]
        for aid in self._left_finger_act:
            self.data.ctrl[aid] = lg
        for aid in self._right_finger_act:
            self.data.ctrl[aid] = rg

        # Step physics
        for _ in range(self.n_substeps):
            mujoco.mj_step(self.model, self.data)

        # Sync passive viewer
        if self._viewer is not None:
            self._viewer.sync()

        # Publish joint state
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(ALL_JOINT_NAMES)
        js.position = [float(self.data.qpos[a]) for a in self._joint_qadr]
        self._js_pub.publish(js)

    # ================================================================
    # Camera step (timer callback @ cam_rate Hz)
    # ================================================================
    def _cam_step(self):
        stamp = self.get_clock().now().to_msg()
        for topic, cam_id in self._cam_ids.items():
            self._renderer.update_scene(self.data, camera=cam_id)
            rgb = self._renderer.render()

            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = CAMERA_MAP[topic]
            msg.height = self.image_height
            msg.width = self.image_width
            msg.encoding = "rgb8"
            msg.is_bigendian = False
            msg.step = self.image_width * 3
            msg.data = rgb.tobytes()

            self._cam_pubs[topic].publish(msg)

    # ================================================================
    # Cleanup
    # ================================================================
    def destroy_node(self):
        if self._viewer is not None:
            self._viewer.close()
        self._renderer.close()
        super().destroy_node()


# ============================================================
# Entry point
# ============================================================
def main():
    parser = argparse.ArgumentParser(
        description="MuJoCo ROS2 Bridge for VR Teleoperation")
    parser.add_argument("--xml", type=str, default=None,
                        help="Path to MuJoCo scene XML")
    parser.add_argument("--ctrl-rate", type=float, default=50.0,
                        help="Control loop rate in Hz (default: 50)")
    parser.add_argument("--cam-rate", type=float, default=30.0,
                        help="Camera publish rate in Hz (default: 30)")
    parser.add_argument("--image-width", type=int, default=640)
    parser.add_argument("--image-height", type=int, default=480)
    parser.add_argument("--viewer", action="store_true",
                        help="Show MuJoCo viewer window (requires display)")
    args = parser.parse_args()

    xml_path = args.xml
    if xml_path is None:
        # Now lives in ros2_ws/tools/, scene in ros2_ws/src/openarm_mujoco/v1/
        base = os.path.dirname(os.path.abspath(__file__))
        for rel in ("../src", ".", "../.."):
            cand = os.path.join(
                base, rel, "openarm_mujoco", "v1", "openarm_scene_groot.xml")
            if os.path.isfile(cand):
                xml_path = cand
                break
        if xml_path is None:
            raise FileNotFoundError(
                "Cannot find openarm_scene_groot.xml. Use --xml to specify.")

    rclpy.init()
    node = MujocoROS2Bridge(
        xml_path=xml_path,
        ctrl_rate=args.ctrl_rate,
        cam_rate=args.cam_rate,
        image_size=(args.image_height, args.image_width),
        show_viewer=args.viewer,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
