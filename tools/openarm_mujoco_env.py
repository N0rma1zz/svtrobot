"""
OpenArm MuJoCo Gymnasium environment for GR00T policy evaluation.

Wraps the OpenArm bimanual MuJoCo model into a Gymnasium-compatible env
that outputs observations matching the GR00T training data format:
  - 3 camera images (cam_head, cam_left_wrist, cam_right_wrist)
  - State: left_arm(7D), left_gripper(1D), right_arm(7D), right_gripper(1D)
  - Action: left_arm(7D absolute), left_gripper(1D abs), right_arm(7D absolute), right_gripper(1D abs)

GR00T's unapply_action converts relative deltas back to absolute positions,
so the policy output is already absolute joint targets.

Arm joints use MuJoCo position servos (kp/kv in XML, implicitfast integrator).
"""

import os
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import mujoco


# Joint names
LEFT_ARM_JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)]
LEFT_GRIPPER_JOINTS = ["openarm_left_finger_joint1"]
RIGHT_ARM_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]
RIGHT_GRIPPER_JOINTS = ["openarm_right_finger_joint1"]

# Default home pose – mean initial state from training data
DEFAULT_LEFT_ARM = [0.246, 0.021, -0.176, 1.561, 0.406, -0.320, 0.110]
DEFAULT_RIGHT_ARM = [-0.156, 0.014, 0.103, 1.413, -0.236, 0.285, -0.087]


class OpenArmMujocoEnv(gym.Env):
    """OpenArm bimanual MuJoCo environment for GR00T inference."""

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(
        self,
        xml_path: str | None = None,
        image_size: tuple[int, int] = (480, 640),
        max_episode_steps: int = 400,
        ctrl_dt: float = 1.0 / 15.0,  # Match training data FPS (15 Hz)
        task_description: str = "pick up the object",
    ):
        super().__init__()

        if xml_path is None:
            # Now lives in ros2_ws/tools/, scene in ros2_ws/src/openarm_mujoco/v1/
            base_dir = os.path.dirname(os.path.abspath(__file__))
            for depth in ("../src", ".", "../.."):
                candidate = os.path.join(base_dir, depth, "openarm_mujoco", "v1", "openarm_scene_groot.xml")
                if os.path.isfile(candidate):
                    xml_path = candidate
                    break
            if xml_path is None:
                raise FileNotFoundError("Cannot find openarm_scene_groot.xml")

        self.image_height, self.image_width = image_size
        self.max_episode_steps = max_episode_steps
        self.task_description = task_description

        # Load model
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.sim_dt = self.model.opt.timestep
        self.ctrl_dt = ctrl_dt
        self.n_substeps = max(1, int(ctrl_dt / self.sim_dt))

        # --- Joint index resolution ---
        self._left_arm_jnt_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEFT_ARM_JOINTS]
        self._left_grip_jnt_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEFT_GRIPPER_JOINTS]
        self._right_arm_jnt_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in RIGHT_ARM_JOINTS]
        self._right_grip_jnt_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in RIGHT_GRIPPER_JOINTS]

        # qpos addresses
        self._left_arm_qadr = [self.model.jnt_qposadr[i] for i in self._left_arm_jnt_ids]
        self._left_grip_qadr = [self.model.jnt_qposadr[i] for i in self._left_grip_jnt_ids]
        self._right_arm_qadr = [self.model.jnt_qposadr[i] for i in self._right_arm_jnt_ids]
        self._right_grip_qadr = [self.model.jnt_qposadr[i] for i in self._right_grip_jnt_ids]

        # --- Actuator index resolution ---
        self._left_arm_act_ids = []
        self._left_grip_act_ids = []
        self._right_arm_act_ids = []
        self._right_grip_act_ids = []
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name.startswith("left_joint") and name.endswith("_ctrl"):
                self._left_arm_act_ids.append(i)
            elif name.startswith("left_finger") and name.endswith("_ctrl"):
                self._left_grip_act_ids.append(i)
            elif name.startswith("right_joint") and name.endswith("_ctrl"):
                self._right_arm_act_ids.append(i)
            elif name.startswith("right_finger") and name.endswith("_ctrl"):
                self._right_grip_act_ids.append(i)

        # --- Camera setup ---
        # All cameras are now defined in the MJCF XML
        self._head_cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "front_camera")
        self._left_wrist_cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "left_wrist_camera")
        self._right_wrist_cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "right_wrist_camera")

        # Renderer
        self._renderer = mujoco.Renderer(self.model, height=self.image_height, width=self.image_width)

        # Position actuator targets (set during step)
        self._left_arm_target = np.array(DEFAULT_LEFT_ARM)
        self._right_arm_target = np.array(DEFAULT_RIGHT_ARM)

        # Gym spaces
        img_space = spaces.Box(0, 255, shape=(self.image_height, self.image_width, 3), dtype=np.uint8)
        self.observation_space = spaces.Dict({
            "cam_head": img_space,
            "cam_left_wrist": img_space,
            "cam_right_wrist": img_space,
            "left_arm": spaces.Box(-np.pi, np.pi, shape=(7,), dtype=np.float64),
            "left_gripper": spaces.Box(0.0, 0.044, shape=(1,), dtype=np.float64),
            "right_arm": spaces.Box(-np.pi, np.pi, shape=(7,), dtype=np.float64),
            "right_gripper": spaces.Box(0.0, 0.044, shape=(1,), dtype=np.float64),
            "annotation.human.task_description": spaces.Text(max_length=512),
        })
        self.action_space = spaces.Box(-1.0, 1.0, shape=(16,), dtype=np.float64)

        self._step_count = 0

    # --- Joint helpers ---

    def _get_joint_positions(self):
        left_arm = np.array([self.data.qpos[a] for a in self._left_arm_qadr])
        left_grip = np.array([self.data.qpos[a] for a in self._left_grip_qadr])
        right_arm = np.array([self.data.qpos[a] for a in self._right_arm_qadr])
        right_grip = np.array([self.data.qpos[a] for a in self._right_grip_qadr])
        return left_arm, left_grip, right_arm, right_grip

    # --- Position servo control ---

    def _apply_position_ctrl(self):
        """Set position targets for arm actuators (MuJoCo position servos)."""
        for i, act_id in enumerate(self._left_arm_act_ids):
            self.data.ctrl[act_id] = self._left_arm_target[i]
        for i, act_id in enumerate(self._right_arm_act_ids):
            self.data.ctrl[act_id] = self._right_arm_target[i]

    # --- Camera rendering ---

    def _render_camera_by_id(self, cam_id: int) -> np.ndarray:
        self._renderer.update_scene(self.data, camera=cam_id)
        return self._renderer.render().copy()

    # --- Observation ---

    def _get_obs(self) -> dict:
        left_arm, left_grip, right_arm, right_grip = self._get_joint_positions()

        obs = {
            "left_arm": left_arm,
            "left_gripper": left_grip,
            "right_arm": right_arm,
            "right_gripper": right_grip,
            "annotation.human.task_description": self.task_description,
        }

        # All cameras rendered from MJCF-defined cameras
        obs["cam_head"] = self._render_camera_by_id(self._head_cam_id)
        obs["cam_left_wrist"] = self._render_camera_by_id(self._left_wrist_cam_id)
        obs["cam_right_wrist"] = self._render_camera_by_id(self._right_wrist_cam_id)

        return obs

    # --- Gym interface ---

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)

        for i, addr in enumerate(self._left_arm_qadr):
            self.data.qpos[addr] = DEFAULT_LEFT_ARM[i]
        for i, addr in enumerate(self._right_arm_qadr):
            self.data.qpos[addr] = DEFAULT_RIGHT_ARM[i]

        self._left_arm_target = np.array(DEFAULT_LEFT_ARM)
        self._right_arm_target = np.array(DEFAULT_RIGHT_ARM)

        mujoco.mj_forward(self.model, self.data)
        self._step_count = 0
        return self._get_obs(), {}

    def step(self, action: np.ndarray):
        """Apply action and step simulation.

        Action layout (all ABSOLUTE joint targets from GR00T unapply_action):
          [0:7]  left_arm target  (ABSOLUTE joint positions)
          [7]    left_gripper     (ABSOLUTE, range [0, 0.044])
          [8:15] right_arm target (ABSOLUTE joint positions)
          [15]   right_gripper    (ABSOLUTE, range [0, 0.044])
        """
        left_arm_target = action[0:7]
        left_grip_abs = action[7]
        right_arm_target = action[8:15]
        right_grip_abs = action[15]

        # Set PD targets directly (policy output is already absolute)
        self._left_arm_target = np.copy(left_arm_target)
        self._right_arm_target = np.copy(right_arm_target)

        # Clip targets to joint limits
        for i, jid in enumerate(self._left_arm_jnt_ids):
            lo, hi = self.model.jnt_range[jid]
            self._left_arm_target[i] = np.clip(self._left_arm_target[i], lo, hi)
        for i, jid in enumerate(self._right_arm_jnt_ids):
            lo, hi = self.model.jnt_range[jid]
            self._right_arm_target[i] = np.clip(self._right_arm_target[i], lo, hi)

        # Gripper: position actuators, set ctrl directly
        left_grip_target = np.clip(left_grip_abs, 0.0, 0.044)
        right_grip_target = np.clip(right_grip_abs, 0.0, 0.044)
        for act_id in self._left_grip_act_ids:
            self.data.ctrl[act_id] = left_grip_target
        for act_id in self._right_grip_act_ids:
            self.data.ctrl[act_id] = right_grip_target

        # Step simulation with position servo control
        for _ in range(self.n_substeps):
            self._apply_position_ctrl()
            mujoco.mj_step(self.model, self.data)

        self._step_count += 1
        truncated = self._step_count >= self.max_episode_steps
        return self._get_obs(), 0.0, False, truncated, {}

    def close(self):
        if hasattr(self, '_renderer'):
            self._renderer.close()
