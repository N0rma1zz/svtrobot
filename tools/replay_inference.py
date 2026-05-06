#!/usr/bin/env python3
"""
Replay dataset episodes through the GR00T model and compare predictions vs ground truth.

Runs on local machine with GPU. Saves trajectories as npz for RViz visualization,
and generates matplotlib comparison plots.

Usage:
  cd Isaac-GR00T
  source .venv/bin/activate
  python examples/OpenArm/replay_inference.py \
      --model_path ./openarm_checkpoints \
      --episode_idx 0 \
      --output_dir ./replay_results

  # Process every 3rd frame for faster run:
  python examples/OpenArm/replay_inference.py --step_interval 3

  # Use specific video backend:
  python examples/OpenArm/replay_inference.py --video_backend opencv
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", ".."))

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import pyarrow.parquet as pq

# Register OpenArm modality config (must import before creating policy)
import examples.OpenArm.config.openarm_config  # noqa: F401
from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.policy.gr00t_policy import Gr00tPolicy


def load_video_frames_sequential(video_path: str, num_frames: int) -> np.ndarray:
    """Load all frames from a video file sequentially (much faster than seeking).

    Args:
        video_path: Path to the MP4 video file.
        num_frames: Expected number of frames.

    Returns:
        np.ndarray of shape (N, H, W, 3), dtype uint8, RGB format.
    """
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise IOError(f"Cannot open video: {video_path}")

    frames = []
    for _ in range(num_frames):
        ret, frame = cap.read()
        if not ret:
            break
        # OpenCV reads BGR, convert to RGB for model consistency
        frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    cap.release()

    if len(frames) < num_frames:
        print(f"  Warning: video {video_path} has {len(frames)} frames, expected {num_frames}")

    return np.array(frames, dtype=np.uint8)


def load_episode_data(dataset_path: str, episode_idx: int) -> dict:
    """Load one episode's parquet data, metadata, and pre-load all video frames."""
    dataset_path = Path(dataset_path)
    meta_dir = dataset_path / "meta"

    # Load dataset info
    with open(meta_dir / "info.json") as f:
        info = json.load(f)

    # Load task descriptions
    with open(meta_dir / "tasks.jsonl") as f:
        tasks = {t["task_index"]: t["task"] for t in (json.loads(line) for line in f)}

    # Read parquet file for this episode
    chunk_idx = episode_idx // info["chunks_size"]
    parquet_path = dataset_path / f"data/chunk-{chunk_idx:03d}/episode_{episode_idx:06d}.parquet"
    table = pq.read_table(str(parquet_path))

    states = np.array([row.as_py() for row in table["observation.state"]], dtype=np.float32)
    actions = np.array([row.as_py() for row in table["action"]], dtype=np.float32)
    task_idx = table["task_index"][0].as_py()
    task_desc = tasks.get(task_idx, "unknown task")
    num_frames = len(states)

    # Pre-load all video frames for efficiency
    cam_names = {
        "cam_head": "observation.images.cam_head",
        "cam_left_wrist": "observation.images.cam_left_wrist",
        "cam_right_wrist": "observation.images.cam_right_wrist",
    }

    print(f"Pre-loading video frames for episode {episode_idx} ({num_frames} frames)...")
    video_frames = {}
    for short_name, full_name in cam_names.items():
        video_path = dataset_path / f"videos/chunk-{chunk_idx:03d}/{full_name}/episode_{episode_idx:06d}.mp4"
        video_frames[short_name] = load_video_frames_sequential(str(video_path), num_frames)
        print(f"  {short_name}: {video_frames[short_name].shape}")

    return {
        "states": states,
        "actions": actions,
        "video_frames": video_frames,
        "task_desc": task_desc,
        "fps": info["fps"],
        "num_frames": num_frames,
    }


def run_replay_inference(
    model_path: str,
    dataset_path: str,
    episode_idx: int,
    device: str,
    output_dir: str,
    step_interval: int = 1,
) -> dict:
    """Run open-loop replay inference: feed recorded observations, compare model predictions."""

    ep_data = load_episode_data(dataset_path, episode_idx)
    num_frames = ep_data["num_frames"]
    states = ep_data["states"]
    actions = ep_data["actions"]
    video_frames = ep_data["video_frames"]

    print(f"\nEpisode {episode_idx}: {num_frames} frames @ {ep_data['fps']} FPS")
    print(f"Task: {ep_data['task_desc']}")

    print(f"\nLoading model from {model_path}...")
    policy = Gr00tPolicy(
        embodiment_tag=EmbodimentTag.NEW_EMBODIMENT,
        model_path=model_path,
        device=device,
    )
    print("Model loaded.\n")

    pred_actions = []
    inference_times = []
    processed_indices = list(range(0, num_frames, step_interval))

    print(f"Running inference on {len(processed_indices)} frames (step_interval={step_interval})...")
    for count, i in enumerate(processed_indices):
        # Build observation dict matching Gr00tPolicy.check_observation expectations
        observation = {
            "video": {
                "cam_head": video_frames["cam_head"][i][np.newaxis, np.newaxis, ...],
                "cam_left_wrist": video_frames["cam_left_wrist"][i][np.newaxis, np.newaxis, ...],
                "cam_right_wrist": video_frames["cam_right_wrist"][i][np.newaxis, np.newaxis, ...],
            },
            "state": {
                "left_arm": states[i, 0:7][np.newaxis, np.newaxis, :].astype(np.float32),
                "left_gripper": states[i, 7:8][np.newaxis, np.newaxis, :].astype(np.float32),
                "right_arm": states[i, 8:15][np.newaxis, np.newaxis, :].astype(np.float32),
                "right_gripper": states[i, 15:16][np.newaxis, np.newaxis, :].astype(np.float32),
            },
            "language": {
                "annotation.human.task_description": [[ep_data["task_desc"]]],
            },
        }

        t0 = time.time()
        action_pred, _ = policy.get_action(observation)
        dt = time.time() - t0
        inference_times.append(dt)

        # action_pred: dict of (B=1, T=16, D) arrays — already absolute joint positions
        # Extract first step (t+1 prediction)
        first_step = np.concatenate([
            action_pred["left_arm"][0, 0, :],
            action_pred["left_gripper"][0, 0, :],
            action_pred["right_arm"][0, 0, :],
            action_pred["right_gripper"][0, 0, :],
        ])  # (16,)

        pred_actions.append(first_step)

        if (count + 1) % 50 == 0 or count == 0:
            print(f"  [{count+1}/{len(processed_indices)}] frame={i}, "
                  f"infer={dt:.3f}s, avg={np.mean(inference_times):.3f}s")

    pred_actions = np.array(pred_actions, dtype=np.float32)

    # Save results
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    joint_names = [
        "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
        "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
        "openarm_left_joint7", "openarm_left_finger_joint1",
        "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
        "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
        "openarm_right_joint7", "openarm_right_finger_joint1",
    ]

    out_path = output_dir / f"episode_{episode_idx:03d}_replay.npz"
    np.savez(
        out_path,
        gt_states=states,
        gt_actions=actions,
        pred_actions=pred_actions,
        processed_indices=np.array(processed_indices),
        fps=np.float32(ep_data["fps"]),
        task_desc=ep_data["task_desc"],
        joint_names=json.dumps(joint_names),
    )

    avg_time = np.mean(inference_times)
    print(f"\nResults saved to {out_path}")
    print(f"Avg inference: {avg_time:.3f}s ({1/avg_time:.1f} Hz)")

    return {
        "gt_states": states,
        "gt_actions": actions,
        "pred_actions": pred_actions,
        "processed_indices": processed_indices,
        "fps": ep_data["fps"],
        "inference_times": inference_times,
    }


def plot_trajectory_comparison(results: dict, episode_idx: int, output_dir: str):
    """Generate a 4x4 grid comparing ground truth vs predicted for each joint."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    gt_actions = results["gt_actions"]
    pred_actions = results["pred_actions"]
    processed_indices = results["processed_indices"]
    fps = results["fps"]

    time_gt = np.arange(len(gt_actions)) / fps
    time_pred = np.array(processed_indices) / fps

    joint_labels = [
        "L_J1", "L_J2", "L_J3", "L_J4", "L_J5", "L_J6", "L_J7", "L_Grip",
        "R_J1", "R_J2", "R_J3", "R_J4", "R_J5", "R_J6", "R_J7", "R_Grip",
    ]

    fig, axes = plt.subplots(4, 4, figsize=(20, 16))
    fig.suptitle(f"Episode {episode_idx}: Ground Truth (blue) vs Predicted (red)", fontsize=14)

    for j in range(16):
        ax = axes[j // 4, j % 4]
        ax.plot(time_gt, gt_actions[:, j], "b-", alpha=0.7, label="GT", linewidth=1)
        ax.plot(time_pred, pred_actions[:, j], "r--", alpha=0.7, label="Pred", linewidth=1)
        ax.set_title(joint_labels[j], fontsize=10)
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.set_ylabel("Rad", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.3)
        if j == 0:
            ax.legend(fontsize=8)

    plt.tight_layout()
    out_path = Path(output_dir) / f"episode_{episode_idx:03d}_comparison.png"
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"Plot saved to {out_path}")


def plot_error_analysis(results: dict, episode_idx: int, output_dir: str):
    """Plot per-joint prediction error over time."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    gt_actions = results["gt_actions"]
    pred_actions = results["pred_actions"]
    processed_indices = results["processed_indices"]
    fps = results["fps"]

    # Compute error at processed frames
    gt_at_pred = gt_actions[processed_indices]
    errors = np.abs(pred_actions - gt_at_pred)
    time_pred = np.array(processed_indices) / fps

    joint_labels = [
        "L_J1", "L_J2", "L_J3", "L_J4", "L_J5", "L_J6", "L_J7", "L_Grip",
        "R_J1", "R_J2", "R_J3", "R_J4", "R_J5", "R_J6", "R_J7", "R_Grip",
    ]

    fig, axes = plt.subplots(2, 1, figsize=(16, 10))

    # Left arm errors
    ax = axes[0]
    for j in range(8):
        ax.plot(time_pred, errors[:, j], label=joint_labels[j], linewidth=1)
    ax.set_title(f"Episode {episode_idx}: Left Arm Absolute Prediction Error", fontsize=12)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (rad)")
    ax.legend(ncol=4, fontsize=8)
    ax.grid(True, alpha=0.3)

    # Right arm errors
    ax = axes[1]
    for j in range(8, 16):
        ax.plot(time_pred, errors[:, j], label=joint_labels[j], linewidth=1)
    ax.set_title(f"Episode {episode_idx}: Right Arm Absolute Prediction Error", fontsize=12)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (rad)")
    ax.legend(ncol=4, fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = Path(output_dir) / f"episode_{episode_idx:03d}_errors.png"
    plt.savefig(out_path, dpi=150)
    plt.close()

    # Print summary statistics
    mean_errors = errors.mean(axis=0)
    max_errors = errors.max(axis=0)
    print(f"\nPer-joint mean absolute error (rad):")
    for j in range(16):
        print(f"  {joint_labels[j]:8s}: mean={mean_errors[j]:.4f}  max={max_errors[j]:.4f}")
    print(f"\nOverall mean error: {errors.mean():.4f} rad ({np.degrees(errors.mean()):.2f} deg)")
    print(f"Error plot saved to {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Replay dataset through GR00T model")
    parser.add_argument("--model_path", type=str, default="./openarm_checkpoints")
    parser.add_argument("--dataset_path", type=str,
                        default=os.path.expanduser(
                            "~/.cache/huggingface/lerobot/local/openarmx_candy_merged"))
    parser.add_argument("--episode_idx", type=int, default=0)
    parser.add_argument("--device", type=str, default="cuda:0")
    parser.add_argument("--output_dir", type=str, default="./replay_results")
    parser.add_argument("--step_interval", type=int, default=1,
                        help="Process every N-th frame (1=all frames)")
    parser.add_argument("--no_plot", action="store_true",
                        help="Skip matplotlib plot generation")
    args = parser.parse_args()

    results = run_replay_inference(
        model_path=args.model_path,
        dataset_path=args.dataset_path,
        episode_idx=args.episode_idx,
        device=args.device,
        output_dir=args.output_dir,
        step_interval=args.step_interval,
    )

    if not args.no_plot:
        plot_trajectory_comparison(results, args.episode_idx, args.output_dir)
        plot_error_analysis(results, args.episode_idx, args.output_dir)
