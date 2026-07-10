import os
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from vr_udp_bridge.lerobot_piper_recorder import (
    CAMERA_NAMES,
    JOINT_NAMES,
    UdpImageReceiver,
    joint_state_to_array,
    parse_bool,
    pose_to_array,
)


def import_smolvla_policy():
    try:
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
        from lerobot.processor.pipeline import DataProcessorPipeline

        return SmolVLAPolicy, DataProcessorPipeline
    except ImportError as exc:
        raise RuntimeError(
            "SmolVLA inference dependencies are missing. Install with: "
            "python3 -m pip install 'lerobot[smolvla]==0.4.4' Pillow"
        ) from exc


class SmolVLAPiperInference(Node):
    def __init__(self) -> None:
        super().__init__("lerobot_piper_smolvla_infer")
        self._declare_parameters()
        self._load_parameters()

        self.joint_state = {"left": None, "right": None}
        self.ee_pose = {"left": None, "right": None}
        self.last_action: Optional[np.ndarray] = None
        self.smoothed_policy_action: Optional[np.ndarray] = None
        self.command_segment_start: Optional[np.ndarray] = None
        self.command_segment_target: Optional[np.ndarray] = None
        self.command_segment_started_at = 0.0
        self.command_segment_duration = 1.0
        self.last_command: Optional[np.ndarray] = None
        self.action_queue: list[np.ndarray] = []
        self.last_action_log_time = 0.0
        self.home_started_at: Optional[float] = None
        self.home_reached_at: Optional[float] = None
        self.home_complete = not (
            self.publish_commands and self.move_to_home_on_start
        )

        self.image_receivers = {}
        for camera_name in CAMERA_NAMES:
            receiver = UdpImageReceiver(
                camera_name,
                self.image_bind,
                self.camera_ports[camera_name],
                self.recv_buffer,
                self.max_datagram_size,
                self.frame_timeout,
                self.get_logger(),
            )
            self.image_receivers[camera_name] = receiver
            receiver.start()

        self._subscribe_arm("left")
        self._subscribe_arm("right")
        self.left_pub = self.create_publisher(JointState, "/left/joint_ctrl_single", 10)
        self.right_pub = self.create_publisher(JointState, "/right/joint_ctrl_single", 10)

        self._load_policy()
        self.timer = self.create_timer(1.0 / self.inference_rate_hz, self._on_timer)
        self.command_timer = None
        if self.interpolate_commands or self.action_chunk_steps > 1:
            self.command_timer = self.create_timer(
                1.0 / self.command_rate_hz,
                self._on_command_timer,
            )

        mode = "PUBLISHING" if self.publish_commands else "DRY RUN"
        self.get_logger().warn(
            "SmolVLA Piper inference started in %s mode. policy_path=%s task=%r"
            % (mode, self.policy_path, self.task)
        )
        self.get_logger().warn(
            "Command safety: action_mode=%s, max_joint_step_rad=%.3f, max_gripper_step_m=%.3f, action_smoothing_alpha=%.3f, gripper_threshold_m=%.3f, action_chunk_steps=%d, interpolate_commands=%s, command_rate_hz=%.1f"
            % (
                self.action_mode,
                self.max_joint_step_rad,
                self.max_gripper_step_m,
                self.action_smoothing_alpha,
                self.gripper_threshold_m,
                self.action_chunk_steps,
                self.interpolate_commands,
                self.command_rate_hz,
            )
        )
        if not self.home_complete:
            self.get_logger().warn(
                "Startup home move enabled: left=%s right=%s gripper=%.3f"
                % (
                    np.array2string(
                        self.left_home_joints, precision=3, suppress_small=True
                    ),
                    np.array2string(
                        self.right_home_joints, precision=3, suppress_small=True
                    ),
                    self.home_gripper_position,
                )
            )

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "policy_path",
            str(
                Path.home()
                / "piper_tele_ws/checkpoints/smolvla_cube_task_010000"
            ),
        )
        self.declare_parameter("task", "Put both green cubes into the box")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("publish_commands", False)
        self.declare_parameter("inference_rate_hz", 5.0)
        self.declare_parameter("image_bind", "0.0.0.0")
        self.declare_parameter("main_image_port", 5020)
        self.declare_parameter("left_wrist_image_port", 5021)
        self.declare_parameter("right_wrist_image_port", 5022)
        self.declare_parameter("main_image_width", 1280)
        self.declare_parameter("main_image_height", 720)
        self.declare_parameter("left_wrist_image_width", 640)
        self.declare_parameter("left_wrist_image_height", 480)
        self.declare_parameter("right_wrist_image_width", 640)
        self.declare_parameter("right_wrist_image_height", 480)
        self.declare_parameter("recv_buffer", 4 * 1024 * 1024)
        self.declare_parameter("max_datagram_size", 65535)
        self.declare_parameter("frame_timeout", 1.0)
        self.declare_parameter("image_timeout", 1.0)
        self.declare_parameter("max_joint_step_rad", 0.04)
        self.declare_parameter("max_gripper_step_m", 0.008)
        self.declare_parameter("action_mode", "absolute")
        self.declare_parameter("action_smoothing_alpha", 1.0)
        self.declare_parameter("gripper_threshold_m", -1.0)
        self.declare_parameter("action_chunk_steps", 1)
        self.declare_parameter("interpolate_commands", False)
        self.declare_parameter("command_rate_hz", 50.0)
        self.declare_parameter("gripper_min_m", 0.0)
        self.declare_parameter("gripper_max_m", 0.10)
        self.declare_parameter("move_to_home_on_start", True)
        self.declare_parameter(
            "left_home_joint_positions",
            "-0.25,0.877031988,-1.130597972,0,1.220678788,0",
        )
        self.declare_parameter(
            "right_home_joint_positions",
            "0.25,0.877031988,-1.130597972,0,1.220678788,0",
        )
        self.declare_parameter("home_gripper_position", 0.10)
        self.declare_parameter("home_tolerance_rad", 0.03)
        self.declare_parameter("home_timeout_s", 20.0)
        self.declare_parameter("home_settle_s", 0.7)
        self.declare_parameter("home_max_joint_step_rad", 0.04)
        self.declare_parameter("home_max_gripper_step_m", 0.010)
        self.declare_parameter("log_actions", True)

    def _load_parameters(self) -> None:
        self.policy_path = Path(
            os.path.expanduser(str(self.get_parameter("policy_path").value))
        )
        self.task = str(self.get_parameter("task").value)
        self.device = str(self.get_parameter("device").value)
        self.publish_commands = parse_bool(self.get_parameter("publish_commands").value)
        self.inference_rate_hz = float(self.get_parameter("inference_rate_hz").value)
        self.image_bind = str(self.get_parameter("image_bind").value)
        self.camera_ports = {
            "main": int(self.get_parameter("main_image_port").value),
            "left_wrist": int(self.get_parameter("left_wrist_image_port").value),
            "right_wrist": int(self.get_parameter("right_wrist_image_port").value),
        }
        self.camera_shapes = {
            "main": (
                int(self.get_parameter("main_image_height").value),
                int(self.get_parameter("main_image_width").value),
            ),
            "left_wrist": (
                int(self.get_parameter("left_wrist_image_height").value),
                int(self.get_parameter("left_wrist_image_width").value),
            ),
            "right_wrist": (
                int(self.get_parameter("right_wrist_image_height").value),
                int(self.get_parameter("right_wrist_image_width").value),
            ),
        }
        self.recv_buffer = int(self.get_parameter("recv_buffer").value)
        self.max_datagram_size = int(self.get_parameter("max_datagram_size").value)
        self.frame_timeout = float(self.get_parameter("frame_timeout").value)
        self.image_timeout = float(self.get_parameter("image_timeout").value)
        self.max_joint_step_rad = float(self.get_parameter("max_joint_step_rad").value)
        self.max_gripper_step_m = float(self.get_parameter("max_gripper_step_m").value)
        self.action_mode = str(self.get_parameter("action_mode").value)
        self.action_smoothing_alpha = float(
            self.get_parameter("action_smoothing_alpha").value
        )
        self.gripper_threshold_m = float(
            self.get_parameter("gripper_threshold_m").value
        )
        self.action_chunk_steps = int(self.get_parameter("action_chunk_steps").value)
        self.interpolate_commands = parse_bool(
            self.get_parameter("interpolate_commands").value
        )
        self.command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        self.gripper_min_m = float(self.get_parameter("gripper_min_m").value)
        self.gripper_max_m = float(self.get_parameter("gripper_max_m").value)
        self.move_to_home_on_start = parse_bool(
            self.get_parameter("move_to_home_on_start").value
        )
        self.left_home_joints = self._parse_home_joint_positions(
            self.get_parameter("left_home_joint_positions").value
        )
        self.right_home_joints = self._parse_home_joint_positions(
            self.get_parameter("right_home_joint_positions").value
        )
        self.home_gripper_position = float(
            self.get_parameter("home_gripper_position").value
        )
        self.home_tolerance_rad = float(self.get_parameter("home_tolerance_rad").value)
        self.home_timeout_s = float(self.get_parameter("home_timeout_s").value)
        self.home_settle_s = float(self.get_parameter("home_settle_s").value)
        self.home_max_joint_step_rad = float(
            self.get_parameter("home_max_joint_step_rad").value
        )
        self.home_max_gripper_step_m = float(
            self.get_parameter("home_max_gripper_step_m").value
        )
        self.log_actions = parse_bool(self.get_parameter("log_actions").value)
        if self.inference_rate_hz <= 0:
            raise ValueError("inference_rate_hz must be greater than zero")
        if self.command_rate_hz <= 0:
            raise ValueError("command_rate_hz must be greater than zero")
        if self.action_chunk_steps < 1:
            raise ValueError("action_chunk_steps must be at least 1")
        if self.action_mode not in ("absolute", "delta_joint_abs_gripper"):
            raise ValueError(
                "action_mode must be 'absolute' or 'delta_joint_abs_gripper'"
            )
        if not 0.0 < self.action_smoothing_alpha <= 1.0:
            raise ValueError("action_smoothing_alpha must be in the range (0.0, 1.0]")
        if self.gripper_threshold_m >= 0.0:
            self.gripper_threshold_m = float(
                np.clip(
                    self.gripper_threshold_m,
                    self.gripper_min_m,
                    self.gripper_max_m,
                )
            )
        self.home_gripper_position = float(
            np.clip(
                self.home_gripper_position,
                self.gripper_min_m,
                self.gripper_max_m,
            )
        )

    @staticmethod
    def _parse_home_joint_positions(value) -> np.ndarray:
        if not isinstance(value, str):
            raise ValueError("home joint positions must be a comma-separated string")
        try:
            joints = np.asarray(
                [float(item.strip()) for item in value.split(",") if item.strip()],
                dtype=np.float32,
            )
        except ValueError as exc:
            raise ValueError(
                "home joint positions must contain numeric values"
            ) from exc
        if joints.shape != (6,):
            raise ValueError("home joint positions must contain exactly 6 values")
        return joints

    def _load_policy(self) -> None:
        if not self.policy_path.exists():
            raise RuntimeError("policy_path does not exist: %s" % self.policy_path)
        if not (self.policy_path / "policy_preprocessor.json").exists():
            pretrained_path = self.policy_path / "pretrained_model"
            if (pretrained_path / "policy_preprocessor.json").exists():
                self.get_logger().info(
                    "Using pretrained_model subdirectory in checkpoint: %s"
                    % pretrained_path
                )
                self.policy_path = pretrained_path
        SmolVLAPolicy, DataProcessorPipeline = import_smolvla_policy()

        self.get_logger().info("Loading policy processors from %s" % self.policy_path)
        overrides = {"device_processor": {"device": self.device}}
        self.preprocessor = DataProcessorPipeline.from_pretrained(
            self.policy_path,
            config_filename="policy_preprocessor.json",
            overrides=overrides,
        )
        self.postprocessor = DataProcessorPipeline.from_pretrained(
            self.policy_path,
            config_filename="policy_postprocessor.json",
        )

        self.get_logger().info("Loading SmolVLA policy from %s" % self.policy_path)
        started = time.monotonic()
        self.policy = SmolVLAPolicy.from_pretrained(
            self.policy_path,
            device=self.device,
        )
        self.policy.eval()
        self.get_logger().info(
            "Policy loaded in %.2f s" % (time.monotonic() - started)
        )

    def _subscribe_arm(self, side: str) -> None:
        self.create_subscription(
            JointState,
            f"/{side}/joint_states_single",
            lambda msg, arm_side=side: self._on_joint_state(arm_side, msg),
            20,
        )
        self.create_subscription(
            Pose,
            f"/{side}/end_pose",
            lambda msg, arm_side=side: self._on_ee_pose(arm_side, msg),
            20,
        )

    def _on_joint_state(self, side: str, msg: JointState) -> None:
        value = joint_state_to_array(msg)
        if value is not None:
            self.joint_state[side] = value

    def _on_ee_pose(self, side: str, msg: Pose) -> None:
        self.ee_pose[side] = pose_to_array(msg)

    def _make_observation(self) -> Tuple[Optional[dict], Tuple[str, ...]]:
        images = {}
        missing = []
        now = time.monotonic()
        for camera_name, receiver in self.image_receivers.items():
            image, _, image_time = receiver.latest()
            if image is None or now - image_time > self.image_timeout:
                missing.append("image:%s" % camera_name)
                continue
            target_height, target_width = self.camera_shapes[camera_name]
            if image.shape[:2] != (target_height, target_width):
                image = cv2.resize(image, (target_width, target_height))
            images[camera_name] = self._image_to_tensor(image)
        if len(images) != len(CAMERA_NAMES):
            return None, tuple(missing)

        if self.joint_state["left"] is None or self.joint_state["right"] is None:
            if self.joint_state["left"] is None:
                missing.append("state:left")
            if self.joint_state["right"] is None:
                missing.append("state:right")
            return None, tuple(missing)

        state = np.concatenate(
            (self.joint_state["left"], self.joint_state["right"])
        ).astype(np.float32)
        ee_pose = np.concatenate(
            (
                self.ee_pose["left"]
                if self.ee_pose["left"] is not None
                else np.zeros(7, dtype=np.float32),
                self.ee_pose["right"]
                if self.ee_pose["right"] is not None
                else np.zeros(7, dtype=np.float32),
            )
        ).astype(np.float32)

        return {
            "observation.images.main": images["main"],
            "observation.images.left_wrist": images["left_wrist"],
            "observation.images.right_wrist": images["right_wrist"],
            "observation.state": torch.from_numpy(state),
            "observation.ee_pose": torch.from_numpy(ee_pose),
            "observation.vr_pose": torch.zeros(14, dtype=torch.float32),
            "observation.vr_joy": torch.zeros(40, dtype=torch.float32),
            "task": self.task,
        }, ()

    @staticmethod
    def _image_to_tensor(image_rgb: np.ndarray) -> torch.Tensor:
        image = np.ascontiguousarray(image_rgb)
        tensor = torch.from_numpy(image).permute(2, 0, 1).to(torch.float32)
        return tensor / 255.0

    def _on_timer(self) -> None:
        if not self.home_complete:
            self._run_startup_home_move()
            return

        observation, missing = self._make_observation()
        if observation is None:
            self.get_logger().warn(
                "Inference skipped; missing=%s" % ",".join(missing),
                throttle_duration_sec=1.0,
            )
            return

        try:
            processed = self.preprocessor(observation)
            started = time.monotonic()
            with torch.inference_mode():
                if self.action_chunk_steps > 1:
                    action = self.policy.predict_action_chunk(processed)
                else:
                    action = self.policy.select_action(processed)
                output = self.postprocessor({"action": action})
            infer_s = time.monotonic() - started
        except Exception as exc:
            self.get_logger().error("Inference failed: %s" % exc)
            return

        actions_np = output["action"].detach().cpu().numpy().reshape(-1, 14)
        if self.action_chunk_steps > 1:
            self._set_action_queue(actions_np, infer_s)
            return

        action_np = actions_np[0]
        action_np = self._policy_action_to_target(action_np)
        action_np = self._smooth_policy_action(action_np)
        action_np = self._threshold_gripper_action(action_np)
        action_np = self._limit_action(action_np)
        self.last_action = action_np

        if self.log_actions and time.monotonic() - self.last_action_log_time > 1.0:
            self.last_action_log_time = time.monotonic()
            self.get_logger().info(
                "action infer_s=%.3f left=%s right=%s"
                % (
                    infer_s,
                    np.array2string(action_np[:7], precision=3, suppress_small=True),
                    np.array2string(action_np[7:], precision=3, suppress_small=True),
                )
            )

        if self.publish_commands:
            if self.interpolate_commands:
                self._start_command_segment(action_np)
            else:
                self._publish_joint_command(action_np)

    def _run_startup_home_move(self) -> None:
        if self.joint_state["left"] is None or self.joint_state["right"] is None:
            self.get_logger().warn(
                "Waiting for joint states before startup home move",
                throttle_duration_sec=1.0,
            )
            return

        now = time.monotonic()
        if self.home_started_at is None:
            self.home_started_at = now

        left_target = self._home_target("left")
        right_target = self._home_target("right")
        left_cmd = self._limit_home_step(self.joint_state["left"], left_target)
        right_cmd = self._limit_home_step(self.joint_state["right"], right_target)
        self.left_pub.publish(self._joint_msg(left_cmd))
        self.right_pub.publish(self._joint_msg(right_cmd))

        left_err = float(np.max(np.abs(left_target[:6] - self.joint_state["left"][:6])))
        right_err = float(
            np.max(np.abs(right_target[:6] - self.joint_state["right"][:6]))
        )
        reached = max(left_err, right_err) <= self.home_tolerance_rad
        timed_out = now - self.home_started_at >= self.home_timeout_s
        if reached and self.home_reached_at is None:
            self.home_reached_at = now
            self.get_logger().warn(
                "Startup home reached; settling for %.2f s" % self.home_settle_s
            )
        if reached and now - self.home_reached_at >= self.home_settle_s:
            self.home_complete = True
            self.get_logger().warn("Startup home move complete; policy inference starts")
        elif timed_out:
            self.home_complete = True
            self.get_logger().warn(
                "Startup home move timed out after %.1f s; policy inference starts"
                % self.home_timeout_s
            )
        else:
            self.get_logger().info(
                "Moving to startup home: left_err=%.3f right_err=%.3f"
                % (left_err, right_err),
                throttle_duration_sec=1.0,
            )

    def _home_target(self, side: str) -> np.ndarray:
        joints = self.left_home_joints if side == "left" else self.right_home_joints
        return np.concatenate(
            (joints, np.asarray([self.home_gripper_position], dtype=np.float32))
        ).astype(np.float32)

    def _limit_home_step(self, current: np.ndarray, target: np.ndarray) -> np.ndarray:
        limited = current.copy()
        limited[:6] = current[:6] + np.clip(
            target[:6] - current[:6],
            -self.home_max_joint_step_rad,
            self.home_max_joint_step_rad,
        )
        limited[6] = current[6] + np.clip(
            target[6] - current[6],
            -self.home_max_gripper_step_m,
            self.home_max_gripper_step_m,
        )
        limited[6] = np.clip(limited[6], self.gripper_min_m, self.gripper_max_m)
        return limited.astype(np.float32)

    def _limit_action(self, action: np.ndarray) -> np.ndarray:
        current = np.concatenate((self.joint_state["left"], self.joint_state["right"]))
        return self._limit_action_from_reference(action, current)

    def _limit_action_from_reference(
        self,
        action: np.ndarray,
        reference: np.ndarray,
    ) -> np.ndarray:
        reference = reference.astype(np.float32)
        limited = reference + np.clip(
            action - reference,
            -self.max_joint_step_rad,
            self.max_joint_step_rad,
        )
        for idx in (6, 13):
            limited[idx] = reference[idx] + np.clip(
                action[idx] - reference[idx],
                -self.max_gripper_step_m,
                self.max_gripper_step_m,
            )
            limited[idx] = np.clip(limited[idx], self.gripper_min_m, self.gripper_max_m)
        return limited.astype(np.float32)

    def _set_action_queue(self, actions: np.ndarray, infer_s: float) -> None:
        current = np.concatenate((self.joint_state["left"], self.joint_state["right"]))
        reference = self.last_command if self.last_command is not None else current
        queued = []
        for raw_action in actions[: self.action_chunk_steps]:
            target = self._policy_action_to_target(raw_action)
            target = self._smooth_policy_action(target)
            target = self._threshold_gripper_action(target)
            limited = self._limit_action_from_reference(target, reference)
            queued.append(limited)
            reference = limited

        self.action_queue = queued
        if queued:
            self.last_action = queued[-1]

        if self.log_actions and time.monotonic() - self.last_action_log_time > 1.0:
            self.last_action_log_time = time.monotonic()
            first = queued[0] if queued else current
            last = queued[-1] if queued else current
            self.get_logger().info(
                "action_chunk infer_s=%.3f steps=%d first_left=%s last_left=%s"
                % (
                    infer_s,
                    len(queued),
                    np.array2string(first[:7], precision=3, suppress_small=True),
                    np.array2string(last[:7], precision=3, suppress_small=True),
                )
            )

    def _publish_joint_command(self, command: np.ndarray) -> None:
        command = command.astype(np.float32)
        self.last_command = command
        self.left_pub.publish(self._joint_msg(command[:7]))
        self.right_pub.publish(self._joint_msg(command[7:]))

    def _start_command_segment(self, target: np.ndarray) -> None:
        now = time.monotonic()
        if self.last_command is not None:
            start = self.last_command
        else:
            start = np.concatenate(
                (self.joint_state["left"], self.joint_state["right"])
            ).astype(np.float32)

        self.command_segment_start = start.astype(np.float32)
        self.command_segment_target = target.astype(np.float32)
        self.command_segment_started_at = now
        self.command_segment_duration = max(1.0 / self.inference_rate_hz, 1e-3)

    def _on_command_timer(self) -> None:
        if self.action_chunk_steps > 1:
            if not self.publish_commands or not self.action_queue:
                return
            self._publish_joint_command(self.action_queue.pop(0))
            return

        if not self.publish_commands or self.command_segment_target is None:
            return

        now = time.monotonic()
        ratio = (now - self.command_segment_started_at) / self.command_segment_duration
        ratio = float(np.clip(ratio, 0.0, 1.0))
        smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
        command = self.command_segment_start + smooth_ratio * (
            self.command_segment_target - self.command_segment_start
        )
        command[6] = np.clip(command[6], self.gripper_min_m, self.gripper_max_m)
        command[13] = np.clip(command[13], self.gripper_min_m, self.gripper_max_m)
        self._publish_joint_command(command)

    def _policy_action_to_target(self, action: np.ndarray) -> np.ndarray:
        if self.action_mode == "absolute":
            return action.astype(np.float32)

        current = np.concatenate((self.joint_state["left"], self.joint_state["right"]))
        target = current.copy()
        joint_indices = [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]
        target[joint_indices] = current[joint_indices] + action[joint_indices]
        target[6] = action[6]
        target[13] = action[13]
        return target.astype(np.float32)

    def _smooth_policy_action(self, action: np.ndarray) -> np.ndarray:
        if self.action_smoothing_alpha >= 1.0:
            return action.astype(np.float32)

        if self.smoothed_policy_action is None:
            self.smoothed_policy_action = np.concatenate(
                (self.joint_state["left"], self.joint_state["right"])
            ).astype(np.float32)

        alpha = self.action_smoothing_alpha
        smoothed = alpha * action + (1.0 - alpha) * self.smoothed_policy_action
        for idx in (6, 13):
            smoothed[idx] = np.clip(
                smoothed[idx],
                self.gripper_min_m,
                self.gripper_max_m,
            )
        self.smoothed_policy_action = smoothed.astype(np.float32)
        return self.smoothed_policy_action

    def _threshold_gripper_action(self, action: np.ndarray) -> np.ndarray:
        if self.gripper_threshold_m < 0.0:
            return action.astype(np.float32)

        thresholded = action.copy()
        for idx in (6, 13):
            thresholded[idx] = (
                self.gripper_max_m
                if thresholded[idx] >= self.gripper_threshold_m
                else self.gripper_min_m
            )
        return thresholded.astype(np.float32)

    def _joint_msg(self, joints: np.ndarray) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(value) for value in joints]
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        return msg

    def destroy_node(self) -> bool:
        for receiver in getattr(self, "image_receivers", {}).values():
            receiver.stop()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = SmolVLAPiperInference()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        print(str(exc))
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
