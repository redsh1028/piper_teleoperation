import json
import math
import os
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, String
from urdf_parser_py.urdf import URDF

from vr_udp_bridge.vr_piper_pose_teleop import (
    apply_deadzone,
    axis_map_to_text,
    clamp,
    copy_pose,
    limit_vector,
    map_quat,
    mat_vec,
    parse_axis_map,
    parse_bool,
    quat_from_pose,
    quat_from_pose_stamped,
    quat_inverse,
    quat_multiply,
    quat_normalize,
    quat_to_matrix,
    scale_quat_rotation,
    set_pose_orientation,
)


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


@dataclass
class ChainJoint:
    name: str
    joint_type: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


@dataclass
class ArmState:
    name: str
    vr_pose: Optional[PoseStamped] = None
    vr_pose_time: Optional[float] = None
    joy: Optional[Joy] = None
    joy_time: Optional[float] = None
    status: Optional[dict] = None
    status_time: Optional[float] = None
    robot_pose: Optional[Pose] = None
    current_joints: Optional[np.ndarray] = None
    current_gripper: float = 0.0
    command_joints: Optional[np.ndarray] = None
    vr_anchor: Optional[PoseStamped] = None
    robot_anchor: Optional[Pose] = None
    enabled_last_cycle: bool = False
    position_history: Deque[tuple] = field(default_factory=deque)
    orientation_history: Deque[tuple] = field(default_factory=deque)
    gripper_closed: bool = False
    gripper_button_last: bool = False


class PiperKinematics:
    def __init__(
        self,
        urdf_path: str,
        base_link: str,
        tip_link: str,
        moveit_joint_limits_path: str = "",
    ) -> None:
        self.robot = load_urdf(urdf_path)
        self.base_link = base_link
        self.tip_link = tip_link
        self.joints = self._build_chain()
        self.active_joint_indices = [
            idx for idx, joint in enumerate(self.joints) if joint.name in JOINT_NAMES
        ]
        if [self.joints[idx].name for idx in self.active_joint_indices] != JOINT_NAMES:
            raise ValueError("URDF chain must contain joint1..joint6 in order")
        self.lower, self.upper = self._joint_bounds(moveit_joint_limits_path)

    def _build_chain(self) -> List[ChainJoint]:
        child_to_joint = {joint.child: joint for joint in self.robot.joints}
        link = self.tip_link
        joints_from_tip = []
        while link != self.base_link:
            if link not in child_to_joint:
                raise ValueError(f"No joint from {self.base_link} toward {self.tip_link}")
            joint = child_to_joint[link]
            joints_from_tip.append(joint)
            link = joint.parent

        chain = []
        for joint in reversed(joints_from_tip):
            origin = joint.origin
            xyz = np.array(origin.xyz if origin and origin.xyz else [0.0, 0.0, 0.0], dtype=float)
            rpy = np.array(origin.rpy if origin and origin.rpy else [0.0, 0.0, 0.0], dtype=float)
            axis = np.array(joint.axis if joint.axis else [0.0, 0.0, 1.0], dtype=float)
            norm = np.linalg.norm(axis)
            if norm > 0.0:
                axis = axis / norm
            lower = -math.inf
            upper = math.inf
            if joint.limit is not None:
                lower = float(joint.limit.lower)
                upper = float(joint.limit.upper)
            chain.append(
                ChainJoint(
                    name=joint.name,
                    joint_type=joint.type,
                    origin_xyz=xyz,
                    origin_rpy=rpy,
                    axis=axis,
                    lower=lower,
                    upper=upper,
                )
            )
        return chain

    def _joint_bounds(self, moveit_joint_limits_path: str) -> Tuple[np.ndarray, np.ndarray]:
        lower = np.array(
            [self.joints[idx].lower for idx in self.active_joint_indices],
            dtype=float,
        )
        upper = np.array(
            [self.joints[idx].upper for idx in self.active_joint_indices],
            dtype=float,
        )
        if moveit_joint_limits_path:
            overrides = load_moveit_joint_limits(moveit_joint_limits_path)
            for idx, name in enumerate(JOINT_NAMES):
                if name in overrides:
                    if "min_position" in overrides[name]:
                        lower[idx] = float(overrides[name]["min_position"])
                    if "max_position" in overrides[name]:
                        upper[idx] = float(overrides[name]["max_position"])
        return lower, upper

    def fk(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        transform = np.eye(4)
        active_cursor = 0
        for joint in self.joints:
            transform = transform @ transform_from_xyz_rpy(joint.origin_xyz, joint.origin_rpy)
            if joint.joint_type in ("revolute", "continuous"):
                angle = float(q[active_cursor])
                transform = transform @ axis_angle_transform(joint.axis, angle)
                active_cursor += 1
            elif joint.joint_type == "prismatic":
                active_cursor += 1
        return transform[:3, 3].copy(), transform[:3, :3].copy()

    def solve(
        self,
        target_position: np.ndarray,
        target_rotation: np.ndarray,
        seed: np.ndarray,
        ik_position_weight: float,
        ik_orientation_weight: float,
        joint_continuity_weight: float,
        ik_max_iterations: int,
    ) -> np.ndarray:
        target_position = np.asarray(target_position, dtype=float)
        target_rotation = np.asarray(target_rotation, dtype=float)
        seed = np.clip(seed, self.lower, self.upper)

        def residual(q: np.ndarray) -> np.ndarray:
            position, rotation = self.fk(q)
            position_error = target_position - position
            orientation_error = Rotation.from_matrix(target_rotation @ rotation.T).as_rotvec()
            continuity_error = q - seed
            return np.concatenate(
                (
                    ik_position_weight * position_error,
                    ik_orientation_weight * orientation_error,
                    joint_continuity_weight * continuity_error,
                )
            )

        result = least_squares(
            residual,
            seed,
            bounds=(self.lower, self.upper),
            max_nfev=ik_max_iterations,
            xtol=1e-4,
            ftol=1e-4,
            gtol=1e-4,
        )
        return np.clip(result.x, self.lower, self.upper)


class VrPiperJointIkTeleop(Node):
    def __init__(self) -> None:
        super().__init__("vr_piper_joint_ik_teleop")

        self.declare_parameter("axis_map", "-z,-x,y")
        self.declare_parameter("controller_axis_map", "-z,-x,y")
        self.declare_parameter("position_scale", 0.8)
        self.declare_parameter("orientation_scale", 0.9)
        self.declare_parameter("publish_rate_hz", 40.0)
        self.declare_parameter("input_timeout_sec", 0.25)
        self.declare_parameter("position_deadzone", 0.002)
        self.declare_parameter("max_target_offset_m", 0.35)
        self.declare_parameter("position_reference_frame", "controller_anchor")
        self.declare_parameter("orientation_reference_frame", "controller_anchor")
        self.declare_parameter("lock_orientation", False)
        self.declare_parameter("enable_button_index", 0)
        self.declare_parameter("trigger_axis_index", 2)
        self.declare_parameter("gripper_toggle_button_index", 1)
        self.declare_parameter("gripper_mode", "toggle")
        self.declare_parameter("filter_window_size", 3)
        self.declare_parameter("debug_axis", False)
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("moveit_joint_limits_path", "")
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tip_link", "gripper_base")
        self.declare_parameter("ik_position_weight", 1.0)
        self.declare_parameter("ik_orientation_weight", 0.6)
        self.declare_parameter("joint_continuity_weight", 0.25)
        self.declare_parameter("max_joint_step_rad", 0.05)
        self.declare_parameter("ik_max_iterations", 30)

        self.axis_map = parse_axis_map(self.get_parameter("axis_map").value)
        self.controller_axis_map = parse_axis_map(
            self.get_parameter("controller_axis_map").value
        )
        self.position_scale = float(self.get_parameter("position_scale").value)
        self.orientation_scale = float(self.get_parameter("orientation_scale").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.input_timeout_sec = float(self.get_parameter("input_timeout_sec").value)
        self.position_deadzone = float(self.get_parameter("position_deadzone").value)
        self.max_target_offset_m = float(
            self.get_parameter("max_target_offset_m").value
        )
        self.position_reference_frame = str(
            self.get_parameter("position_reference_frame").value
        ).lower()
        self.orientation_reference_frame = str(
            self.get_parameter("orientation_reference_frame").value
        ).lower()
        self.lock_orientation = parse_bool(self.get_parameter("lock_orientation").value)
        self.enable_button_index = int(self.get_parameter("enable_button_index").value)
        self.trigger_axis_index = int(self.get_parameter("trigger_axis_index").value)
        self.gripper_toggle_button_index = int(
            self.get_parameter("gripper_toggle_button_index").value
        )
        self.gripper_mode = str(self.get_parameter("gripper_mode").value).lower()
        self.filter_window_size = int(self.get_parameter("filter_window_size").value)
        self.debug_axis = parse_bool(self.get_parameter("debug_axis").value)
        self.ik_position_weight = float(
            self.get_parameter("ik_position_weight").value
        )
        self.ik_orientation_weight = float(
            self.get_parameter("ik_orientation_weight").value
        )
        self.joint_continuity_weight = float(
            self.get_parameter("joint_continuity_weight").value
        )
        self.max_joint_step_rad = float(self.get_parameter("max_joint_step_rad").value)
        self.ik_max_iterations = int(self.get_parameter("ik_max_iterations").value)

        validate_positive("publish_rate_hz", self.publish_rate_hz)
        validate_positive("input_timeout_sec", self.input_timeout_sec)
        validate_positive("max_target_offset_m", self.max_target_offset_m)
        validate_positive("ik_position_weight", self.ik_position_weight)
        validate_positive("ik_orientation_weight", self.ik_orientation_weight)
        validate_positive("max_joint_step_rad", self.max_joint_step_rad)
        if self.position_deadzone < 0.0:
            raise ValueError("position_deadzone must be zero or greater")
        if self.position_reference_frame not in ("controller_anchor", "vr_world"):
            raise ValueError(
                "position_reference_frame must be 'controller_anchor' or 'vr_world'"
            )
        if self.orientation_reference_frame not in ("controller_anchor", "vr_world"):
            raise ValueError(
                "orientation_reference_frame must be 'controller_anchor' or 'vr_world'"
            )
        if self.gripper_mode not in ("trigger", "toggle"):
            raise ValueError("gripper_mode must be 'trigger' or 'toggle'")
        if self.filter_window_size < 1:
            raise ValueError("filter_window_size must be at least one")
        if self.ik_max_iterations < 1:
            raise ValueError("ik_max_iterations must be at least one")

        urdf_path = str(self.get_parameter("urdf_path").value)
        if not urdf_path:
            urdf_path = default_package_path(
                "piper_description",
                "urdf/piper_description.urdf",
            )
        moveit_limits_path = str(self.get_parameter("moveit_joint_limits_path").value)
        if not moveit_limits_path:
            moveit_limits_path = default_package_path(
                "piper_moveit",
                "config/joint_limits.yaml",
                required=False,
            )

        self.kinematics = PiperKinematics(
            urdf_path=urdf_path,
            base_link=str(self.get_parameter("base_link").value),
            tip_link=str(self.get_parameter("tip_link").value),
            moveit_joint_limits_path=moveit_limits_path,
        )

        self.arms: Dict[str, ArmState] = {
            "left": ArmState("left"),
            "right": ArmState("right"),
        }
        self.last_warn_time: Dict[str, float] = {}
        self.last_debug_time: Dict[str, float] = {}

        self.left_joint_pub = self.create_publisher(
            JointState,
            "/left/joint_ctrl_single",
            10,
        )
        self.right_joint_pub = self.create_publisher(
            JointState,
            "/right/joint_ctrl_single",
            10,
        )
        self.left_gripper_pub = self.create_publisher(Bool, "/left/gripper_ctrl", 10)
        self.right_gripper_pub = self.create_publisher(Bool, "/right/gripper_ctrl", 10)

        self._subscribe_arm("left")
        self._subscribe_arm("right")

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)
        self.get_logger().info(
            "VR Piper joint IK teleop started with controller_axis_map=%s, position_scale=%.3f, orientation_scale=%.3f"
            % (
                axis_map_to_text(self.controller_axis_map),
                self.position_scale,
                self.orientation_scale,
            )
        )

    def _subscribe_arm(self, side: str) -> None:
        self.create_subscription(
            PoseStamped,
            f"/vr/{side}_controller/pose",
            lambda msg, arm_side=side: self._on_vr_pose(arm_side, msg),
            10,
        )
        self.create_subscription(
            Joy,
            f"/vr/{side}_controller/joy",
            lambda msg, arm_side=side: self._on_joy(arm_side, msg),
            10,
        )
        self.create_subscription(
            String,
            f"/vr/{side}_controller/status",
            lambda msg, arm_side=side: self._on_status(arm_side, msg),
            10,
        )
        self.create_subscription(
            Pose,
            f"/{side}/end_pose",
            lambda msg, arm_side=side: self._on_robot_pose(arm_side, msg),
            10,
        )
        self.create_subscription(
            JointState,
            f"/{side}/joint_states_single",
            lambda msg, arm_side=side: self._on_joint_state(arm_side, msg),
            10,
        )

    def _on_vr_pose(self, side: str, msg: PoseStamped) -> None:
        arm = self.arms[side]
        arm.vr_pose = self._filtered_pose(arm, msg)
        arm.vr_pose_time = time.monotonic()

    def _on_joy(self, side: str, msg: Joy) -> None:
        arm = self.arms[side]
        arm.joy = msg
        arm.joy_time = time.monotonic()

    def _on_status(self, side: str, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            self._warn_throttled(f"{side}_bad_status", f"Ignoring bad {side} status")
            return
        if isinstance(status, dict):
            arm = self.arms[side]
            arm.status = status
            arm.status_time = time.monotonic()

    def _on_robot_pose(self, side: str, msg: Pose) -> None:
        self.arms[side].robot_pose = msg

    def _on_joint_state(self, side: str, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in JOINT_NAMES):
            self.arms[side].current_joints = np.array(
                [positions[name] for name in JOINT_NAMES],
                dtype=float,
            )
        if "joint7" in positions:
            self.arms[side].current_gripper = float(positions["joint7"])
        elif len(msg.position) >= 7:
            self.arms[side].current_gripper = float(msg.position[6])

    def _on_timer(self) -> None:
        now = time.monotonic()
        self._handle_arm("left", self.left_joint_pub, self.left_gripper_pub, now)
        self._handle_arm("right", self.right_joint_pub, self.right_gripper_pub, now)

    def _handle_arm(self, side: str, joint_pub, gripper_pub, now: float) -> None:
        arm = self.arms[side]
        self._handle_gripper_toggle(arm, gripper_pub, now)

        if not self._enabled(arm) or not self._input_ready(arm, now):
            arm.vr_anchor = None
            arm.robot_anchor = None
            arm.enabled_last_cycle = False
            return

        if not arm.enabled_last_cycle or arm.vr_anchor is None or arm.robot_anchor is None:
            if arm.robot_pose is None:
                self._warn_throttled(
                    f"{side}_missing_robot_pose",
                    f"{side} Piper end_pose is missing; joint IK command is not published",
                )
                return
            if arm.current_joints is None:
                self._warn_throttled(
                    f"{side}_missing_joint_state",
                    f"{side} joint_states_single is missing; joint IK command is not published",
                )
                return
            arm.vr_anchor = arm.vr_pose
            arm.robot_anchor = copy_pose(arm.robot_pose)
            arm.command_joints = arm.current_joints.copy()
            arm.enabled_last_cycle = True
            self.get_logger().info(f"{side} joint IK anchor reset")
            return

        target_pose, vr_delta, delta_p_base = self._target_pose(arm)
        seed = self._ik_seed(arm)
        q_solved = self.kinematics.solve(
            target_position=pose_position_array(target_pose),
            target_rotation=quat_to_matrix(quat_from_pose(target_pose)),
            seed=seed,
            ik_position_weight=self.ik_position_weight,
            ik_orientation_weight=self.ik_orientation_weight,
            joint_continuity_weight=self.joint_continuity_weight,
            ik_max_iterations=self.ik_max_iterations,
        )
        q_cmd = seed + np.clip(
            q_solved - seed,
            -self.max_joint_step_rad,
            self.max_joint_step_rad,
        )
        q_cmd = np.clip(q_cmd, self.kinematics.lower, self.kinematics.upper)
        arm.command_joints = q_cmd
        joint_pub.publish(self._joint_command_msg(q_cmd, self._gripper_position(arm)))

        if self.debug_axis:
            self._debug_axis_throttled(arm.name, vr_delta, delta_p_base, q_cmd)

    def _input_ready(self, arm: ArmState, now: float) -> bool:
        if arm.vr_pose is None or arm.vr_pose_time is None:
            self._warn_throttled(
                f"{arm.name}_missing_vr_pose",
                f"{arm.name} VR pose is missing",
            )
            return False
        if now - arm.vr_pose_time > self.input_timeout_sec:
            self._warn_throttled(
                f"{arm.name}_stale_vr_pose",
                f"{arm.name} VR pose timed out",
            )
            return False
        if arm.joy is None or arm.joy_time is None:
            self._warn_throttled(
                f"{arm.name}_missing_joy",
                f"{arm.name} VR joy is missing",
            )
            return False
        if now - arm.joy_time > self.input_timeout_sec:
            self._warn_throttled(
                f"{arm.name}_stale_joy",
                f"{arm.name} VR joy timed out",
            )
            return False
        if arm.status is not None and arm.status.get("tracking_valid") is False:
            self._warn_throttled(
                f"{arm.name}_tracking_invalid",
                f"{arm.name} VR tracking is invalid",
            )
            return False
        return True

    def _enabled(self, arm: ArmState) -> bool:
        if arm.joy is None:
            return False
        if self.enable_button_index < 0 or self.enable_button_index >= len(arm.joy.buttons):
            self._warn_throttled(
                f"{arm.name}_bad_enable_button",
                "enable_button_index %d is outside %s Joy.buttons"
                % (self.enable_button_index, arm.name),
            )
            return False
        return int(arm.joy.buttons[self.enable_button_index]) == 1

    def _target_pose(self, arm: ArmState) -> Tuple[Pose, Tuple[float, float, float], Tuple[float, float, float]]:
        if arm.vr_anchor is None or arm.robot_anchor is None or arm.vr_pose is None:
            raise RuntimeError("teleop anchors are not initialized")

        vr_delta_world = (
            arm.vr_pose.pose.position.x - arm.vr_anchor.pose.position.x,
            arm.vr_pose.pose.position.y - arm.vr_anchor.pose.position.y,
            arm.vr_pose.pose.position.z - arm.vr_anchor.pose.position.z,
        )
        if self.position_reference_frame == "controller_anchor":
            local_delta = rotate_vector(
                transpose3(quat_to_matrix(quat_from_pose_stamped(arm.vr_anchor))),
                vr_delta_world,
            )
            mapped_vr_delta = mat_vec(self.controller_axis_map, local_delta)
        else:
            mapped_vr_delta = mat_vec(self.axis_map, vr_delta_world)
        delta_p_base = limit_vector(
            apply_deadzone(mapped_vr_delta, self.position_deadzone),
            self.max_target_offset_m,
        )

        target_pose = copy_pose(arm.robot_anchor)
        target_pose.position.x += self.position_scale * delta_p_base[0]
        target_pose.position.y += self.position_scale * delta_p_base[1]
        target_pose.position.z += self.position_scale * delta_p_base[2]

        q_target = quat_from_pose(arm.robot_anchor)
        if not self.lock_orientation:
            if self.orientation_reference_frame == "controller_anchor":
                q_delta_local = quat_multiply(
                    quat_inverse(quat_from_pose_stamped(arm.vr_anchor)),
                    quat_from_pose_stamped(arm.vr_pose),
                )
                q_delta_base = map_quat(self.controller_axis_map, q_delta_local)
                q_delta_base = scale_quat_rotation(
                    q_delta_base,
                    self.orientation_scale,
                )
            else:
                q_vr_current = map_quat(
                    self.axis_map,
                    quat_from_pose_stamped(arm.vr_pose),
                )
                q_vr_anchor = map_quat(
                    self.axis_map,
                    quat_from_pose_stamped(arm.vr_anchor),
                )
                q_delta_base = quat_multiply(q_vr_current, quat_inverse(q_vr_anchor))
                q_delta_base = scale_quat_rotation(q_delta_base, self.orientation_scale)
            q_target = quat_normalize(
                quat_multiply(q_delta_base, quat_from_pose(arm.robot_anchor))
            )
        set_pose_orientation(target_pose, q_target)
        return target_pose, vr_delta_world, delta_p_base

    def _ik_seed(self, arm: ArmState) -> np.ndarray:
        if arm.command_joints is not None:
            return arm.command_joints.copy()
        if arm.current_joints is not None:
            return arm.current_joints.copy()
        return np.zeros(6)

    def _joint_command_msg(self, joints: np.ndarray, gripper: float) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES + ["joint7"]
        msg.position = [float(value) for value in joints] + [float(gripper)]
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        return msg

    def _gripper_position(self, arm: ArmState) -> float:
        if self.gripper_mode == "toggle":
            return 0.0 if arm.gripper_closed else 0.07
        if arm.joy is None:
            return arm.current_gripper
        if self.trigger_axis_index < 0 or self.trigger_axis_index >= len(arm.joy.axes):
            return arm.current_gripper
        close_amount = clamp(float(arm.joy.axes[self.trigger_axis_index]), 0.0, 1.0)
        return 0.07 * (1.0 - close_amount)

    def _handle_gripper_toggle(self, arm: ArmState, gripper_pub, now: float) -> None:
        if self.gripper_mode != "toggle":
            return
        if arm.joy is None or arm.joy_time is None:
            return
        if now - arm.joy_time > self.input_timeout_sec:
            return
        if (
            self.gripper_toggle_button_index < 0
            or self.gripper_toggle_button_index >= len(arm.joy.buttons)
        ):
            self._warn_throttled(
                f"{arm.name}_bad_gripper_button",
                "gripper_toggle_button_index %d is outside %s Joy.buttons"
                % (self.gripper_toggle_button_index, arm.name),
            )
            return

        pressed = int(arm.joy.buttons[self.gripper_toggle_button_index]) == 1
        if pressed and not arm.gripper_button_last:
            arm.gripper_closed = not arm.gripper_closed
            gripper_pub.publish(Bool(data=arm.gripper_closed))
            state = "closed" if arm.gripper_closed else "open"
            self.get_logger().info(f"{arm.name} gripper toggled {state}")
        arm.gripper_button_last = pressed

    def _filtered_pose(self, arm: ArmState, msg: PoseStamped) -> PoseStamped:
        if self.filter_window_size <= 1:
            return msg

        arm.position_history.append(
            (
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            )
        )
        arm.orientation_history.append(
            quat_normalize(
                (
                    float(msg.pose.orientation.x),
                    float(msg.pose.orientation.y),
                    float(msg.pose.orientation.z),
                    float(msg.pose.orientation.w),
                )
            )
        )
        while len(arm.position_history) > self.filter_window_size:
            arm.position_history.popleft()
        while len(arm.orientation_history) > self.filter_window_size:
            arm.orientation_history.popleft()

        filtered = PoseStamped()
        filtered.header = msg.header
        count = float(len(arm.position_history))
        filtered.pose.position.x = sum(p[0] for p in arm.position_history) / count
        filtered.pose.position.y = sum(p[1] for p in arm.position_history) / count
        filtered.pose.position.z = sum(p[2] for p in arm.position_history) / count

        q_count = float(len(arm.orientation_history))
        q_avg = quat_normalize(
            (
                sum(q[0] for q in arm.orientation_history) / q_count,
                sum(q[1] for q in arm.orientation_history) / q_count,
                sum(q[2] for q in arm.orientation_history) / q_count,
                sum(q[3] for q in arm.orientation_history) / q_count,
            )
        )
        filtered.pose.orientation.x = q_avg[0]
        filtered.pose.orientation.y = q_avg[1]
        filtered.pose.orientation.z = q_avg[2]
        filtered.pose.orientation.w = q_avg[3]
        return filtered

    def _warn_throttled(self, key: str, message: str, period: float = 1.0) -> None:
        now = time.monotonic()
        last = self.last_warn_time.get(key)
        if last is None or now - last >= period:
            self.last_warn_time[key] = now
            self.get_logger().warn(message)

    def _debug_axis_throttled(self, side: str, vr_delta, delta_p_base, q_cmd, period: float = 0.5) -> None:
        now = time.monotonic()
        last = self.last_debug_time.get(side)
        if last is not None and now - last < period:
            return
        self.last_debug_time[side] = now
        self.get_logger().info(
            "%s joint IK vr_delta=(%.3f, %.3f, %.3f) delta_p_base=(%.3f, %.3f, %.3f) q=(%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)"
            % (side, *vr_delta, *delta_p_base, *q_cmd)
        )


def validate_positive(name: str, value: float) -> None:
    if value <= 0.0:
        raise ValueError(f"{name} must be greater than zero")


def default_package_path(package_name: str, relative_path: str, required: bool = True) -> str:
    try:
        path = os.path.join(get_package_share_directory(package_name), relative_path)
    except PackageNotFoundError:
        if required:
            raise
        return ""
    if required and not os.path.exists(path):
        raise FileNotFoundError(path)
    if not os.path.exists(path):
        return ""
    return path


def load_moveit_joint_limits(path: str) -> Dict[str, dict]:
    try:
        import yaml
    except ImportError:
        return {}
    if not path or not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    limits = data.get("joint_limits", {})
    return limits if isinstance(limits, dict) else {}


def load_urdf(path: str) -> URDF:
    with open(path, "r", encoding="utf-8") as handle:
        xml = handle.read()
    if xml.startswith("<?xml"):
        xml = xml.split("?>", 1)[1].lstrip()
    return URDF.from_xml_string(xml)


def transform_from_xyz_rpy(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    transform[:3, 3] = xyz
    return transform


def axis_angle_transform(axis: np.ndarray, angle: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
    return transform


def pose_position_array(pose: Pose) -> np.ndarray:
    return np.array(
        [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        ],
        dtype=float,
    )


def rotate_vector(matrix, vector):
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def transpose3(matrix):
    return (
        (matrix[0][0], matrix[1][0], matrix[2][0]),
        (matrix[0][1], matrix[1][1], matrix[2][1]),
        (matrix[0][2], matrix[1][2], matrix[2][2]),
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VrPiperJointIkTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
