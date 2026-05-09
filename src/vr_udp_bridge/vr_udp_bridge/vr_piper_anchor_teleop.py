import json
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from piper_msgs.msg import PosCmd
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

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
    quat_to_euler,
    scale_quat_rotation,
    set_pose_orientation,
)


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
    vr_anchor: Optional[PoseStamped] = None
    robot_anchor: Optional[Pose] = None
    commanded_pose: Optional[Pose] = None
    command_time: Optional[float] = None
    enabled_last_cycle: bool = False
    position_history: Deque[tuple] = field(default_factory=deque)
    orientation_history: Deque[tuple] = field(default_factory=deque)
    gripper_closed: bool = False
    gripper_button_last: bool = False


class VrPiperAnchorTeleop(Node):
    def __init__(self) -> None:
        super().__init__("vr_piper_anchor_teleop")

        self.declare_parameter("axis_map", "-z,-x,-y")
        self.declare_parameter("controller_axis_map", "-z,-x,-y")
        self.declare_parameter("position_scale", 0.5)
        self.declare_parameter("orientation_scale", 0.2)
        self.declare_parameter("publish_rate_hz", 60.0)
        self.declare_parameter("input_timeout_sec", 0.25)
        self.declare_parameter("position_deadzone", 0.002)
        self.declare_parameter("max_target_offset_m", 0.35)
        self.declare_parameter("smooth_command", False)
        self.declare_parameter("max_linear_speed_mps", 0.25)
        self.declare_parameter("max_angular_speed_radps", 0.8)
        self.declare_parameter("position_reference_frame", "controller_anchor")
        self.declare_parameter("orientation_reference_frame", "controller_anchor")
        self.declare_parameter("lock_orientation", False)
        self.declare_parameter("enable_button_index", 0)
        self.declare_parameter("trigger_axis_index", 2)
        self.declare_parameter("gripper_toggle_button_index", 1)
        self.declare_parameter("gripper_mode", "toggle")
        self.declare_parameter("filter_window_size", 3)
        self.declare_parameter("debug_axis", False)
        self.declare_parameter("move_mode", 0)
        self.declare_parameter("mode1", 1)

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
        self.smooth_command = parse_bool(self.get_parameter("smooth_command").value)
        self.max_linear_speed_mps = float(
            self.get_parameter("max_linear_speed_mps").value
        )
        self.max_angular_speed_radps = float(
            self.get_parameter("max_angular_speed_radps").value
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
        self.move_mode = int(self.get_parameter("move_mode").value)
        self.mode1 = int(self.get_parameter("mode1").value)

        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")
        if self.input_timeout_sec <= 0.0:
            raise ValueError("input_timeout_sec must be greater than zero")
        if self.position_deadzone < 0.0:
            raise ValueError("position_deadzone must be zero or greater")
        if self.max_target_offset_m <= 0.0:
            raise ValueError("max_target_offset_m must be greater than zero")
        if self.max_linear_speed_mps <= 0.0:
            raise ValueError("max_linear_speed_mps must be greater than zero")
        if self.max_angular_speed_radps <= 0.0:
            raise ValueError("max_angular_speed_radps must be greater than zero")
        if self.position_reference_frame not in ("controller_anchor", "vr_world"):
            raise ValueError(
                "position_reference_frame must be 'controller_anchor' or 'vr_world'"
            )
        if self.orientation_reference_frame not in ("controller_anchor", "vr_world"):
            raise ValueError(
                "orientation_reference_frame must be 'controller_anchor' or 'vr_world'"
            )
        if self.filter_window_size < 1:
            raise ValueError("filter_window_size must be at least one")
        if self.gripper_mode not in ("trigger", "toggle"):
            raise ValueError("gripper_mode must be 'trigger' or 'toggle'")

        self.arms: Dict[str, ArmState] = {
            "left": ArmState("left"),
            "right": ArmState("right"),
        }
        self.last_warn_time: Dict[str, float] = {}
        self.last_debug_time: Dict[str, float] = {}

        self.left_pos_pub = self.create_publisher(PosCmd, "/left/pos_cmd", 10)
        self.right_pos_pub = self.create_publisher(PosCmd, "/right/pos_cmd", 10)
        self.left_gripper_pub = self.create_publisher(Bool, "/left/gripper_ctrl", 10)
        self.right_gripper_pub = self.create_publisher(Bool, "/right/gripper_ctrl", 10)
        self._subscribe_arm("left")
        self._subscribe_arm("right")

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)
        self.get_logger().info(
            "VR Piper anchor teleop started with axis_map=%s, controller_axis_map=%s, position_scale=%.3f, orientation_scale=%.3f, position_reference_frame=%s, orientation_reference_frame=%s"
            % (
                axis_map_to_text(self.axis_map),
                axis_map_to_text(self.controller_axis_map),
                self.position_scale,
                self.orientation_scale,
                self.position_reference_frame,
                self.orientation_reference_frame,
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

    def _on_timer(self) -> None:
        now = time.monotonic()
        self._handle_arm("left", self.left_pos_pub, self.left_gripper_pub, now)
        self._handle_arm("right", self.right_pos_pub, self.right_gripper_pub, now)

    def _handle_arm(self, side: str, pos_pub, gripper_pub, now: float) -> None:
        arm = self.arms[side]
        self._handle_gripper_toggle(arm, gripper_pub, now)

        enabled = self._enabled(arm)
        input_ready = self._input_ready(arm, now)
        if not enabled or not input_ready:
            arm.vr_anchor = None
            arm.robot_anchor = None
            arm.commanded_pose = None
            arm.command_time = None
            arm.enabled_last_cycle = False
            return

        if not arm.enabled_last_cycle or arm.vr_anchor is None or arm.robot_anchor is None:
            if arm.robot_pose is None:
                self._warn_throttled(
                    f"{side}_missing_robot_pose",
                    f"{side} Piper end_pose is missing; not publishing PosCmd",
                )
                return
            arm.vr_anchor = arm.vr_pose
            arm.robot_anchor = copy_pose(arm.robot_pose)
            arm.commanded_pose = copy_pose(arm.robot_pose)
            arm.command_time = now
            arm.enabled_last_cycle = True
            self.get_logger().info(f"{side} anchor reset")
            return

        pos_pub.publish(self._target_pos_cmd(arm, now))

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

    def _target_pos_cmd(self, arm: ArmState, now: float) -> PosCmd:
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

        raw_target_pose = copy_pose(arm.robot_anchor)
        raw_target_pose.position.x += self.position_scale * delta_p_base[0]
        raw_target_pose.position.y += self.position_scale * delta_p_base[1]
        raw_target_pose.position.z += self.position_scale * delta_p_base[2]

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
        set_pose_orientation(raw_target_pose, q_target)

        target_pose = self._smoothed_target_pose(arm, raw_target_pose, now)
        q_target = quat_from_pose(target_pose)

        roll, pitch, yaw = quat_to_euler(q_target)

        if self.debug_axis:
            self._debug_axis_throttled(arm.name, vr_delta_world, delta_p_base)

        msg = PosCmd()
        msg.x = target_pose.position.x * 1000.0
        msg.y = target_pose.position.y * 1000.0
        msg.z = target_pose.position.z * 1000.0
        msg.roll = math.degrees(roll)
        msg.pitch = math.degrees(pitch)
        msg.yaw = math.degrees(yaw)
        msg.gripper = self._gripper_position(arm)
        msg.mode1 = self.mode1
        msg.mode2 = self.move_mode
        return msg

    def _smoothed_target_pose(self, arm: ArmState, raw_target_pose: Pose, now: float) -> Pose:
        if not self.smooth_command:
            return raw_target_pose

        if arm.commanded_pose is None or arm.command_time is None:
            arm.commanded_pose = copy_pose(arm.robot_anchor or raw_target_pose)
            arm.command_time = now

        dt = max(1.0 / self.publish_rate_hz, now - arm.command_time)
        arm.command_time = now

        max_linear_step = self.max_linear_speed_mps * dt
        current = arm.commanded_pose
        delta = (
            raw_target_pose.position.x - current.position.x,
            raw_target_pose.position.y - current.position.y,
            raw_target_pose.position.z - current.position.z,
        )
        limited_delta = limit_vector(delta, max_linear_step)

        next_pose = copy_pose(current)
        next_pose.position.x += limited_delta[0]
        next_pose.position.y += limited_delta[1]
        next_pose.position.z += limited_delta[2]

        q_current = quat_from_pose(current)
        q_raw = quat_from_pose(raw_target_pose)
        q_delta = quat_multiply(q_raw, quat_inverse(q_current))
        max_angle_step = self.max_angular_speed_radps * dt
        angle = quat_angle(q_delta)
        if angle > max_angle_step and angle > 1e-9:
            q_delta = scale_quat_rotation(q_delta, max_angle_step / angle)
        q_next = quat_normalize(quat_multiply(q_delta, q_current))
        set_pose_orientation(next_pose, q_next)

        arm.commanded_pose = next_pose
        return next_pose

    def _gripper_position(self, arm: ArmState) -> float:
        if self.gripper_mode == "toggle":
            return 0.0 if arm.gripper_closed else 70.0
        if arm.joy is None:
            return 0.0
        if self.trigger_axis_index < 0 or self.trigger_axis_index >= len(arm.joy.axes):
            return 0.0
        close_amount = clamp(float(arm.joy.axes[self.trigger_axis_index]), 0.0, 1.0)
        return 70.0 * (1.0 - close_amount)

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

    def _debug_axis_throttled(self, side: str, vr_delta, delta_p_base, period: float = 0.5) -> None:
        now = time.monotonic()
        last = self.last_debug_time.get(side)
        if last is not None and now - last < period:
            return
        self.last_debug_time[side] = now
        self.get_logger().info(
            "%s anchor vr_delta=(%.3f, %.3f, %.3f) delta_p_base=(%.3f, %.3f, %.3f)"
            % (side, *vr_delta, *delta_p_base)
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


def quat_angle(q) -> float:
    x, y, z, w = quat_normalize(q)
    vector_norm = math.sqrt(x * x + y * y + z * z)
    return 2.0 * math.atan2(vector_norm, abs(w))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VrPiperAnchorTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
