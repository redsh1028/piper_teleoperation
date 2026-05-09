import json
import math
import time
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from piper_msgs.msg import PosCmd
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

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
    quat_to_matrix,
    quat_multiply,
    quat_normalize,
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
    previous_vr_pose: Optional[PoseStamped] = None
    target_pose: Optional[Pose] = None
    controller_to_ee_rotation: Optional[tuple] = None
    enabled_last_cycle: bool = False


class VrPiperControllerAsEndEffector(Node):
    def __init__(self) -> None:
        super().__init__("vr_piper_controller_ee")

        self.declare_parameter("axis_map", "z,x,-y")
        self.declare_parameter("position_scale", 0.2)
        self.declare_parameter("orientation_scale", 1.0)
        self.declare_parameter("publish_rate_hz", 60.0)
        self.declare_parameter("input_timeout_sec", 0.25)
        self.declare_parameter("position_deadzone", 0.002)
        self.declare_parameter("max_target_offset_m", 0.35)
        self.declare_parameter("lock_orientation", False)
        self.declare_parameter("enable_button_index", 0)
        self.declare_parameter("trigger_axis_index", 2)
        self.declare_parameter("debug_axis", False)
        self.declare_parameter("move_mode", 0)
        self.declare_parameter("mode1", 1)

        self.axis_map = parse_axis_map(self.get_parameter("axis_map").value)
        self.position_scale = float(self.get_parameter("position_scale").value)
        self.orientation_scale = float(self.get_parameter("orientation_scale").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.input_timeout_sec = float(self.get_parameter("input_timeout_sec").value)
        self.position_deadzone = float(self.get_parameter("position_deadzone").value)
        self.max_target_offset_m = float(self.get_parameter("max_target_offset_m").value)
        self.lock_orientation = parse_bool(self.get_parameter("lock_orientation").value)
        self.enable_button_index = int(self.get_parameter("enable_button_index").value)
        self.trigger_axis_index = int(self.get_parameter("trigger_axis_index").value)
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

        self.arms: Dict[str, ArmState] = {
            "left": ArmState("left"),
            "right": ArmState("right"),
        }
        self.last_warn_time: Dict[str, float] = {}
        self.last_debug_time: Dict[str, float] = {}

        self.left_pos_pub = self.create_publisher(PosCmd, "/left/pos_cmd", 10)
        self.right_pos_pub = self.create_publisher(PosCmd, "/right/pos_cmd", 10)

        self._subscribe_arm("left")
        self._subscribe_arm("right")

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)
        self.get_logger().info(
            "VR controller-as-EE teleop started with axis_map=%s, position_scale=%.3f, orientation_scale=%.3f"
            % (
                axis_map_to_text(self.axis_map),
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

    def _on_vr_pose(self, side: str, msg: PoseStamped) -> None:
        arm = self.arms[side]
        arm.vr_pose = msg
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
        self._handle_arm("left", self.left_pos_pub, now)
        self._handle_arm("right", self.right_pos_pub, now)

    def _handle_arm(self, side: str, pos_pub, now: float) -> None:
        arm = self.arms[side]
        if not self._enabled(arm) or not self._input_ready(arm, now):
            arm.vr_anchor = None
            arm.previous_vr_pose = None
            arm.target_pose = None
            arm.controller_to_ee_rotation = None
            arm.enabled_last_cycle = False
            return

        if (
            not arm.enabled_last_cycle
            or arm.vr_anchor is None
            or arm.previous_vr_pose is None
            or arm.target_pose is None
            or arm.controller_to_ee_rotation is None
        ):
            if arm.robot_pose is None:
                self._warn_throttled(
                    f"{side}_missing_robot_pose",
                    f"{side} Piper end_pose is missing; not publishing PosCmd",
                )
                return
            arm.vr_anchor = arm.vr_pose
            arm.previous_vr_pose = arm.vr_pose
            arm.target_pose = copy_pose(arm.robot_pose)
            q_vr_anchor = map_quat(
                self.axis_map,
                quat_from_pose_stamped(arm.vr_anchor),
            )
            q_robot_anchor = quat_from_pose(arm.target_pose)
            # Button press calibration:
            # from this moment on, the VR controller local frame is treated as
            # the current robot end-effector(gripper_base) local frame.
            arm.controller_to_ee_rotation = quat_normalize(
                quat_multiply(q_robot_anchor, quat_inverse(q_vr_anchor))
            )
            arm.enabled_last_cycle = True
            self.get_logger().info(
                f"{side} controller frame calibrated to current EE frame"
            )
            return

        pos_pub.publish(self._target_pos_cmd(arm))

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
        if (
            arm.status is not None
            and arm.status_time is not None
            and now - arm.status_time > self.input_timeout_sec
        ):
            self._warn_throttled(
                f"{arm.name}_stale_status",
                f"{arm.name} VR status timed out",
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

    def _target_pos_cmd(self, arm: ArmState) -> PosCmd:
        if (
            arm.vr_anchor is None
            or arm.previous_vr_pose is None
            or arm.target_pose is None
            or arm.vr_pose is None
            or arm.controller_to_ee_rotation is None
        ):
            raise RuntimeError("teleop anchors are not initialized")

        vr_delta_world = (
            arm.vr_pose.pose.position.x - arm.previous_vr_pose.pose.position.x,
            arm.vr_pose.pose.position.y - arm.previous_vr_pose.pose.position.y,
            arm.vr_pose.pose.position.z - arm.previous_vr_pose.pose.position.z,
        )
        # First map raw VR world axes into the robot axis convention.
        vr_delta_robot_axes = mat_vec(self.axis_map, vr_delta_world)

        q_vr_current = map_quat(self.axis_map, quat_from_pose_stamped(arm.vr_pose))
        q_vr_previous = map_quat(self.axis_map, quat_from_pose_stamped(arm.previous_vr_pose))
        q_target_previous = quat_from_pose(arm.target_pose)

        # Then reinterpret that displacement through the button-time
        # controller->EE calibration, so movement is relative to EE axes.
        delta_p_robot = rotate_vector(arm.controller_to_ee_rotation, vr_delta_robot_axes)
        delta_p_robot = limit_vector(
            apply_deadzone(delta_p_robot, self.position_deadzone),
            self.max_target_offset_m,
        )
        arm.target_pose.position.x += self.position_scale * delta_p_robot[0]
        arm.target_pose.position.y += self.position_scale * delta_p_robot[1]
        arm.target_pose.position.z += self.position_scale * delta_p_robot[2]

        q_target = q_target_previous
        if not self.lock_orientation:
            q_delta_controller = quat_multiply(q_vr_current, quat_inverse(q_vr_previous))
            q_delta_robot = quat_multiply(
                quat_multiply(arm.controller_to_ee_rotation, q_delta_controller),
                quat_inverse(arm.controller_to_ee_rotation),
            )
            q_delta = q_delta_robot
            q_delta = scale_quat_rotation(q_delta, self.orientation_scale)
            q_target = quat_normalize(quat_multiply(q_delta, q_target_previous))
            set_pose_orientation(arm.target_pose, q_target)

        arm.previous_vr_pose = arm.vr_pose

        roll, pitch, yaw = quat_to_euler(q_target)

        if self.debug_axis:
            self._debug_axis_throttled(
                arm.name,
                vr_delta_world,
                delta_p_robot,
            )

        msg = PosCmd()
        msg.x = arm.target_pose.position.x * 1000.0
        msg.y = arm.target_pose.position.y * 1000.0
        msg.z = arm.target_pose.position.z * 1000.0
        msg.roll = math.degrees(roll)
        msg.pitch = math.degrees(pitch)
        msg.yaw = math.degrees(yaw)
        msg.gripper = self._gripper_position(arm.joy)
        msg.mode1 = self.mode1
        msg.mode2 = self.move_mode
        return msg

    def _gripper_position(self, joy: Optional[Joy]) -> float:
        if joy is None:
            return 0.0
        if self.trigger_axis_index < 0 or self.trigger_axis_index >= len(joy.axes):
            return 0.0
        close_amount = clamp(float(joy.axes[self.trigger_axis_index]), 0.0, 1.0)
        return 70.0 * (1.0 - close_amount)

    def _warn_throttled(self, key: str, message: str, period: float = 1.0) -> None:
        now = time.monotonic()
        last = self.last_warn_time.get(key)
        if last is None or now - last >= period:
            self.last_warn_time[key] = now
            self.get_logger().warn(message)

    def _debug_axis_throttled(
        self,
        side: str,
        vr_delta,
        delta_p_robot,
        period: float = 0.5,
    ) -> None:
        now = time.monotonic()
        last = self.last_debug_time.get(side)
        if last is not None and now - last < period:
            return
        self.last_debug_time[side] = now
        self.get_logger().info(
            "%s controller-as-EE vr_delta=(%.3f, %.3f, %.3f) delta_p_robot=(%.3f, %.3f, %.3f)"
            % (side, *vr_delta, *delta_p_robot)
        )


def rotate_vector(quaternion, vector):
    return mat_vec(quat_to_matrix(quat_normalize(quaternion)), vector)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VrPiperControllerAsEndEffector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
