import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException

from vr_udp_bridge.vr_piper_joint_ik_teleop import (
    VrPiperJointIkTeleop,
    clamp,
    copy_pose,
    parse_bool,
)


class VrPiperJointIkAxisGripperTeleop(VrPiperJointIkTeleop):
    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter("gripper_axis_index", 1)
        self.declare_parameter("gripper_axis_input_min", -1.0)
        self.declare_parameter("gripper_axis_input_max", 1.0)
        self.declare_parameter("gripper_open_position_m", 0.10)
        self.declare_parameter("gripper_closed_position_m", 0.0)
        self.declare_parameter("gripper_axis_deadzone", 0.02)
        self.declare_parameter("gripper_max_step_m", 0.01)
        self.declare_parameter("gripper_smoothing_alpha", 0.35)
        self.declare_parameter("gripper_full_open_button_index", 1)
        self.declare_parameter("gripper_effort", 2.0)
        self.declare_parameter("estop_button_index", 2)
        self.declare_parameter("estop_hold_both_arms", True)

        self.gripper_axis_index = int(
            self.get_parameter("gripper_axis_index").value
        )
        self.gripper_axis_input_min = float(
            self.get_parameter("gripper_axis_input_min").value
        )
        self.gripper_axis_input_max = float(
            self.get_parameter("gripper_axis_input_max").value
        )
        self.gripper_open_position_m = float(
            self.get_parameter("gripper_open_position_m").value
        )
        self.gripper_closed_position_m = float(
            self.get_parameter("gripper_closed_position_m").value
        )
        self.gripper_axis_deadzone = float(
            self.get_parameter("gripper_axis_deadzone").value
        )
        self.gripper_max_step_m = float(
            self.get_parameter("gripper_max_step_m").value
        )
        self.gripper_smoothing_alpha = float(
            self.get_parameter("gripper_smoothing_alpha").value
        )
        self.gripper_full_open_button_index = int(
            self.get_parameter("gripper_full_open_button_index").value
        )
        self.gripper_effort = float(self.get_parameter("gripper_effort").value)
        self.estop_button_index = int(self.get_parameter("estop_button_index").value)
        self.estop_hold_both_arms = parse_bool(
            self.get_parameter("estop_hold_both_arms").value
        )
        self.estop_active = False

        self._validate_axis_gripper_parameters()
        self.get_logger().info(
            "Analog gripper enabled with Joy.axes[%d], input=[%.3f, %.3f], joint7=[%.3f, %.3f]"
            % (
                self.gripper_axis_index,
                self.gripper_axis_input_min,
                self.gripper_axis_input_max,
                self.gripper_closed_position_m,
                self.gripper_open_position_m,
            )
        )

    def _validate_axis_gripper_parameters(self) -> None:
        if self.gripper_axis_index < 0:
            raise ValueError("gripper_axis_index must be zero or greater")
        if self.gripper_axis_input_min == self.gripper_axis_input_max:
            raise ValueError(
                "gripper_axis_input_min and gripper_axis_input_max must be different"
            )
        if self.gripper_axis_deadzone < 0.0:
            raise ValueError("gripper_axis_deadzone must be zero or greater")
        if self.gripper_max_step_m <= 0.0:
            raise ValueError("gripper_max_step_m must be greater than zero")
        if not 0.0 <= self.gripper_smoothing_alpha <= 1.0:
            raise ValueError("gripper_smoothing_alpha must be between 0 and 1")
        if self.gripper_full_open_button_index < -1:
            raise ValueError("gripper_full_open_button_index must be -1 or greater")
        if self.gripper_effort <= 0.0:
            raise ValueError("gripper_effort must be greater than zero")
        if self.estop_button_index < -1:
            raise ValueError("estop_button_index must be -1 or greater")

    def _on_joy(self, side: str, msg) -> None:
        super()._on_joy(side, msg)
        arm = self.arms[side]
        if self._estop_button_pressed(arm):
            self._trigger_estop(side)

    def _handle_arm(self, side: str, joint_pub, gripper_pub, now: float) -> None:
        arm = self.arms[side]

        if self.estop_active:
            arm.vr_anchor = None
            arm.robot_anchor = None
            arm.enabled_last_cycle = False
            self._publish_estop_hold(arm, joint_pub)
            return

        if not self._enabled(arm) or not self._input_ready(arm, now):
            arm.vr_anchor = None
            arm.robot_anchor = None
            arm.enabled_last_cycle = False
            self._publish_axis_gripper_only(arm, joint_pub, now)
            return

        if (
            not arm.enabled_last_cycle
            or arm.vr_anchor is None
            or arm.robot_anchor is None
        ):
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

        super()._handle_arm(side, joint_pub, gripper_pub, now)

    def _handle_gripper_toggle(self, arm, gripper_pub, now: float) -> None:
        return

    def _joint_command_msg(self, joints, gripper: float):
        msg = super()._joint_command_msg(joints, gripper)
        msg.effort[6] = self.gripper_effort
        return msg

    def _publish_axis_gripper_only(self, arm, joint_pub, now: float) -> None:
        if arm.joy is None or arm.joy_time is None:
            return
        if now - arm.joy_time > self.input_timeout_sec:
            return

        if arm.current_joints is None:
            self._warn_throttled(
                f"{arm.name}_missing_gripper_joint_state",
                f"{arm.name} joint_states_single is missing; gripper axis command is not published",
            )
            return

        joints = arm.current_joints.copy()
        joint_pub.publish(self._joint_command_msg(joints, self._gripper_position(arm)))

    def _publish_estop_hold(self, arm, joint_pub) -> None:
        hold_joints = getattr(arm, "estop_hold_joints", None)
        if hold_joints is None:
            return
        hold_gripper = float(
            getattr(
                arm,
                "estop_hold_gripper",
                getattr(arm, "command_gripper", arm.current_gripper),
            )
        )
        joint_pub.publish(self._joint_command_msg(hold_joints, hold_gripper))

    def _gripper_position(self, arm) -> float:
        previous = float(getattr(arm, "command_gripper", arm.current_gripper))

        if arm.joy is None or arm.joy_time is None:
            return previous
        if time.monotonic() - arm.joy_time > self.input_timeout_sec:
            return previous
        if self._full_open_button_pressed(arm):
            arm.command_gripper = self.gripper_open_position_m
            return self.gripper_open_position_m
        if self.gripper_axis_index >= len(arm.joy.axes):
            self._warn_throttled(
                f"{arm.name}_bad_gripper_axis",
                "gripper_axis_index %d is outside %s Joy.axes"
                % (self.gripper_axis_index, arm.name),
            )
            return previous

        raw = float(arm.joy.axes[self.gripper_axis_index])
        if abs(raw) <= self.gripper_axis_deadzone:
            command = previous
        else:
            max_abs_input = max(
                abs(self.gripper_axis_input_min),
                abs(self.gripper_axis_input_max),
            )
            axis_amount = clamp(raw / max_abs_input, -1.0, 1.0)
            target = previous + axis_amount * self.gripper_max_step_m
            smoothed = previous + self.gripper_smoothing_alpha * (target - previous)
            step = clamp(
                smoothed - previous,
                -self.gripper_max_step_m,
                self.gripper_max_step_m,
            )
            command = previous + step

        lower = min(self.gripper_open_position_m, self.gripper_closed_position_m)
        upper = max(self.gripper_open_position_m, self.gripper_closed_position_m)
        command = clamp(command, lower, upper)
        arm.command_gripper = command
        return command

    def _full_open_button_pressed(self, arm) -> bool:
        if self.gripper_full_open_button_index < 0:
            return False
        if self.gripper_full_open_button_index >= len(arm.joy.buttons):
            self._warn_throttled(
                f"{arm.name}_bad_gripper_full_open_button",
                "gripper_full_open_button_index %d is outside %s Joy.buttons"
                % (self.gripper_full_open_button_index, arm.name),
            )
            return False
        return int(arm.joy.buttons[self.gripper_full_open_button_index]) == 1

    def _estop_button_pressed(self, arm) -> bool:
        if self.estop_button_index < 0:
            return False
        if arm.joy is None:
            return False
        if self.estop_button_index >= len(arm.joy.buttons):
            self._warn_throttled(
                f"{arm.name}_bad_estop_button",
                "estop_button_index %d is outside %s Joy.buttons"
                % (self.estop_button_index, arm.name),
            )
            return False
        return int(arm.joy.buttons[self.estop_button_index]) == 1

    def _trigger_estop(self, side: str) -> None:
        if self.estop_active:
            return
        self.estop_active = True
        hold_sides = ("left", "right") if self.estop_hold_both_arms else (side,)
        missing = []
        for hold_side in hold_sides:
            arm = self.arms[hold_side]
            if arm.current_joints is None:
                missing.append(hold_side)
                continue
            arm.estop_hold_joints = arm.current_joints.copy()
            arm.estop_hold_gripper = float(
                getattr(arm, "command_gripper", arm.current_gripper)
            )
            arm.vr_anchor = None
            arm.robot_anchor = None
            arm.enabled_last_cycle = False
        if missing:
            self.get_logger().error(
                "%s controller e-stop pressed; missing joint feedback for %s"
                % (side, ", ".join(missing))
            )
        self.get_logger().error(
            "%s controller e-stop pressed; holding %s Piper arm%s in place"
            % (
                side,
                "both" if self.estop_hold_both_arms else side,
                "s" if self.estop_hold_both_arms else "",
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VrPiperJointIkAxisGripperTeleop()
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
