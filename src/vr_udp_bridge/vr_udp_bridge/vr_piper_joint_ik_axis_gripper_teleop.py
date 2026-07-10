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
        self.declare_parameter("home_button_index", 2)
        self.declare_parameter("home_both_arms", True)
        self.declare_parameter("home_max_step_rad", 0.03)
        self.declare_parameter("home_joint_positions", "0,0,0,0,0,0")
        self.declare_parameter("left_home_joint_positions", "")
        self.declare_parameter("right_home_joint_positions", "")

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
        self.home_button_index = int(self.get_parameter("home_button_index").value)
        self.home_both_arms = parse_bool(self.get_parameter("home_both_arms").value)
        self.home_max_step_rad = float(self.get_parameter("home_max_step_rad").value)
        self.home_joint_positions = self._parse_home_joint_positions(
            self.get_parameter("home_joint_positions").value
        )
        self.left_home_joint_positions = self._home_positions_for_side("left")
        self.right_home_joint_positions = self._home_positions_for_side("right")

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
        if self.home_button_index < -1:
            raise ValueError("home_button_index must be -1 or greater")
        if self.home_max_step_rad <= 0.0:
            raise ValueError("home_max_step_rad must be greater than zero")
        if len(self.home_joint_positions) != 6:
            raise ValueError("home_joint_positions must contain exactly 6 values")
        if len(self.left_home_joint_positions) != 6:
            raise ValueError("left_home_joint_positions must contain exactly 6 values")
        if len(self.right_home_joint_positions) != 6:
            raise ValueError("right_home_joint_positions must contain exactly 6 values")

    def _parse_home_joint_positions(self, value):
        if isinstance(value, str):
            parts = [part.strip() for part in value.split(",") if part.strip()]
            if not parts:
                return []
            return [float(part) for part in parts]
        if isinstance(value, (list, tuple)):
            return [float(part) for part in value]
        raise ValueError("home_joint_positions must be a comma-separated string")

    def _home_positions_for_side(self, side: str):
        value = self.get_parameter(f"{side}_home_joint_positions").value
        positions = self._parse_home_joint_positions(value)
        return positions if positions else self.home_joint_positions

    def _on_joy(self, side: str, msg) -> None:
        super()._on_joy(side, msg)
        arm = self.arms[side]
        pressed = self._home_button_pressed(arm)
        arm.home_button_down = pressed
        if self.home_both_arms:
            home_active = any(
                bool(getattr(self.arms[arm_side], "home_button_down", False))
                for arm_side in ("left", "right")
            )
        else:
            home_active = pressed
        self._set_home_active(side, home_active)

    def _handle_arm(self, side: str, joint_pub, gripper_pub, now: float) -> None:
        arm = self.arms[side]

        if getattr(arm, "home_active", False):
            arm.vr_anchor = None
            arm.robot_anchor = None
            arm.enabled_last_cycle = False
            self._publish_home_step(arm, joint_pub)
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

    def _home_button_pressed(self, arm) -> bool:
        if self.home_button_index < 0:
            return False
        if arm.joy is None:
            return False
        if self.home_button_index >= len(arm.joy.buttons):
            self._warn_throttled(
                f"{arm.name}_bad_home_button",
                "home_button_index %d is outside %s Joy.buttons"
                % (self.home_button_index, arm.name),
            )
            return False
        return int(arm.joy.buttons[self.home_button_index]) == 1

    def _set_home_active(self, side: str, active: bool) -> None:
        targets = ("left", "right") if self.home_both_arms else (side,)
        for target in targets:
            arm = self.arms[target]
            was_active = getattr(arm, "home_active", False)
            arm.home_active = active
            if active:
                self._reset_teleop_state(target)
            elif was_active:
                self._reset_teleop_state(target)

        if active and not getattr(self.arms[side], "home_button_last", False):
            self.get_logger().warn(
                "%s controller home button pressed; moving %s Piper arm%s toward home"
                % (
                    side,
                    "both" if self.home_both_arms else side,
                    "s" if self.home_both_arms else "",
                )
            )
        elif not active and getattr(self.arms[side], "home_button_last", False):
            self.get_logger().warn(
                "%s controller home button released; stopping home motion" % side
            )
        self.arms[side].home_button_last = active

    def _publish_home_step(self, arm, joint_pub) -> None:
        if arm.current_joints is None:
            self._warn_throttled(
                f"{arm.name}_missing_home_joint_state",
                f"{arm.name} joint_states_single is missing; home command is not published",
            )
            return

        target = (
            self.left_home_joint_positions
            if arm.name == "left"
            else self.right_home_joint_positions
        )
        command = arm.current_joints.copy()
        for idx, value in enumerate(command):
            delta = target[idx] - float(value)
            command[idx] = float(value) + clamp(
                delta,
                -self.home_max_step_rad,
                self.home_max_step_rad,
            )

        arm.command_joints = command.copy()
        joint_pub.publish(self._joint_command_msg(command, arm.current_gripper))

    def _reset_teleop_state(self, side: str) -> None:
        arm = self.arms[side]
        arm.vr_anchor = None
        arm.robot_anchor = None
        arm.enabled_last_cycle = False
        arm.command_joints = None


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
