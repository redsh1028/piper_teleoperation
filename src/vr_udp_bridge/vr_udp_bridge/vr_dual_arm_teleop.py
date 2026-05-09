import json
import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
Matrix3 = Tuple[
    Tuple[float, float, float],
    Tuple[float, float, float],
    Tuple[float, float, float],
]


@dataclass
class ArmState:
    name: str
    vr_pose: Optional[PoseStamped] = None
    vr_pose_time: Optional[float] = None
    joy: Optional[Joy] = None
    joy_time: Optional[float] = None
    current_pose: Optional[PoseStamped] = None
    status: Optional[dict] = None
    status_time: Optional[float] = None
    vr_anchor: Optional[PoseStamped] = None
    robot_anchor: Optional[PoseStamped] = None
    enabled_last_cycle: bool = False


class VrDualArmTeleop(Node):
    def __init__(self) -> None:
        super().__init__("vr_dual_arm_teleop")

        self.declare_parameter("position_scale", 1.0)
        self.declare_parameter("publish_rate_hz", 60.0)
        self.declare_parameter("left_current_pose_topic", "/left_arm/current_pose")
        self.declare_parameter("right_current_pose_topic", "/right_arm/current_pose")
        self.declare_parameter("left_target_pose_topic", "/teleop/left_ee_target_pose")
        self.declare_parameter("right_target_pose_topic", "/teleop/right_ee_target_pose")
        self.declare_parameter("left_gripper_topic", "/teleop/left_gripper_command")
        self.declare_parameter("right_gripper_topic", "/teleop/right_gripper_command")
        self.declare_parameter("enable_button_index", 1)
        self.declare_parameter("trigger_axis_index", 2)
        self.declare_parameter("input_timeout_sec", 0.25)
        self.declare_parameter("target_frame_id", "piper_base")

        self.position_scale = float(self.get_parameter("position_scale").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.enable_button_index = int(self.get_parameter("enable_button_index").value)
        self.trigger_axis_index = int(self.get_parameter("trigger_axis_index").value)
        self.input_timeout_sec = float(self.get_parameter("input_timeout_sec").value)
        self.target_frame_id = str(self.get_parameter("target_frame_id").value)

        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")
        if self.input_timeout_sec <= 0.0:
            raise ValueError("input_timeout_sec must be greater than zero")

        self.arms: Dict[str, ArmState] = {
            "left": ArmState("left"),
            "right": ArmState("right"),
        }
        self.last_warn_time: Dict[str, float] = {}

        self.left_target_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("left_target_pose_topic").value),
            10,
        )
        self.right_target_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("right_target_pose_topic").value),
            10,
        )
        self.left_gripper_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("left_gripper_topic").value),
            10,
        )
        self.right_gripper_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("right_gripper_topic").value),
            10,
        )

        self.create_subscription(
            PoseStamped,
            "/vr/left_controller/pose",
            lambda msg: self._on_vr_pose("left", msg),
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/vr/right_controller/pose",
            lambda msg: self._on_vr_pose("right", msg),
            10,
        )
        self.create_subscription(
            Joy,
            "/vr/left_controller/joy",
            lambda msg: self._on_joy("left", msg),
            10,
        )
        self.create_subscription(
            Joy,
            "/vr/right_controller/joy",
            lambda msg: self._on_joy("right", msg),
            10,
        )
        self.create_subscription(
            String,
            "/vr/left_controller/status",
            lambda msg: self._on_status("left", msg),
            10,
        )
        self.create_subscription(
            String,
            "/vr/right_controller/status",
            lambda msg: self._on_status("right", msg),
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("left_current_pose_topic").value),
            lambda msg: self._on_current_pose("left", msg),
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("right_current_pose_topic").value),
            lambda msg: self._on_current_pose("right", msg),
            10,
        )

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)
        self.get_logger().info(
            "VR dual-arm teleop started with Grip deadman and %.2fx position scale"
            % self.position_scale
        )

    def _on_vr_pose(self, side: str, msg: PoseStamped) -> None:
        arm = self.arms[side]
        arm.vr_pose = msg
        arm.vr_pose_time = time.monotonic()

    def _on_joy(self, side: str, msg: Joy) -> None:
        arm = self.arms[side]
        arm.joy = msg
        arm.joy_time = time.monotonic()

    def _on_current_pose(self, side: str, msg: PoseStamped) -> None:
        self.arms[side].current_pose = msg

    def _on_status(self, side: str, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            self._warn_throttled(
                f"{side}_bad_status",
                f"Ignoring invalid {side} status JSON",
            )
            return

        if not isinstance(status, dict):
            self._warn_throttled(
                f"{side}_bad_status_shape",
                f"Ignoring invalid {side} status shape",
            )
            return

        arm = self.arms[side]
        arm.status = status
        arm.status_time = time.monotonic()

    def _on_timer(self) -> None:
        now = time.monotonic()
        self._handle_arm("left", self.left_target_pub, self.left_gripper_pub, now)
        self._handle_arm("right", self.right_target_pub, self.right_gripper_pub, now)

    def _handle_arm(self, side: str, target_pub, gripper_pub, now: float) -> None:
        arm = self.arms[side]

        if self._input_ready_for_gripper(arm, now):
            gripper_value = self._trigger_value(arm.joy)
            if gripper_value is not None:
                gripper_pub.publish(Float32(data=gripper_value))

        enabled = self._enabled(arm)
        input_ready = self._input_ready_for_motion(arm, now)
        if not enabled or not input_ready:
            arm.vr_anchor = None
            arm.robot_anchor = None
            arm.enabled_last_cycle = False
            return

        if (
            not arm.enabled_last_cycle
            or arm.vr_anchor is None
            or arm.robot_anchor is None
        ):
            if arm.current_pose is None:
                self._warn_throttled(
                    f"{side}_missing_current_pose",
                    f"{side} arm current pose is missing; teleop target is not published",
                )
                arm.enabled_last_cycle = False
                return

            arm.vr_anchor = arm.vr_pose
            arm.robot_anchor = arm.current_pose
            arm.enabled_last_cycle = True
            self.get_logger().info(f"{side} arm teleop recentered")
            return

        target = self._target_pose(arm)
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.target_frame_id
        target_pub.publish(target)

    def _input_ready_for_motion(self, arm: ArmState, now: float) -> bool:
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
        if arm.status is None or arm.status_time is None:
            self._warn_throttled(
                f"{arm.name}_missing_status",
                f"{arm.name} VR status is missing",
            )
            return False
        if now - arm.status_time > self.input_timeout_sec:
            self._warn_throttled(
                f"{arm.name}_stale_status",
                f"{arm.name} VR status timed out",
            )
            return False
        if arm.status.get("tracking_valid") is False:
            self._warn_throttled(
                f"{arm.name}_tracking_invalid",
                f"{arm.name} VR tracking is invalid",
            )
            return False
        if arm.joy is None or arm.joy_time is None:
            self._warn_throttled(
                f"{arm.name}_missing_joy",
                f"{arm.name} VR joy input is missing",
            )
            return False
        if now - arm.joy_time > self.input_timeout_sec:
            self._warn_throttled(
                f"{arm.name}_stale_joy",
                f"{arm.name} VR joy input timed out",
            )
            return False
        return True

    def _input_ready_for_gripper(self, arm: ArmState, now: float) -> bool:
        if arm.joy is None or arm.joy_time is None:
            return False
        if now - arm.joy_time > self.input_timeout_sec:
            return False
        if arm.status is not None and arm.status.get("tracking_valid") is False:
            return False
        return True

    def _enabled(self, arm: ArmState) -> bool:
        if arm.joy is None:
            return False
        if (
            self.enable_button_index < 0
            or self.enable_button_index >= len(arm.joy.buttons)
        ):
            self._warn_throttled(
                f"{arm.name}_bad_enable_button",
                "enable_button_index %d is outside %s Joy.buttons"
                % (self.enable_button_index, arm.name),
            )
            return False
        return int(arm.joy.buttons[self.enable_button_index]) == 1

    def _trigger_value(self, joy: Optional[Joy]) -> Optional[float]:
        if joy is None:
            return None
        if self.trigger_axis_index < 0 or self.trigger_axis_index >= len(joy.axes):
            return None
        return clamp(float(joy.axes[self.trigger_axis_index]), 0.0, 1.0)

    def _target_pose(self, arm: ArmState) -> PoseStamped:
        vr_anchor = arm.vr_anchor
        robot_anchor = arm.robot_anchor
        vr_pose = arm.vr_pose
        if vr_anchor is None or robot_anchor is None or vr_pose is None:
            raise RuntimeError("teleop anchors are not initialized")

        vr_delta = (
            vr_pose.pose.position.x - vr_anchor.pose.position.x,
            vr_pose.pose.position.y - vr_anchor.pose.position.y,
            vr_pose.pose.position.z - vr_anchor.pose.position.z,
        )
        robot_delta = vr_vector_to_robot(vr_delta)

        target = PoseStamped()
        target.pose.position.x = (
            robot_anchor.pose.position.x + self.position_scale * robot_delta[0]
        )
        target.pose.position.y = (
            robot_anchor.pose.position.y + self.position_scale * robot_delta[1]
        )
        target.pose.position.z = (
            robot_anchor.pose.position.z + self.position_scale * robot_delta[2]
        )

        q_vr_anchor = quat_from_pose(vr_anchor)
        q_vr_current = quat_from_pose(vr_pose)
        q_vr_delta = quat_multiply(q_vr_current, quat_inverse(q_vr_anchor))
        q_robot_delta = vr_rotation_to_robot(q_vr_delta)
        q_robot_anchor = quat_from_pose(robot_anchor)
        q_target = quat_normalize(quat_multiply(q_robot_delta, q_robot_anchor))

        target.pose.orientation.x = q_target[0]
        target.pose.orientation.y = q_target[1]
        target.pose.orientation.z = q_target[2]
        target.pose.orientation.w = q_target[3]
        return target

    def _warn_throttled(self, key: str, message: str, period: float = 1.0) -> None:
        now = time.monotonic()
        last = self.last_warn_time.get(key)
        if last is None or now - last >= period:
            self.last_warn_time[key] = now
            self.get_logger().warn(message)


def vr_vector_to_robot(vector: Vector3) -> Vector3:
    x, y, z = vector
    return (-z, -x, y)


def vr_rotation_to_robot(quaternion: Quaternion) -> Quaternion:
    transform: Matrix3 = (
        (0.0, 0.0, -1.0),
        (-1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
    )
    rotation = quat_to_matrix(quat_normalize(quaternion))
    mapped = matmul(matmul(transform, rotation), transpose(transform))
    return matrix_to_quat(mapped)


def quat_from_pose(msg: PoseStamped) -> Quaternion:
    q = msg.pose.orientation
    return quat_normalize((float(q.x), float(q.y), float(q.z), float(q.w)))


def quat_normalize(q: Quaternion) -> Quaternion:
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def quat_inverse(q: Quaternion) -> Quaternion:
    x, y, z, w = quat_normalize(q)
    return (-x, -y, -z, w)


def quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_to_matrix(q: Quaternion) -> Matrix3:
    x, y, z, w = quat_normalize(q)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def matrix_to_quat(m: Matrix3) -> Quaternion:
    m00, m01, m02 = m[0]
    m10, m11, m12 = m[1]
    m20, m21, m22 = m[2]

    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s

    return quat_normalize((qx, qy, qz, qw))


def matmul(a: Matrix3, b: Matrix3) -> Matrix3:
    return tuple(
        tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )


def transpose(m: Matrix3) -> Matrix3:
    return tuple(tuple(m[row][col] for row in range(3)) for col in range(3))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VrDualArmTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
