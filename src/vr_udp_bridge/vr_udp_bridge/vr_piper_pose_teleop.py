import json
import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from piper_msgs.msg import PosCmd
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


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
    status: Optional[dict] = None
    status_time: Optional[float] = None
    robot_pose: Optional[Pose] = None
    previous_vr_pose: Optional[PoseStamped] = None
    target_pose: Optional[Pose] = None
    enabled_last_cycle: bool = False


class VrPiperPoseTeleop(Node):
    def __init__(self) -> None:
        super().__init__("vr_piper_pose_teleop")

        self.declare_parameter("axis_map", "z,x,-y")
        self.declare_parameter("position_scale", 0.2)
        self.declare_parameter("publish_rate_hz", 60.0)
        self.declare_parameter("input_timeout_sec", 0.25)
        self.declare_parameter("position_deadzone", 0.002)
        self.declare_parameter("max_step_m", 0.03)
        self.declare_parameter("lock_orientation", False)
        self.declare_parameter("orientation_scale", 1.0)
        self.declare_parameter("enable_button_index", 1)
        self.declare_parameter("trigger_axis_index", 2)
        self.declare_parameter("debug_axis", False)
        self.declare_parameter("move_mode", 0)
        self.declare_parameter("mode1", 1)

        self.axis_map = parse_axis_map(self.get_parameter("axis_map").value)
        self.position_scale = float(self.get_parameter("position_scale").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.input_timeout_sec = float(self.get_parameter("input_timeout_sec").value)
        self.position_deadzone = float(self.get_parameter("position_deadzone").value)
        self.max_step_m = float(self.get_parameter("max_step_m").value)
        self.lock_orientation = parse_bool(self.get_parameter("lock_orientation").value)
        self.orientation_scale = float(self.get_parameter("orientation_scale").value)
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
        if self.max_step_m <= 0.0:
            raise ValueError("max_step_m must be greater than zero")

        self.arms: Dict[str, ArmState] = {
            "left": ArmState("left"),
            "right": ArmState("right"),
        }
        self.last_warn_time: Dict[str, float] = {}
        self.last_debug_time: Dict[str, float] = {}

        self.left_pos_pub = self.create_publisher(PosCmd, "/left/pos_cmd", 10)
        self.right_pos_pub = self.create_publisher(PosCmd, "/right/pos_cmd", 10)

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
            Pose,
            "/left/end_pose",
            lambda msg: self._on_robot_pose("left", msg),
            10,
        )
        self.create_subscription(
            Pose,
            "/right/end_pose",
            lambda msg: self._on_robot_pose("right", msg),
            10,
        )

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)
        self.get_logger().info(
            "VR Piper pose teleop started with axis_map=%s, position_scale=%.3f"
            % (axis_map_to_text(self.axis_map), self.position_scale)
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
            arm.previous_vr_pose = None
            arm.target_pose = None
            arm.enabled_last_cycle = False
            return

        if not arm.enabled_last_cycle or arm.previous_vr_pose is None or arm.target_pose is None:
            if arm.robot_pose is None:
                self._warn_throttled(
                    f"{side}_missing_robot_pose",
                    f"{side} Piper end_pose is missing; not publishing PosCmd",
                )
                return
            arm.previous_vr_pose = arm.vr_pose
            arm.target_pose = copy_pose(arm.robot_pose)
            arm.enabled_last_cycle = True
            self.get_logger().info(f"{side} Piper incremental teleop started")
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
        if arm.previous_vr_pose is None or arm.target_pose is None or arm.vr_pose is None:
            raise RuntimeError("teleop state is not initialized")

        vr_delta = (
            arm.vr_pose.pose.position.x - arm.previous_vr_pose.pose.position.x,
            arm.vr_pose.pose.position.y - arm.previous_vr_pose.pose.position.y,
            arm.vr_pose.pose.position.z - arm.previous_vr_pose.pose.position.z,
        )
        previous_vr_pose = arm.previous_vr_pose
        robot_delta = limit_vector(
            apply_deadzone(mat_vec(self.axis_map, vr_delta), self.position_deadzone),
            self.max_step_m,
        )

        arm.target_pose.position.x += self.position_scale * robot_delta[0]
        arm.target_pose.position.y += self.position_scale * robot_delta[1]
        arm.target_pose.position.z += self.position_scale * robot_delta[2]

        if not self.lock_orientation:
            q_vr_delta = quat_multiply(
                quat_from_pose_stamped(arm.vr_pose),
                quat_inverse(quat_from_pose_stamped(previous_vr_pose)),
            )
            q_robot_delta = scale_quat_rotation(
                map_quat(self.axis_map, q_vr_delta),
                self.orientation_scale,
            )
            q_target = quat_normalize(
                quat_multiply(q_robot_delta, quat_from_pose(arm.target_pose))
            )
            set_pose_orientation(arm.target_pose, q_target)

        arm.previous_vr_pose = arm.vr_pose

        q_target = quat_from_pose(arm.target_pose)
        roll, pitch, yaw = quat_to_euler(q_target)

        if self.debug_axis:
            self._debug_axis_throttled(arm.name, vr_delta, robot_delta)

        msg = PosCmd()
        # Piper PosCmd examples use mm for position/gripper and degrees for RPY.
        # piper_single_ctrl_node converts these to the SDK's 0.001 mm/deg units.
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
        self, side: str, vr_delta: Vector3, robot_delta: Vector3, period: float = 0.5
    ) -> None:
        now = time.monotonic()
        last = self.last_debug_time.get(side)
        if last is not None and now - last < period:
            return
        self.last_debug_time[side] = now
        self.get_logger().info(
            "%s axis check vr_delta=(%.3f, %.3f, %.3f) robot_delta=(%.3f, %.3f, %.3f)"
            % (side, *vr_delta, *robot_delta)
        )


def parse_axis_map(value) -> Matrix3:
    if isinstance(value, str):
        text = value.strip()
        if text.startswith("[") and text.endswith("]"):
            text = text[1:-1]
        terms = [item.strip().strip("'\"") for item in text.split(",") if item.strip()]
    elif isinstance(value, Sequence):
        terms = [str(item).strip().strip("'\"") for item in value]
    else:
        raise ValueError("axis_map must be a string or list")

    if len(terms) != 3:
        raise ValueError("axis_map must contain three axes, e.g. '-z,-x,y'")

    rows = tuple(axis_term_to_row(term) for term in terms)
    if sorted(row.index(next(v for v in row if v != 0.0)) for row in rows) != [0, 1, 2]:
        raise ValueError("axis_map must use x, y, z exactly once")
    return rows  # type: ignore[return-value]


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def axis_term_to_row(term: str) -> Tuple[float, float, float]:
    sign = 1.0
    axis = term.lower()
    if axis.startswith("+"):
        axis = axis[1:]
    elif axis.startswith("-"):
        axis = axis[1:]
        sign = -1.0

    if axis not in ("x", "y", "z"):
        raise ValueError("axis_map entries must be x, y, z with optional sign")

    row = [0.0, 0.0, 0.0]
    row[{"x": 0, "y": 1, "z": 2}[axis]] = sign
    return (row[0], row[1], row[2])


def axis_map_to_text(axis_map: Matrix3) -> str:
    names = ("x", "y", "z")
    terms = []
    for row in axis_map:
        for idx, value in enumerate(row):
            if value != 0.0:
                terms.append(("" if value > 0 else "-") + names[idx])
                break
    return ",".join(terms)


def mat_vec(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3))  # type: ignore[return-value]


def apply_deadzone(vector: Vector3, deadzone: float) -> Vector3:
    if vector_norm(vector) < deadzone:
        return (0.0, 0.0, 0.0)
    return vector


def limit_vector(vector: Vector3, max_norm: float) -> Vector3:
    norm = vector_norm(vector)
    if norm <= max_norm or norm <= 0.0:
        return vector
    scale = max_norm / norm
    return (vector[0] * scale, vector[1] * scale, vector[2] * scale)


def vector_norm(vector: Vector3) -> float:
    return math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)


def copy_pose(msg: Pose) -> Pose:
    pose = Pose()
    pose.position.x = msg.position.x
    pose.position.y = msg.position.y
    pose.position.z = msg.position.z
    pose.orientation.x = msg.orientation.x
    pose.orientation.y = msg.orientation.y
    pose.orientation.z = msg.orientation.z
    pose.orientation.w = msg.orientation.w
    return pose


def set_pose_orientation(msg: Pose, quaternion: Quaternion) -> None:
    qx, qy, qz, qw = quat_normalize(quaternion)
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw


def map_quat(axis_map: Matrix3, quaternion: Quaternion) -> Quaternion:
    rotation = quat_to_matrix(quat_normalize(quaternion))
    mapped = matmul(matmul(axis_map, rotation), transpose(axis_map))
    return matrix_to_quat(mapped)


def scale_quat_rotation(q: Quaternion, scale: float) -> Quaternion:
    if scale == 1.0:
        return quat_normalize(q)
    x, y, z, w = quat_normalize(q)
    angle = 2.0 * math.atan2(math.sqrt(x * x + y * y + z * z), w)
    if angle <= 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    axis_norm = math.sqrt(x * x + y * y + z * z)
    axis = (x / axis_norm, y / axis_norm, z / axis_norm)
    scaled_half = 0.5 * angle * scale
    sin_half = math.sin(scaled_half)
    return quat_normalize(
        (
            axis[0] * sin_half,
            axis[1] * sin_half,
            axis[2] * sin_half,
            math.cos(scaled_half),
        )
    )


def quat_from_pose_stamped(msg: PoseStamped) -> Quaternion:
    return quat_from_pose(msg.pose)


def quat_from_pose(msg: Pose) -> Quaternion:
    q = msg.orientation
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


def quat_to_euler(q: Quaternion) -> Tuple[float, float, float]:
    x, y, z, w = quat_normalize(q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def matmul(a: Matrix3, b: Matrix3) -> Matrix3:
    return tuple(
        tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def transpose(m: Matrix3) -> Matrix3:
    return tuple(tuple(m[row][col] for row in range(3)) for col in range(3))  # type: ignore[return-value]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VrPiperPoseTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
