import json
import math
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


Quaternion = Tuple[float, float, float, float]


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
JOINT_LIMITS = [
    (-2.618, 2.618),
    (0.0, 3.14),
    (-2.967, 0.0),
    (-1.745, 1.745),
    (-1.22, 1.22),
    (-2.0944, 2.0944),
]


class VrPiperGazeboJointTeleop(Node):
    """Simple VR-to-joint adapter for validating Piper Gazebo motion."""

    def __init__(self) -> None:
        super().__init__("vr_piper_gazebo_joint_teleop")

        self.declare_parameter("controller_side", "right")
        self.declare_parameter("arm_command_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("gripper_command_topic", "/gripper_controller/joint_trajectory")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("enable_button_index", 1)
        self.declare_parameter("trigger_axis_index", 2)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("input_timeout_sec", 0.25)
        self.declare_parameter("trajectory_duration_sec", 0.2)
        self.declare_parameter("position_scales", [1.4, 1.2, 1.2])
        self.declare_parameter("orientation_scales", [0.8, 0.8, 0.8])
        self.declare_parameter("fallback_home_joints", [0.0, 1.2, -1.4, 0.0, 0.0, 0.0])

        self.controller_side = str(self.get_parameter("controller_side").value)
        self.enable_button_index = int(self.get_parameter("enable_button_index").value)
        self.trigger_axis_index = int(self.get_parameter("trigger_axis_index").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.input_timeout_sec = float(self.get_parameter("input_timeout_sec").value)
        self.trajectory_duration_sec = float(
            self.get_parameter("trajectory_duration_sec").value
        )
        self.position_scales = _float_list(
            self.get_parameter("position_scales").value,
            [1.4, 1.2, 1.2],
            3,
        )
        self.orientation_scales = _float_list(
            self.get_parameter("orientation_scales").value,
            [0.8, 0.8, 0.8],
            3,
        )
        self.fallback_home_joints = _float_list(
            self.get_parameter("fallback_home_joints").value,
            [0.0, 1.2, -1.4, 0.0, 0.0, 0.0],
            6,
        )

        self.vr_pose: Optional[PoseStamped] = None
        self.vr_pose_time: Optional[float] = None
        self.joy: Optional[Joy] = None
        self.joy_time: Optional[float] = None
        self.status: Dict = {}
        self.status_time: Optional[float] = None
        self.latest_joints: Optional[List[float]] = None
        self.vr_anchor: Optional[PoseStamped] = None
        self.joint_anchor: Optional[List[float]] = None
        self.enabled_last_cycle = False
        self.last_warn_time: Dict[str, float] = {}

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("arm_command_topic").value),
            10,
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("gripper_command_topic").value),
            10,
        )

        prefix = f"/vr/{self.controller_side}_controller"
        self.create_subscription(PoseStamped, f"{prefix}/pose", self._on_vr_pose, 10)
        self.create_subscription(Joy, f"{prefix}/joy", self._on_joy, 10)
        self.create_subscription(String, f"{prefix}/status", self._on_status, 10)
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            10,
        )

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)
        self.get_logger().info(
            "VR Piper Gazebo joint teleop started from %s controller"
            % self.controller_side
        )

    def _on_vr_pose(self, msg: PoseStamped) -> None:
        self.vr_pose = msg
        self.vr_pose_time = time.monotonic()

    def _on_joy(self, msg: Joy) -> None:
        self.joy = msg
        self.joy_time = time.monotonic()

    def _on_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            self._warn_throttled("bad_status", "Ignoring invalid VR status JSON")
            return
        self.status = status if isinstance(status, dict) else {}
        self.status_time = time.monotonic()

    def _on_joint_state(self, msg: JointState) -> None:
        positions: Dict[str, float] = {}
        for name, position in zip(msg.name, msg.position):
            positions[name] = float(position)

        if all(name in positions for name in JOINT_NAMES):
            self.latest_joints = [positions[name] for name in JOINT_NAMES]

    def _on_timer(self) -> None:
        now = time.monotonic()
        self._publish_gripper_if_ready(now)

        if not self._enabled() or not self._input_ready(now):
            self.vr_anchor = None
            self.joint_anchor = None
            self.enabled_last_cycle = False
            return

        if not self.enabled_last_cycle or self.vr_anchor is None:
            self.vr_anchor = self.vr_pose
            self.joint_anchor = list(self.latest_joints or self.fallback_home_joints)
            self.enabled_last_cycle = True
            self.get_logger().info("Piper Gazebo joint teleop recentered")
            return

        self.arm_pub.publish(self._arm_trajectory())

    def _publish_gripper_if_ready(self, now: float) -> None:
        if self.joy is None or self.joy_time is None:
            return
        if now - self.joy_time > self.input_timeout_sec:
            return
        if self.trigger_axis_index < 0 or self.trigger_axis_index >= len(self.joy.axes):
            return

        close_amount = clamp(float(self.joy.axes[self.trigger_axis_index]), 0.0, 1.0)
        joint7 = 0.035 * (1.0 - close_amount)
        self.gripper_pub.publish(
            self._trajectory(["joint7"], [joint7], self.trajectory_duration_sec)
        )

    def _input_ready(self, now: float) -> bool:
        if self.vr_pose is None or self.vr_pose_time is None:
            self._warn_throttled("missing_pose", "VR pose is missing")
            return False
        if now - self.vr_pose_time > self.input_timeout_sec:
            self._warn_throttled("stale_pose", "VR pose timed out")
            return False
        if self.joy is None or self.joy_time is None:
            self._warn_throttled("missing_joy", "VR joy input is missing")
            return False
        if now - self.joy_time > self.input_timeout_sec:
            self._warn_throttled("stale_joy", "VR joy input timed out")
            return False
        if self.status.get("tracking_valid") is False:
            self._warn_throttled("invalid_tracking", "VR tracking is invalid")
            return False
        return True

    def _enabled(self) -> bool:
        if self.joy is None:
            return False
        if self.enable_button_index < 0 or self.enable_button_index >= len(self.joy.buttons):
            self._warn_throttled("bad_enable_button", "enable button index is invalid")
            return False
        return int(self.joy.buttons[self.enable_button_index]) == 1

    def _arm_trajectory(self) -> JointTrajectory:
        if self.vr_pose is None or self.vr_anchor is None or self.joint_anchor is None:
            raise RuntimeError("teleop anchors are not initialized")

        dx = self.vr_pose.pose.position.x - self.vr_anchor.pose.position.x
        dy = self.vr_pose.pose.position.y - self.vr_anchor.pose.position.y
        dz = self.vr_pose.pose.position.z - self.vr_anchor.pose.position.z

        q_delta = quat_multiply(
            quat_from_pose(self.vr_pose),
            quat_inverse(quat_from_pose(self.vr_anchor)),
        )
        roll, pitch, yaw = quat_to_euler(q_delta)

        target = list(self.joint_anchor)
        target[0] += -dx * self.position_scales[0]
        target[1] += dy * self.position_scales[1]
        target[2] += -dz * self.position_scales[2]
        target[3] += roll * self.orientation_scales[0]
        target[4] += pitch * self.orientation_scales[1]
        target[5] += yaw * self.orientation_scales[2]
        target = [
            clamp(position, limit[0], limit[1])
            for position, limit in zip(target, JOINT_LIMITS)
        ]

        return self._trajectory(JOINT_NAMES, target, self.trajectory_duration_sec)

    def _trajectory(
        self,
        joint_names: List[str],
        positions: List[float],
        duration_sec: float,
    ) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1.0) * 1e9),
        )
        trajectory.points.append(point)
        return trajectory

    def _warn_throttled(self, key: str, message: str, period: float = 1.0) -> None:
        now = time.monotonic()
        last = self.last_warn_time.get(key)
        if last is None or now - last >= period:
            self.last_warn_time[key] = now
            self.get_logger().warn(message)


def _float_list(value, fallback: List[float], length: int) -> List[float]:
    try:
        result = [float(item) for item in value]
    except TypeError:
        return fallback
    if len(result) != length:
        return fallback
    return result


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
    return quat_normalize(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


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


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VrPiperGazeboJointTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
