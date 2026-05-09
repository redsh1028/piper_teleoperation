import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


try:
    from quest2ros.msg import OVR2ROSInputs
except ImportError:
    OVR2ROSInputs = None


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class Quest2RosAdapter(Node):
    def __init__(self) -> None:
        super().__init__("quest2ros_adapter")

        self.declare_parameter("mirror", False)
        self.declare_parameter("input_timeout_sec", 2.0)

        self.mirror = parse_bool(self.get_parameter("mirror").value)
        self.input_timeout_sec = float(self.get_parameter("input_timeout_sec").value)
        self.last_msg_time = {}

        if OVR2ROSInputs is None:
            self.get_logger().error(
                "quest2ros.msg.OVR2ROSInputs is not available. "
                "Build/source the quest2ros message package first."
            )
            raise RuntimeError("quest2ros message package is missing")

        self.pose_pubs = {
            "left": self.create_publisher(PoseStamped, "/vr/left_controller/pose", 10),
            "right": self.create_publisher(PoseStamped, "/vr/right_controller/pose", 10),
        }
        self.joy_pubs = {
            "left": self.create_publisher(Joy, "/vr/left_controller/joy", 10),
            "right": self.create_publisher(Joy, "/vr/right_controller/joy", 10),
        }
        self.twist_pubs = {
            "left": self.create_publisher(TwistStamped, "/vr/left_controller/twist", 10),
            "right": self.create_publisher(TwistStamped, "/vr/right_controller/twist", 10),
        }
        self.status_pubs = {
            "left": self.create_publisher(String, "/vr/left_controller/status", 10),
            "right": self.create_publisher(String, "/vr/right_controller/status", 10),
        }

        for side in ("left", "right"):
            quest_side = self._quest_side(side)
            self.create_subscription(
                PoseStamped,
                f"/q2r_{quest_side}_hand_pose",
                lambda msg, arm_side=side: self._on_pose(arm_side, msg),
                10,
            )
            self.create_subscription(
                OVR2ROSInputs,
                f"/q2r_{quest_side}_hand_inputs",
                lambda msg, arm_side=side: self._on_inputs(arm_side, msg),
                10,
            )
            self.create_subscription(
                Twist,
                f"/q2r_{quest_side}_hand_twist",
                lambda msg, arm_side=side: self._on_twist(arm_side, msg),
                10,
            )

        self.timer = self.create_timer(1.0, self._on_timer)
        self.get_logger().info(
            "Quest2ROS adapter started. Joy layout: "
            "buttons[0]=lower/move, buttons[1]=upper/gripper, "
            "axes[0]=stick_x, axes[1]=stick_y, axes[2]=press_index, axes[3]=press_middle"
        )

    def _quest_side(self, side: str) -> str:
        if not self.mirror:
            return side
        return "right" if side == "left" else "left"

    def _on_pose(self, side: str, msg: PoseStamped) -> None:
        msg.header.frame_id = msg.header.frame_id or "quest_world"
        self.pose_pubs[side].publish(msg)
        self._publish_status(side, tracking_valid=True)

    def _on_inputs(self, side: str, msg) -> None:
        stamp = self.get_clock().now().to_msg()
        joy = Joy()
        joy.header.stamp = stamp
        joy.header.frame_id = "quest_world"
        joy.axes = [
            float(msg.thumb_stick_horizontal),
            float(msg.thumb_stick_vertical),
            float(msg.press_index),
            float(msg.press_middle),
        ]
        joy.buttons = [
            int(bool(msg.button_lower)),
            int(bool(msg.button_upper)),
        ]
        self.joy_pubs[side].publish(joy)
        self._publish_status(side, tracking_valid=True)

    def _on_twist(self, side: str, msg: Twist) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = "quest_world"
        stamped.twist = msg
        self.twist_pubs[side].publish(stamped)
        self._publish_status(side, tracking_valid=True)

    def _publish_status(self, side: str, tracking_valid: bool) -> None:
        now = time.monotonic()
        self.last_msg_time[side] = now
        status = {
            "name": f"{side}_controller",
            "source": "quest2ros",
            "tracking_valid": tracking_valid,
            "timestamp": now,
        }
        msg = String()
        msg.data = json.dumps(status, separators=(",", ":"))
        self.status_pubs[side].publish(msg)

    def _on_timer(self) -> None:
        now = time.monotonic()
        for side in ("left", "right"):
            last = self.last_msg_time.get(side)
            if last is not None and now - last > self.input_timeout_sec:
                self._publish_status(side, tracking_valid=False)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Quest2RosAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
