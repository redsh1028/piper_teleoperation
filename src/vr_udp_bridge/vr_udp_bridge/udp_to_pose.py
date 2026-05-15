import json
import socket
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class VrUdpBridge(Node):
    def __init__(self):
        super().__init__("vr_udp_bridge")

        self.left_pose_pub = self.create_publisher(
            PoseStamped,
            "/vr/left_controller/pose",
            10,
        )
        self.right_pose_pub = self.create_publisher(
            PoseStamped,
            "/vr/right_controller/pose",
            10,
        )
        self.left_joy_pub = self.create_publisher(
            Joy,
            "/vr/left_controller/joy",
            10,
        )
        self.right_joy_pub = self.create_publisher(
            Joy,
            "/vr/right_controller/joy",
            10,
        )
        self.left_twist_pub = self.create_publisher(
            TwistStamped,
            "/vr/left_controller/twist",
            10,
        )
        self.right_twist_pub = self.create_publisher(
            TwistStamped,
            "/vr/right_controller/twist",
            10,
        )
        self.left_status_pub = self.create_publisher(
            String,
            "/vr/left_controller/status",
            10,
        )
        self.right_status_pub = self.create_publisher(
            String,
            "/vr/right_controller/status",
            10,
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 5005))
        self.sock.setblocking(False)

        self.timer = self.create_timer(1.0 / 120.0, self.timer_callback)

        self.get_logger().info("VR UDP bridge started on port 5005")

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()

    def timer_callback(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(8192)
            except BlockingIOError:
                return

            try:
                packets = json.loads(data.decode("utf-8"))
            except json.JSONDecodeError:
                self.get_logger().warn("Invalid JSON packet")
                continue

            if not isinstance(packets, list):
                self.get_logger().warn("Invalid packet shape")
                continue

            stamp = self.get_clock().now().to_msg()
            for packet in packets:
                if not isinstance(packet, dict):
                    continue

                name = packet.get("name", "")
                pose_pub = self._pose_publisher_for_name(name)
                joy_pub = self._joy_publisher_for_name(name)
                twist_pub = self._twist_publisher_for_name(name)
                status_pub = self._status_publisher_for_name(name)

                pose_msg = self._pose_msg(packet, stamp)
                if pose_msg is not None and pose_pub is not None:
                    pose_pub.publish(pose_msg)

                joy_msg = self._joy_msg(packet, stamp)
                if joy_msg is not None and joy_pub is not None:
                    joy_pub.publish(joy_msg)

                twist_msg = self._twist_msg(packet, stamp)
                if twist_msg is not None and twist_pub is not None:
                    twist_pub.publish(twist_msg)

                status_msg = self._status_msg(packet)
                if status_msg is not None and status_pub is not None:
                    status_pub.publish(status_msg)

    def _pose_publisher_for_name(self, name):
        if name == "left_controller":
            return self.left_pose_pub
        if name == "right_controller":
            return self.right_pose_pub
        return None

    def _joy_publisher_for_name(self, name):
        if name == "left_controller":
            return self.left_joy_pub
        if name == "right_controller":
            return self.right_joy_pub
        return None

    def _twist_publisher_for_name(self, name):
        if name == "left_controller":
            return self.left_twist_pub
        if name == "right_controller":
            return self.right_twist_pub
        return None

    def _status_publisher_for_name(self, name):
        if name == "left_controller":
            return self.left_status_pub
        if name == "right_controller":
            return self.right_status_pub
        return None

    def _pose_msg(self, packet, stamp) -> Optional[PoseStamped]:
        try:
            pos = packet["position"]
            quat = packet["orientation"]

            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = "vr_world"
            msg.pose.position.x = float(pos[0])
            msg.pose.position.y = float(pos[1])
            msg.pose.position.z = float(pos[2])
            msg.pose.orientation.x = float(quat[0])
            msg.pose.orientation.y = float(quat[1])
            msg.pose.orientation.z = float(quat[2])
            msg.pose.orientation.w = float(quat[3])
            return msg
        except (KeyError, IndexError, TypeError, ValueError):
            self.get_logger().warn("Invalid pose packet")
            return None

    def _joy_msg(self, packet, stamp) -> Optional[Joy]:
        axes = packet.get("axes")
        buttons = packet.get("buttons")

        if axes is None and any(
            key in packet
            for key in (
                "thumb_stick_horizontal",
                "thumb_stick_vertical",
                "press_index",
                "press_middle",
            )
        ):
            axes = [
                packet.get("thumb_stick_horizontal", 0.0),
                packet.get("thumb_stick_vertical", 0.0),
                packet.get("press_index", 0.0),
                packet.get("press_middle", 0.0),
            ]

        if buttons is None and any(
            key in packet for key in ("button_lower", "button_upper")
        ):
            buttons = [
                int(bool(packet.get("button_lower", False))),
                int(bool(packet.get("button_upper", False))),
            ]

        if axes is None and buttons is None:
            return None

        try:
            msg = Joy()
            msg.header.stamp = stamp
            msg.header.frame_id = "vr_world"
            msg.axes = [float(value) for value in axes] if axes is not None else []
            msg.buttons = (
                [int(value) for value in buttons] if buttons is not None else []
            )
            return msg
        except (TypeError, ValueError):
            self.get_logger().warn("Invalid joy packet")
            return None

    def _twist_msg(self, packet, stamp) -> Optional[TwistStamped]:
        velocity = packet.get("velocity")
        angular_velocity = packet.get("angular_velocity")
        if velocity is None and angular_velocity is None:
            return None

        try:
            msg = TwistStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = "vr_world"

            if velocity is not None:
                msg.twist.linear.x = float(velocity[0])
                msg.twist.linear.y = float(velocity[1])
                msg.twist.linear.z = float(velocity[2])

            if angular_velocity is not None:
                msg.twist.angular.x = float(angular_velocity[0])
                msg.twist.angular.y = float(angular_velocity[1])
                msg.twist.angular.z = float(angular_velocity[2])

            return msg
        except (IndexError, TypeError, ValueError):
            self.get_logger().warn("Invalid twist packet")
            return None

    def _status_msg(self, packet) -> Optional[String]:
        status_keys = [
            "name",
            "role",
            "device_index",
            "tracking_valid",
            "tracking_result",
            "button_pressed_mask",
            "button_touched_mask",
            "input_packet_num",
            "timestamp",
        ]
        status = {key: packet[key] for key in status_keys if key in packet}
        if not status:
            return None

        msg = String()
        msg.data = json.dumps(status, separators=(",", ":"))
        return msg


def main():
    rclpy.init()
    node = VrUdpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
