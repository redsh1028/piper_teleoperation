#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from piper_msgs.msg import PosCmd


class WegoPublisher(Node):
    def __init__(self, node_name, topic, msg_type, timer_callback, timer_period=1.0):
        super().__init__(node_name)
        self.pub = self.create_publisher(msg_type, topic, 1)
        self.msg = msg_type()
        self.timer = self.create_timer(timer_period, timer_callback)


class WegoPubMoveJ(WegoPublisher):
    def __init__(self, position):
        super().__init__("wego_pub_movej_node", "joint_states", JointState, self.publisher_joint)
        self.position = position

    def publisher_joint(self):
        self.msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        self.msg.position = self.position
        self.pub.publish(self.msg)


class WegoPubMoveP(WegoPublisher):
    def __init__(self, waypoint):
        super().__init__("wego_pub_movep_node", "pos_cmd", PosCmd, self.publisher_point)
        self.waypoint = waypoint

    def publisher_point(self):
        # [0x01, 0x00] 를 숨기기 위해 internal 처리
        self.msg.x, self.msg.y, self.msg.z = self.waypoint[:3]
        self.msg.roll, self.msg.pitch, self.msg.yaw = self.waypoint[3:6]
        self.msg.gripper = self.waypoint[6]
        self.msg.mode1, self.msg.mode2 = [0x01, 0x00]
        self.pub.publish(self.msg)


class WegoPubMoveL(WegoPublisher):
    def __init__(self, waypoint):
        super().__init__("wego_pub_movel_node", "pos_cmd", PosCmd, self.publisher_point)
        self.waypoint = waypoint

    def publisher_point(self):
        # [0x01, 0x02] 를 숨기기 위해 internal 처리
        self.msg.x, self.msg.y, self.msg.z = self.waypoint[:3]
        self.msg.roll, self.msg.pitch, self.msg.yaw = self.waypoint[3:6]
        self.msg.gripper = self.waypoint[6]
        self.msg.mode1, self.msg.mode2 = [0x01, 0x02]
        self.pub.publish(self.msg)


class WegoPubMoveC(WegoPublisher):
    def __init__(self, waypoints):
        super().__init__("wego_pub_movec_node", "pos_cmd", PosCmd, self.publisher_point)
        self.waypoints = waypoints
        self.count = 0

    def publisher_point(self):
        if self.count < len(self.waypoints):
            waypoint = self.waypoints[self.count]
            self.msg.x, self.msg.y, self.msg.z = waypoint[:3]
            self.msg.roll, self.msg.pitch, self.msg.yaw = waypoint[3:6]
            self.msg.gripper = waypoint[6]
            self.msg.mode1, self.msg.mode2 = [0x01, 0x03]
            self.pub.publish(self.msg)
            self.get_logger().info(f"Published waypoint {self.count + 1}: {waypoint}")
            self.count += 1


def main(args=None):
    rclpy.init(args=args)

    waypoints = {
        "1": lambda: WegoPubMoveJ([0.3, 0.3, -0.3, 0.3, -0.3, 0.3, 0.0]),
        "2": lambda: WegoPubMoveP([190.0, 0.0, 400.0, 0.0, 85.0, 0.0, 70.0]),
        "3": lambda: WegoPubMoveL([190.0, 0.0, 250.0, 0.0, 85.0, 0.0, 0.0]),
        "4": lambda: WegoPubMoveC([[60.0, 0.0, 250.0, 0.0, 85.0, 0.0, 0.0], [200.0, 0.0, 400.0, 0.0, 85.0, 0.0, 0.0], [60.0, 0.0, 550.0, 0.0, 85.0, 0.0, 0.0]]),
    }

    # Request waypoint input
    waypoint = input("Enter waypoint: ")

    # Get the corresponding class based on user input
    node_class = waypoints.get(waypoint)

    if node_class:
        node = node_class()  # Instantiate the node based on user input
        rclpy.spin(node)
        node.destroy_node()
    else:
        print("Invalid waypoint type!")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
