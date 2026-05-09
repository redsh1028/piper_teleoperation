#!/usr/bin/env python3

import rclpy
import os
import yaml
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


def load_waypoints(file_path):
    file_path = os.path.expanduser(file_path)
    with open(file_path, "r") as file:
        return yaml.safe_load(file)["waypoints"]


def main(args=None):
    rclpy.init(args=args)

    # Load waypoints from YAML file
    waypoints = load_waypoints("./waypoints.yaml")

    # Request waypoint number input from user
    waypoint_number = input("Enter waypoint number (1, 2, 3, 4, ...): ")

    # Find the corresponding waypoint based on the number
    waypoint_data = waypoints.get(int(waypoint_number), None)

    if waypoint_data:
        wp_type = waypoint_data["type"]
        node_class = {
            "movej": WegoPubMoveJ,
            "movep": WegoPubMoveP,
            "movel": WegoPubMoveL,
            "movec": WegoPubMoveC,
        }.get(wp_type)

        if node_class:
            # Pass the appropriate data to the class constructor
            if wp_type == "movec":
                node = node_class(waypoint_data["waypoints"])
            else:
                node = node_class(waypoint_data["waypoint"])

            rclpy.spin(node)
            node.destroy_node()
        else:
            print(f"Invalid type: {wp_type}")
    else:
        print("Invalid waypoint number!")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
