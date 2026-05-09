#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from piper_msgs.msg import PosCmd


class WegoPublisher(Node):
    def __init__(self):
        super().__init__("wego_pub_movec_node")  # 노드 이름 설정
        self.pub = self.create_publisher(PosCmd, "pos_cmd", 10)  # 퍼블리셔 생성
        self.msg = PosCmd()  # 메시지 객체 생성
        self.waypoints = [[60.0, 0.0, 250.0, 0.0, 85.0, 0.0, 0.0], [200.0, 0.0, 400.0, 0.0, 85.0, 0.0, 0.0], [60.0, 0.0, 550.0, 0.0, 85.0, 0.0, 0.0]]
        self.mode = [0x01, 0x03]

    def publisher_point(self, count: int):
        waypoint = self.waypoints[count]
        combined = waypoint + self.mode

        self.msg.x, self.msg.y, self.msg.z = combined[0], combined[1], combined[2]
        self.msg.roll, self.msg.pitch, self.msg.yaw = combined[3], combined[4], combined[5]
        self.msg.gripper = combined[6]
        self.msg.mode1, self.msg.mode2 = combined[7], combined[8]

        self.pub.publish(self.msg)
        self.get_logger().info(f"Published waypoint {count + 1}: {waypoint}")


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = WegoPublisher()  # 노드 생성
    node.publisher_point(0)
    node.publisher_point(1)
    node.publisher_point(2)
    node.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS 2 종료


if __name__ == "__main__":
    main()
