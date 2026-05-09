#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WegoPublisher(Node):
    def __init__(self):
        super().__init__("wego_pub_movej_node")  # 노드 이름 설정
        self.pub = self.create_publisher(JointState, "joint_states", 10)  # 퍼블리셔 생성
        self.timer = self.create_timer(2.0, self.publisher_joint)  # 수정된 부분
        self.msg = JointState()  # 메시지 객체 생성
        self.msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]

    def publisher_joint(self):
        self.msg.position = [0.3, 0.3, -0.3, 0.3, -0.3, 0.3, 0.00]  # 각 조인트 값
        self.pub.publish(self.msg)  # 메시지 퍼블리시


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = WegoPublisher()  # 노드 생성
    rclpy.spin(node)  # 콜백 실행
    node.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS 2 종료


if __name__ == "__main__":
    main()
