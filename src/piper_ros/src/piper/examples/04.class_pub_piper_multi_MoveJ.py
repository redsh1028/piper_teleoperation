#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WegoPublisher(Node):
    def __init__(self):
        super().__init__("wego_pub_movej_node")  # 노드 이름 설정
        self.pub = self.create_publisher(JointState, "joint_states", 10)  # 퍼블리셔 생성

        # 타이머 생성: 2초마다 콜백 함수 호출
        self.timer = self.create_timer(1.0, self.publisher_joint)
        self.msg = JointState()  # 메시지 객체 생성
        self.msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]

        # Waypoints 정의 (값을 float으로 변경)
        self.waypoints = [
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (0.523, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (0.0, 0.523, 0.0, 0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, -0.523, 0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.523, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0, -0.523, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.523, 0.0),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.07),
            (0.523, 0.523, -0.523, 0.523, -0.523, 0.523, 0),
        ]

        # 카운터와 인덱스 설정
        self.count = 0
        self.index = 0

    def publisher_joint(self):
        # Waypoint 순차적으로 변경
        self.msg.position = [float(val) for val in self.waypoints[self.index]]  # 전체 waypoint를 사용하여 float으로 변환

        self.pub.publish(self.msg)  # 퍼블리시

        # 다음 index로 변경, 끝에 도달하면 0으로 리셋
        self.index = (self.index + 1) % len(self.waypoints)  # 순차적으로 인덱스를 증가시키며, 끝에 도달하면 0으로 리셋


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = WegoPublisher()  # 노드 생성
    rclpy.spin(node)  # 콜백 실행
    node.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS 2 종료


if __name__ == "__main__":
    main()
