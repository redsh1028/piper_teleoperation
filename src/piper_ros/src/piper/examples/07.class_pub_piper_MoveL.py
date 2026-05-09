#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from piper_msgs.msg import PosCmd


class WegoPublisher(Node):
    def __init__(self):
        super().__init__("wego_pub_movel_node")  # 노드 이름 설정
        self.pub = self.create_publisher(PosCmd, "pos_cmd", 10)  # 퍼블리셔 생성
        self.timer = self.create_timer(2.0, self.publisher_point)  # 수정된 부분
        self.msg = PosCmd()  # 메시지 객체 생성
        self.waypoint = [160.0, 0.0, 250.0, 0.0, 85.0, 0.0, 10.0]  # 이동해야 할 위치
        self.mode = [0x01, 0x02]  # 바꾸지 말기

    def publisher_point(self):
        combined = self.waypoint + self.mode

        self.msg.x, self.msg.y, self.msg.z = combined[0], combined[1], combined[2]
        self.msg.roll, self.msg.pitch, self.msg.yaw = combined[3], combined[4], combined[5]
        self.msg.gripper = combined[6]
        self.msg.mode1, self.msg.mode2 = combined[7], combined[8]

        self.pub.publish(self.msg)  # 메시지 퍼블리시


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = WegoPublisher()  # 노드 생성
    rclpy.spin(node)  # 콜백 실행
    node.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS 2 종료


if __name__ == "__main__":
    main()
