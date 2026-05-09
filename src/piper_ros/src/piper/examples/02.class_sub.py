#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class WegoSubscriber(Node):
    def __init__(self):
        super().__init__("wego_sub_node")  # 노드 이름 설정
        self.sub = self.create_subscription(Int32, "counter", self.listener_callback, 10)  # 서브스크라이버 생성
        self.get_logger().info("WegoSubscriber has started and is listening to 'counter' topic.")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")  # 메시지 수신 시 로그 출력


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = WegoSubscriber()  # WegoSubscriber 노드 생성
    rclpy.spin(node)  # 노드 실행하여 콜백 함수 지속 처리
    node.destroy_node()  # 노드 리소스 해제
    rclpy.shutdown()  # ROS 2 종료

if __name__ == "__main__":
    main()