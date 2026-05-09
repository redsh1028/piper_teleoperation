#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class WegoPublisher(Node): 
    def __init__(self):
        super().__init__("wego_pub_node")  # 노드 이름 설정
        self.pub = self.create_publisher(Int32, "counter", 10)  # 퍼블리셔 생성
        self.timer = self.create_timer(0.5, self.publish_counter)  # 0.5초마다 실행되는 타이머 생성
        self.msg = Int32()  # 메시지 객체 생성
        self.num = 0 

    def publish_counter(self):
        self.num += 1
        self.msg.data = self.num # 데이터 업데이트
        self.pub.publish(self.msg)  # 메시지 퍼블리시
        self.get_logger().info(f"Published: {self.msg.data}")  # 로그 출력

def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = WegoPublisher()  # 노드 생성
    rclpy.spin(node)  # 콜백 실행
    node.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS 2 종료

if __name__ == "__main__":
    main()
