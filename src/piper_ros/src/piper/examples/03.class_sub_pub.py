import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# 퍼블리셔 노드
class WegoPublisher(Node):
    def __init__(self):
        super().__init__("counter_publisher")
        self.pub = self.create_publisher(Int32, "counter", 10)  # counter 토픽 사용
        self.timer = self.create_timer(1.0, self.publish_counter)
        self.count = 0

    def publish_counter(self):
        msg = Int32()
        self.count += 1
        msg.data = self.count
        self.pub.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

# 서브스크라이버 노드
class WegoSubscriber(Node):
    def __init__(self):
        super().__init__("counter_subscriber")
        self.sub = self.create_subscription(Int32, "counter", self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

def main():
    rclpy.init()

    publisher = WegoPublisher()
    subscriber = WegoSubscriber()

    # ROS2 멀티스레드 지원 - 멀티 노드 실행 및 각 노드의 콜백이 병렬적으로 처리
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(subscriber)

    executor.spin()  # 두 개의 노드를 동시에 실행
    publisher.destroy_node()
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
