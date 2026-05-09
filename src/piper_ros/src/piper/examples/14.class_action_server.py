import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer


class ActionServerNode(Node):
    def __init__(self):
        super().__init__("action_server_node")
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            self.execute_callback,
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Received goal: {goal_handle.request}")
        # 목표를 처리하고 결과 반환
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result


def main():
    rclpy.init()
    node = ActionServerNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
