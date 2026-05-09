from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare the launch arguments
    can_port_arg = DeclareLaunchArgument("can_port", default_value="can0", description="PiPER 노드에서 사용할 CAN 포트")

    auto_enable_arg = DeclareLaunchArgument("auto_enable", default_value="true", description="PiPER 노드를 자동으로 활성화 여부")

    rviz_ctrl_flag_arg = DeclareLaunchArgument("rviz_ctrl_flag", default_value="false", description="RViz 제어 활성화/비활성화 플래그")

    gripper_exist_arg = DeclareLaunchArgument("gripper_exist", default_value="true", description="그리퍼 존재 여부 플래그")

    debug_flag_arg = DeclareLaunchArgument("debug_flag", default_value="true", description="로그 확인 여부 플래그")

    # Define the node
    piper_node = Node(
        package="piper",
        executable="piper_single_ctrl_node.py",
        output="screen",
        parameters=[
            {
                "can_port": LaunchConfiguration("can_port"),
                "auto_enable": LaunchConfiguration("auto_enable"),
                "rviz_ctrl_flag": LaunchConfiguration("rviz_ctrl_flag"),
                "gripper_exist": LaunchConfiguration("gripper_exist"),
                "debug_flag": LaunchConfiguration("debug_flag"),
            }
        ],
        remappings=[("/joint_ctrl_single", "/joint_states")],
    )

    # Return the LaunchDescription
    return LaunchDescription([can_port_arg, auto_enable_arg, rviz_ctrl_flag_arg, gripper_exist_arg, debug_flag_arg, piper_node])
