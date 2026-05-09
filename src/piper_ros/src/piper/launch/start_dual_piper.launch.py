from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _piper_node(namespace, can_port):
    return Node(
        package="piper",
        executable="piper_single_ctrl_node.py",
        namespace=namespace,
        name="piper_single_ctrl_node",
        output="screen",
        parameters=[
            {
                "can_port": can_port,
                "auto_enable": LaunchConfiguration("auto_enable"),
                "rviz_ctrl_flag": LaunchConfiguration("rviz_ctrl_flag"),
                "gripper_exist": LaunchConfiguration("gripper_exist"),
                "debug_flag": LaunchConfiguration("debug_flag"),
            }
        ],
    )


def generate_launch_description():
    left_can_port_arg = DeclareLaunchArgument(
        "left_can_port",
        default_value="can0",
        description="CAN interface for the left Piper",
    )
    right_can_port_arg = DeclareLaunchArgument(
        "right_can_port",
        default_value="can1",
        description="CAN interface for the right Piper",
    )
    auto_enable_arg = DeclareLaunchArgument(
        "auto_enable",
        default_value="true",
        description="Automatically enable both Piper arms",
    )
    rviz_ctrl_flag_arg = DeclareLaunchArgument(
        "rviz_ctrl_flag",
        default_value="false",
        description="Enable RViz control mode",
    )
    gripper_exist_arg = DeclareLaunchArgument(
        "gripper_exist",
        default_value="true",
        description="Whether grippers are installed",
    )
    debug_flag_arg = DeclareLaunchArgument(
        "debug_flag",
        default_value="false",
        description="Enable debug logging",
    )

    return LaunchDescription(
        [
            left_can_port_arg,
            right_can_port_arg,
            auto_enable_arg,
            rviz_ctrl_flag_arg,
            gripper_exist_arg,
            debug_flag_arg,
            _piper_node("left", LaunchConfiguration("left_can_port")),
            _piper_node("right", LaunchConfiguration("right_can_port")),
        ]
    )
