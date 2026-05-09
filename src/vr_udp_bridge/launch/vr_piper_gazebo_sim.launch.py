from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    piper_gazebo_launch = PythonLaunchDescriptionSource(
        [
            FindPackageShare("piper_gazebo"),
            "/launch/piper_with_gripper/piper_gazebo.launch.py",
        ]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(piper_gazebo_launch),
            Node(
                package="vr_udp_bridge",
                executable="udp_to_pose",
                name="udp_to_pose",
                output="screen",
            ),
            Node(
                package="vr_udp_bridge",
                executable="vr_piper_gazebo_joint_teleop",
                name="vr_piper_gazebo_joint_teleop",
                output="screen",
                parameters=[
                    {
                        "controller_side": "right",
                        "enable_button_index": 0,
                        "arm_command_topic": "/arm_controller/joint_trajectory",
                        "gripper_command_topic": "/gripper_controller/joint_trajectory",
                    }
                ],
            ),
        ]
    )
