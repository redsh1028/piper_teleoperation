from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "repo_id", default_value="local/piper_teleoperation_video"
            ),
            DeclareLaunchArgument(
                "root",
                default_value="~/lerobot_datasets/cube_task",
            ),
            DeclareLaunchArgument("fps", default_value="15"),
            DeclareLaunchArgument("image_bind", default_value="0.0.0.0"),
            DeclareLaunchArgument("image_port", default_value="5020"),
            DeclareLaunchArgument("main_image_port", default_value="5020"),
            DeclareLaunchArgument("left_wrist_image_port", default_value="5021"),
            DeclareLaunchArgument("right_wrist_image_port", default_value="5022"),
            DeclareLaunchArgument("image_width", default_value="640"),
            DeclareLaunchArgument("image_height", default_value="480"),
            DeclareLaunchArgument("main_image_width", default_value="1280"),
            DeclareLaunchArgument("main_image_height", default_value="720"),
            DeclareLaunchArgument("left_wrist_image_width", default_value="640"),
            DeclareLaunchArgument("left_wrist_image_height", default_value="480"),
            DeclareLaunchArgument("right_wrist_image_width", default_value="640"),
            DeclareLaunchArgument("right_wrist_image_height", default_value="480"),
            DeclareLaunchArgument(
                "task",
                default_value="teleoperate dual Piper arms",
            ),
            DeclareLaunchArgument("left_task", default_value=""),
            DeclareLaunchArgument("right_task", default_value=""),
            DeclareLaunchArgument("object_name", default_value=""),
            DeclareLaunchArgument("box_position", default_value=""),
            DeclareLaunchArgument("start_position", default_value=""),
            DeclareLaunchArgument("success", default_value="true"),
            DeclareLaunchArgument("note", default_value=""),
            DeclareLaunchArgument("robot_type", default_value="piper_dual_vr"),
            DeclareLaunchArgument("use_videos", default_value="true"),
            DeclareLaunchArgument("vcodec", default_value="h264"),
            DeclareLaunchArgument("recv_buffer", default_value="4194304"),
            DeclareLaunchArgument("max_datagram_size", default_value="65535"),
            DeclareLaunchArgument("frame_timeout", default_value="1.0"),
            DeclareLaunchArgument("image_timeout", default_value="1.0"),
            Node(
                package="vr_udp_bridge",
                executable="lerobot_piper_recorder",
                name="lerobot_piper_recorder",
                output="screen",
                parameters=[
                    {
                        "repo_id": LaunchConfiguration("repo_id"),
                        "root": LaunchConfiguration("root"),
                        "fps": LaunchConfiguration("fps"),
                        "image_bind": LaunchConfiguration("image_bind"),
                        "image_port": LaunchConfiguration("image_port"),
                        "main_image_port": LaunchConfiguration("main_image_port"),
                        "left_wrist_image_port": LaunchConfiguration(
                            "left_wrist_image_port"
                        ),
                        "right_wrist_image_port": LaunchConfiguration(
                            "right_wrist_image_port"
                        ),
                        "image_width": LaunchConfiguration("image_width"),
                        "image_height": LaunchConfiguration("image_height"),
                        "main_image_width": LaunchConfiguration("main_image_width"),
                        "main_image_height": LaunchConfiguration(
                            "main_image_height"
                        ),
                        "left_wrist_image_width": LaunchConfiguration(
                            "left_wrist_image_width"
                        ),
                        "left_wrist_image_height": LaunchConfiguration(
                            "left_wrist_image_height"
                        ),
                        "right_wrist_image_width": LaunchConfiguration(
                            "right_wrist_image_width"
                        ),
                        "right_wrist_image_height": LaunchConfiguration(
                            "right_wrist_image_height"
                        ),
                        "task": LaunchConfiguration("task"),
                        "left_task": LaunchConfiguration("left_task"),
                        "right_task": LaunchConfiguration("right_task"),
                        "object_name": LaunchConfiguration("object_name"),
                        "box_position": LaunchConfiguration("box_position"),
                        "start_position": LaunchConfiguration("start_position"),
                        "success": LaunchConfiguration("success"),
                        "note": LaunchConfiguration("note"),
                        "robot_type": LaunchConfiguration("robot_type"),
                        "use_videos": LaunchConfiguration("use_videos"),
                        "vcodec": LaunchConfiguration("vcodec"),
                        "recv_buffer": LaunchConfiguration("recv_buffer"),
                        "max_datagram_size": LaunchConfiguration(
                            "max_datagram_size"
                        ),
                        "frame_timeout": LaunchConfiguration("frame_timeout"),
                        "image_timeout": LaunchConfiguration("image_timeout"),
                    }
                ],
            ),
        ]
    )
