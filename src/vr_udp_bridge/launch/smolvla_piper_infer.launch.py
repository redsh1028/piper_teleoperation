from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "policy_path",
                default_value="~/piper_tele_ws/checkpoints/smolvla_cube_task_010000",
            ),
            DeclareLaunchArgument(
                "task",
                default_value="Put both green cubes into the box",
            ),
            DeclareLaunchArgument("device", default_value="cpu"),
            DeclareLaunchArgument("publish_commands", default_value="false"),
            DeclareLaunchArgument("inference_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("image_bind", default_value="0.0.0.0"),
            DeclareLaunchArgument("main_image_port", default_value="5020"),
            DeclareLaunchArgument("left_wrist_image_port", default_value="5021"),
            DeclareLaunchArgument("right_wrist_image_port", default_value="5022"),
            DeclareLaunchArgument("main_image_width", default_value="1280"),
            DeclareLaunchArgument("main_image_height", default_value="720"),
            DeclareLaunchArgument("left_wrist_image_width", default_value="640"),
            DeclareLaunchArgument("left_wrist_image_height", default_value="480"),
            DeclareLaunchArgument("right_wrist_image_width", default_value="640"),
            DeclareLaunchArgument("right_wrist_image_height", default_value="480"),
            DeclareLaunchArgument("image_timeout", default_value="1.0"),
            DeclareLaunchArgument("max_joint_step_rad", default_value="0.04"),
            DeclareLaunchArgument("max_gripper_step_m", default_value="0.008"),
            DeclareLaunchArgument("action_mode", default_value="absolute"),
            DeclareLaunchArgument("action_smoothing_alpha", default_value="1.0"),
            DeclareLaunchArgument("gripper_threshold_m", default_value="-1.0"),
            DeclareLaunchArgument("action_chunk_steps", default_value="1"),
            DeclareLaunchArgument("interpolate_commands", default_value="false"),
            DeclareLaunchArgument("command_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("move_to_home_on_start", default_value="true"),
            DeclareLaunchArgument(
                "left_home_joint_positions",
                default_value="-0.25,0.877031988,-1.130597972,0,1.220678788,0",
            ),
            DeclareLaunchArgument(
                "right_home_joint_positions",
                default_value="0.25,0.877031988,-1.130597972,0,1.220678788,0",
            ),
            DeclareLaunchArgument("home_gripper_position", default_value="0.10"),
            DeclareLaunchArgument("home_tolerance_rad", default_value="0.03"),
            DeclareLaunchArgument("home_timeout_s", default_value="20.0"),
            DeclareLaunchArgument("home_settle_s", default_value="0.7"),
            DeclareLaunchArgument("home_max_joint_step_rad", default_value="0.04"),
            DeclareLaunchArgument("home_max_gripper_step_m", default_value="0.010"),
            DeclareLaunchArgument("log_actions", default_value="true"),
            Node(
                package="vr_udp_bridge",
                executable="lerobot_piper_smolvla_infer",
                name="lerobot_piper_smolvla_infer",
                output="screen",
                parameters=[
                    {
                        "policy_path": LaunchConfiguration("policy_path"),
                        "task": LaunchConfiguration("task"),
                        "device": LaunchConfiguration("device"),
                        "publish_commands": LaunchConfiguration("publish_commands"),
                        "inference_rate_hz": LaunchConfiguration("inference_rate_hz"),
                        "image_bind": LaunchConfiguration("image_bind"),
                        "main_image_port": LaunchConfiguration("main_image_port"),
                        "left_wrist_image_port": LaunchConfiguration(
                            "left_wrist_image_port"
                        ),
                        "right_wrist_image_port": LaunchConfiguration(
                            "right_wrist_image_port"
                        ),
                        "main_image_width": LaunchConfiguration("main_image_width"),
                        "main_image_height": LaunchConfiguration("main_image_height"),
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
                        "image_timeout": LaunchConfiguration("image_timeout"),
                        "max_joint_step_rad": LaunchConfiguration(
                            "max_joint_step_rad"
                        ),
                        "max_gripper_step_m": LaunchConfiguration(
                            "max_gripper_step_m"
                        ),
                        "action_mode": LaunchConfiguration("action_mode"),
                        "action_smoothing_alpha": LaunchConfiguration(
                            "action_smoothing_alpha"
                        ),
                        "gripper_threshold_m": LaunchConfiguration(
                            "gripper_threshold_m"
                        ),
                        "action_chunk_steps": LaunchConfiguration(
                            "action_chunk_steps"
                        ),
                        "interpolate_commands": LaunchConfiguration(
                            "interpolate_commands"
                        ),
                        "command_rate_hz": LaunchConfiguration("command_rate_hz"),
                        "move_to_home_on_start": LaunchConfiguration(
                            "move_to_home_on_start"
                        ),
                        "left_home_joint_positions": LaunchConfiguration(
                            "left_home_joint_positions"
                        ),
                        "right_home_joint_positions": LaunchConfiguration(
                            "right_home_joint_positions"
                        ),
                        "home_gripper_position": LaunchConfiguration(
                            "home_gripper_position"
                        ),
                        "home_tolerance_rad": LaunchConfiguration(
                            "home_tolerance_rad"
                        ),
                        "home_timeout_s": LaunchConfiguration("home_timeout_s"),
                        "home_settle_s": LaunchConfiguration("home_settle_s"),
                        "home_max_joint_step_rad": LaunchConfiguration(
                            "home_max_joint_step_rad"
                        ),
                        "home_max_gripper_step_m": LaunchConfiguration(
                            "home_max_gripper_step_m"
                        ),
                        "log_actions": LaunchConfiguration("log_actions"),
                    }
                ],
            ),
        ]
    )
