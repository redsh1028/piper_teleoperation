from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    piper_launch = PythonLaunchDescriptionSource(
        [FindPackageShare("piper"), "/launch/start_dual_piper.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("left_can_port", default_value="can0"),
            DeclareLaunchArgument("right_can_port", default_value="can1"),
            DeclareLaunchArgument("auto_enable", default_value="true"),
            DeclareLaunchArgument("gripper_exist", default_value="true"),
            DeclareLaunchArgument("piper_debug_flag", default_value="false"),
            DeclareLaunchArgument("axis_map", default_value="-z,-x,y"),
            DeclareLaunchArgument("controller_axis_map", default_value="-z,-x,y"),
            DeclareLaunchArgument("position_scale", default_value="0.8"),
            DeclareLaunchArgument("orientation_scale", default_value="0.9"),
            DeclareLaunchArgument("publish_rate_hz", default_value="40.0"),
            DeclareLaunchArgument("position_deadzone", default_value="0.002"),
            DeclareLaunchArgument("max_target_offset_m", default_value="0.35"),
            DeclareLaunchArgument("max_joint_step_rad", default_value="0.05"),
            DeclareLaunchArgument("ik_position_weight", default_value="1.0"),
            DeclareLaunchArgument("ik_orientation_weight", default_value="0.6"),
            DeclareLaunchArgument("joint_continuity_weight", default_value="0.25"),
            DeclareLaunchArgument("ik_max_iterations", default_value="30"),
            DeclareLaunchArgument("base_link", default_value="base_link"),
            DeclareLaunchArgument("tip_link", default_value="gripper_base"),
            DeclareLaunchArgument(
                "position_reference_frame", default_value="controller_anchor"
            ),
            DeclareLaunchArgument(
                "orientation_reference_frame", default_value="controller_anchor"
            ),
            DeclareLaunchArgument("lock_orientation", default_value="false"),
            DeclareLaunchArgument("debug_axis", default_value="false"),
            DeclareLaunchArgument("enable_button_index", default_value="0"),
            DeclareLaunchArgument("trigger_axis_index", default_value="2"),
            DeclareLaunchArgument("gripper_toggle_button_index", default_value="1"),
            DeclareLaunchArgument("gripper_mode", default_value="toggle"),
            DeclareLaunchArgument("filter_window_size", default_value="3"),
            IncludeLaunchDescription(
                piper_launch,
                launch_arguments={
                    "left_can_port": LaunchConfiguration("left_can_port"),
                    "right_can_port": LaunchConfiguration("right_can_port"),
                    "auto_enable": LaunchConfiguration("auto_enable"),
                    "gripper_exist": LaunchConfiguration("gripper_exist"),
                    "debug_flag": LaunchConfiguration("piper_debug_flag"),
                }.items(),
            ),
            Node(
                package="vr_udp_bridge",
                executable="udp_to_pose",
                name="udp_to_pose",
                output="screen",
            ),
            Node(
                package="vr_udp_bridge",
                executable="vr_piper_joint_ik_teleop",
                name="vr_piper_joint_ik_teleop",
                output="screen",
                parameters=[
                    {
                        "axis_map": LaunchConfiguration("axis_map"),
                        "controller_axis_map": LaunchConfiguration(
                            "controller_axis_map"
                        ),
                        "position_scale": LaunchConfiguration("position_scale"),
                        "orientation_scale": LaunchConfiguration("orientation_scale"),
                        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                        "position_deadzone": LaunchConfiguration("position_deadzone"),
                        "max_target_offset_m": LaunchConfiguration(
                            "max_target_offset_m"
                        ),
                        "max_joint_step_rad": LaunchConfiguration(
                            "max_joint_step_rad"
                        ),
                        "ik_position_weight": LaunchConfiguration(
                            "ik_position_weight"
                        ),
                        "ik_orientation_weight": LaunchConfiguration(
                            "ik_orientation_weight"
                        ),
                        "joint_continuity_weight": LaunchConfiguration(
                            "joint_continuity_weight"
                        ),
                        "ik_max_iterations": LaunchConfiguration(
                            "ik_max_iterations"
                        ),
                        "base_link": LaunchConfiguration("base_link"),
                        "tip_link": LaunchConfiguration("tip_link"),
                        "position_reference_frame": LaunchConfiguration(
                            "position_reference_frame"
                        ),
                        "orientation_reference_frame": LaunchConfiguration(
                            "orientation_reference_frame"
                        ),
                        "lock_orientation": LaunchConfiguration("lock_orientation"),
                        "debug_axis": LaunchConfiguration("debug_axis"),
                        "enable_button_index": LaunchConfiguration(
                            "enable_button_index"
                        ),
                        "trigger_axis_index": LaunchConfiguration(
                            "trigger_axis_index"
                        ),
                        "gripper_toggle_button_index": LaunchConfiguration(
                            "gripper_toggle_button_index"
                        ),
                        "gripper_mode": LaunchConfiguration("gripper_mode"),
                        "filter_window_size": LaunchConfiguration(
                            "filter_window_size"
                        ),
                    }
                ],
            ),
        ]
    )
