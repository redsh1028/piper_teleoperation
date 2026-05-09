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
            DeclareLaunchArgument("position_deadzone", default_value="0.002"),
            DeclareLaunchArgument("max_target_offset_m", default_value="0.35"),
            DeclareLaunchArgument("smooth_command", default_value="true"),
            DeclareLaunchArgument("max_linear_speed_mps", default_value="0.20"),
            DeclareLaunchArgument("max_angular_speed_radps", default_value="0.70"),
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
                executable="vr_piper_anchor_teleop",
                name="vr_piper_anchor_smooth_teleop",
                output="screen",
                parameters=[
                    {
                        "axis_map": LaunchConfiguration("axis_map"),
                        "controller_axis_map": LaunchConfiguration(
                            "controller_axis_map"
                        ),
                        "position_scale": LaunchConfiguration("position_scale"),
                        "orientation_scale": LaunchConfiguration("orientation_scale"),
                        "position_deadzone": LaunchConfiguration("position_deadzone"),
                        "max_target_offset_m": LaunchConfiguration(
                            "max_target_offset_m"
                        ),
                        "smooth_command": LaunchConfiguration("smooth_command"),
                        "max_linear_speed_mps": LaunchConfiguration(
                            "max_linear_speed_mps"
                        ),
                        "max_angular_speed_radps": LaunchConfiguration(
                            "max_angular_speed_radps"
                        ),
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
