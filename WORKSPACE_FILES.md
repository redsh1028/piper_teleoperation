# piper_tele_ws file set

This workspace contains only the files used for the current dual Piper VR teleoperation and LeRobot recording workflow.

## Root scripts

- `send_openvr_udp.py`: sends SteamVR/OpenVR controller pose and input over UDP.
- `send_realsense_udp.py`: sends one RealSense camera stream over UDP.
- `send_three_realsense_udp.sh`: starts the three RealSense UDP senders.
- `receive_realsense_udp_viewer.py`: receives and displays the three camera streams.
- `can_config.sh`, `can_activate.sh`, `can_find_and_config.sh`, `find_all_can_port.sh`, `99-can.rules`: CAN setup helpers.
- `install_realsense_sdk.sh`: optional RealSense SDK install helper.
- `requirements.txt`: Python package requirements.
- `README.md`: usage guide.

## ROS packages

- `src/vr_udp_bridge`
  - `udp_to_pose`
  - `vr_piper_joint_ik_axis_gripper_teleop`
  - `lerobot_piper_recorder`
  - launch files for teleop and LeRobot recording.
- `src/piper_ros/src/piper`
  - minimal Piper ROS node package with `piper_single_ctrl_node.py` and `start_dual_piper.launch.py`.
- `src/piper_ros/src/piper_msgs`
  - Piper ROS messages and service.
- `src/piper_ros/src/piper_description`
  - Piper URDF and meshes used by direct IK.
- `src/piper_sdk`
  - Python Piper SDK packaged for this workspace.

## Build

```bash
cd ~/piper_tele_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Main launch

```bash
ros2 launch vr_udp_bridge vr_piper_joint_ik_axis_gripper_teleop.launch.py \
  left_can_port:=can0 \
  right_can_port:=can1 \
  debug_axis:=true
```
