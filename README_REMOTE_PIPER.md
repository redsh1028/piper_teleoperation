# Remote Piper Teleop / SmolVLA Setup

This workspace is prepared on `pjh22@172.16.64.161` so the remote PC can directly run cameras, CAN robot control, and GPU SmolVLA inference.

Workspace:

```bash
cd ~/piper_tele_ws
source setup_remote_env.sh
```

Always source `setup_remote_env.sh` first. It loads ROS Humble, this workspace, Piper SDK source path, and the ZED Open Capture library path.

## What Is Installed

- ROS workspace: `~/piper_tele_ws`
- SmolVLA checkpoint:

```bash
~/piper_tele_ws/checkpoints/smolvla_bottle_task_one_020000/pretrained_model
```

- GPU inference deps: `lerobot==0.4.4`, CUDA PyTorch
- ZED UDP sender: `~/piper_tele_ws/send_zed_open_capture_udp`
- Camera sender script: `~/piper_tele_ws/send_three_realsense_udp.sh`
- Piper ROS node: `piper piper_single_ctrl_node.py`
- VR / recorder / inference nodes: `vr_udp_bridge`

## 1. Connect Hardware

Connect directly to the remote PC:

- ZED 2i main camera
- Left wrist RealSense D435
- Right wrist RealSense D435
- Left Piper CAN adapter
- Right Piper CAN adapter

Then check devices:

```bash
cd ~/piper_tele_ws
source setup_remote_env.sh

v4l2-ctl --list-devices
lsusb -t
ip -br link show type can
```

Expected:

- ZED 2i appears in `v4l2-ctl --list-devices`
- ZED should be on USB3 speed: `5000M` or `10000M` in `lsusb -t`
- Two CAN interfaces should appear before naming/config, often `can0`, `can1`, or temporary names

If ZED appears at `480M`, reconnect it to a USB3 port. HD720 will fail at USB2 speed.

## 2. Configure CAN

First inspect CAN adapters and USB bus mapping:

```bash
cd ~/piper_tele_ws
source setup_remote_env.sh

ip -br link show type can
./find_all_can_port.sh
```

The copied `can_config.sh` currently expects two CAN adapters and maps USB bus-info to:

```text
1-7:1.0 -> can0 @ 1000000
1-8:1.0 -> can1 @ 1000000
```

The remote PC USB addresses may differ. If so, edit `can_config.sh` and update:

```bash
declare -A USB_PORTS
USB_PORTS["<left_usb_bus_info>"]="can0:1000000"
USB_PORTS["<right_usb_bus_info>"]="can1:1000000"
```

Then run:

```bash
sudo bash ./can_config.sh
ip -br link show type can
```

Expected final state:

```text
can0 UP ...
can1 UP ...
```

## 3. Start Camera UDP Senders

```bash
cd ~/piper_tele_ws
source setup_remote_env.sh
./send_three_realsense_udp.sh
```

Expected logs:

```text
Starting main ZED 2i with zed-open-capture: resolution=HD720, crop=right-half, port=5020
Starting left_wrist: serial=116622072176, port=5021
Starting right_wrist: serial=134322071792, port=5022
ZED Open Capture started ... resolution=HD720 ...
```

If ZED fails with resolution error, check USB speed with:

```bash
lsusb -t
v4l2-ctl --list-devices
```

Temporary low-resolution fallback only if needed:

```bash
MAIN_ZED_RESOLUTION=VGA FPS=15 ./send_three_realsense_udp.sh
```

Do not use VGA for final evaluation unless necessary.

## 4. Start Teleoperation

Use this when collecting data or manually testing arms:

```bash
cd ~/piper_tele_ws
source setup_remote_env.sh

ros2 launch vr_udp_bridge vr_piper_joint_ik_axis_gripper_teleop.launch.py \
  left_can_port:=can0 \
  right_can_port:=can1 \
  debug_axis:=true
```

Important teleop settings in the launch file:

```text
max_target_offset_m = 0.7
left_home_joint_positions  = -0.25,0.877031988,-1.130597972,0,1.220678788,0
right_home_joint_positions =  0.25,0.877031988,-1.130597972,0,1.220678788,0
```

## 5. Start SmolVLA GPU Inference

Right bottle task:

```bash
cd ~/piper_tele_ws
source setup_remote_env.sh

ros2 launch vr_udp_bridge smolvla_piper_infer.launch.py \
  policy_path:=~/piper_tele_ws/checkpoints/smolvla_bottle_task_one_020000 \
  task:="Pick the bottle on the right and place it in the box." \
  device:=cuda \
  publish_commands:=true \
  move_to_home_on_start:=true \
  max_joint_step_rad:=0.08 \
  max_gripper_step_m:=0.015 \
  home_max_joint_step_rad:=0.06 \
  home_max_gripper_step_m:=0.012
```

Left bottle task:

```bash
ros2 launch vr_udp_bridge smolvla_piper_infer.launch.py \
  policy_path:=~/piper_tele_ws/checkpoints/smolvla_bottle_task_one_020000 \
  task:="Pick the bottle on the left and place it in the box." \
  device:=cuda \
  publish_commands:=true \
  move_to_home_on_start:=true \
  max_joint_step_rad:=0.08 \
  max_gripper_step_m:=0.015 \
  home_max_joint_step_rad:=0.06 \
  home_max_gripper_step_m:=0.012
```

For safe dry-run without moving the robot:

```bash
ros2 launch vr_udp_bridge smolvla_piper_infer.launch.py \
  policy_path:=~/piper_tele_ws/checkpoints/smolvla_bottle_task_one_020000 \
  task:="Pick the bottle on the right and place it in the box." \
  device:=cuda \
  publish_commands:=false \
  move_to_home_on_start:=false \
  log_actions:=true
```

## 6. Safety Settings

Inference command limits:

```text
max_joint_step_rad       default 0.04
max_gripper_step_m       default 0.008
home_max_joint_step_rad  default 0.04
home_max_gripper_step_m  default 0.010
```

Currently used relaxed settings:

```text
max_joint_step_rad       0.08
max_gripper_step_m       0.015
home_max_joint_step_rad  0.06
home_max_gripper_step_m  0.012
```

If movement is too aggressive, reduce to:

```bash
max_joint_step_rad:=0.04 max_gripper_step_m:=0.008 home_max_joint_step_rad:=0.04
```

## 7. Stop Processes

Stop foreground launch or camera script:

```text
Ctrl+C
```

Kill inference from another terminal:

```bash
pkill -f lerobot_piper_smolvla_infer
```

Kill camera senders:

```bash
pkill -f send_realsense_udp.py
pkill -f send_zed_open_capture_udp
```

Check remaining processes:

```bash
pgrep -af 'smolvla|send_realsense|send_zed|piper_single'
```

## 8. Quick Health Checks

ROS packages:

```bash
source ~/piper_tele_ws/setup_remote_env.sh
ros2 pkg executables vr_udp_bridge
ros2 pkg executables piper
```

Expected:

```text
vr_udp_bridge lerobot_piper_recorder
vr_udp_bridge lerobot_piper_smolvla_infer
vr_udp_bridge udp_to_pose
vr_udp_bridge vr_piper_joint_ik_axis_gripper_teleop
piper piper_single_ctrl_node.py
```

GPU:

```bash
nvidia-smi
python3 -c 'import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))'
```

SmolVLA import:

```bash
python3 -c 'from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy; print("SmolVLA import OK")'
```

Checkpoint exists:

```bash
ls ~/piper_tele_ws/checkpoints/smolvla_bottle_task_one_020000/pretrained_model
```

## 9. Notes

- `piper_sdk` is intentionally used through source path in `setup_remote_env.sh`.
- `piper_sdk` was skipped in `colcon build`; do not remove the `PYTHONPATH=$HOME/piper_tele_ws/src` line.
- ZED Open Capture needs `LD_LIBRARY_PATH=$HOME/piper_tele_ws`, also set by `setup_remote_env.sh`.
- If code is updated on the local PC, resync `~/piper_tele_ws/src`, scripts, and checkpoints to the remote PC, then rebuild:

```bash
cd ~/piper_tele_ws
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install --packages-skip piper_sdk
source setup_remote_env.sh
```
