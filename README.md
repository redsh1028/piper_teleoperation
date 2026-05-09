# piper_teleoperation

Piper arm 2대를 VR 컨트롤러로 텔레오퍼레이션하기 위한 ROS2 Humble 워크스페이스입니다.

현재 메인 실행 코드는 `vr_piper_joint_ik_axis_gripper_teleop.launch.py`입니다. 이 launch는 VR 컨트롤러 상대 움직임으로 좌/우 Piper를 직접 IK joint 제어하고, 그리퍼는 `Joy.axes[1]` analog 값으로 별도 제어합니다.

## 핵심 기능

- OpenVR/SteamVR 컨트롤러 pose와 joy 입력을 UDP로 받아 ROS2 토픽으로 변환합니다.
- 버튼을 누른 순간의 VR controller pose와 Piper end-effector pose를 anchor로 저장하고, 이후 상대 변화량만 로봇에 반영합니다.
- Piper 내부 Cartesian IK 대신 이 프로젝트 노드가 직접 IK를 풀어 `/left/joint_ctrl_single`, `/right/joint_ctrl_single`에 `JointState`를 보냅니다.
- IK는 완벽한 pose 추종보다 이전 joint 자세와 가까운 해를 우선해 갑작스러운 branch jump를 줄입니다.
- 그리퍼는 `Joy.axes[1]`의 `-1~1` analog 값을 열기/닫기 속도 명령으로 사용하며, 0 근처에서는 마지막 그리퍼 값을 유지합니다.
- 버튼 1을 누르면 그리퍼가 즉시 완전 열림 위치로 이동합니다.
- 그리퍼 힘은 launch 파라미터 `gripper_effort`로 조절합니다.

## 주요 파일

```text
tele_ws/
├── send_openvr_udp.py
├── can_config.sh
├── find_all_can_port.sh
└── src/
    ├── piper_ros/src/piper/launch/start_dual_piper.launch.py
    ├── piper_ros/src/piper/scripts/piper_single_ctrl_node.py
    └── vr_udp_bridge/
        ├── launch/vr_piper_joint_ik_axis_gripper_teleop.launch.py
        └── vr_udp_bridge/
            ├── udp_to_pose.py
            ├── vr_piper_joint_ik_axis_gripper_teleop.py
            └── vr_piper_joint_ik_teleop.py
```

- `send_openvr_udp.py`: SteamVR/OpenVR 컨트롤러 정보를 UDP로 송신합니다.
- `udp_to_pose.py`: UDP JSON을 `/vr/.../pose`, `/vr/.../joy`, `/vr/.../status` 토픽으로 변환합니다.
- `vr_piper_joint_ik_axis_gripper_teleop.py`: 메인 teleop 노드입니다.
- `vr_piper_joint_ik_axis_gripper_teleop.launch.py`: 메인 launch 파일입니다.
- `piper_single_ctrl_node.py`: `/left`, `/right` namespace 안에서 CAN으로 실제 Piper를 제어합니다.

## 설치 및 빌드

```bash
cd ~/tele_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

필요한 Python 패키지:

```bash
pip3 install -r requirements.txt
```

직접 IK 노드는 `numpy`, `scipy`, `urdf_parser_py`, `PyYAML`을 사용합니다.

## CAN 설정

두 대의 Piper를 연결한 뒤 CAN 포트를 확인합니다.

```bash
cd ~/tele_ws
bash find_all_can_port.sh
```

예시:

```text
USB 포트 7-1:1.0에 인터페이스 can0를 삽입할 것을 추천합니다.
USB 포트 3-1:1.0에 인터페이스 can1를 삽입할 것을 추천합니다.
```

CAN을 수동으로 활성화하는 예:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

환경에 맞게 `can_config.sh`를 수정했다면 자동 설정도 가능합니다.

```bash
sudo bash can_config.sh
```

## VR 데이터 송신

SteamVR/OpenVR가 실행되는 쪽에서 컨트롤러 데이터를 UDP로 보냅니다.

```bash
python3 send_openvr_udp.py --host <ROS_PC_IP> --port 5005 --rate 60
```

ROS2 PC에서 입력 확인:

```bash
ros2 topic echo /vr/left_controller/pose
ros2 topic echo /vr/left_controller/joy
```

`Joy` 입력 예:

```yaml
axes:
- 0.0
- 0.18
buttons:
- 0
- 1
```

여기서 `axes[1]`은 그리퍼 analog 제어에 사용되고, `buttons[1]`은 그리퍼 완전 열림 버튼으로 사용됩니다.

## 메인 실행

기본 실행:

```bash
cd ~/tele_ws
source install/setup.bash

ros2 launch vr_udp_bridge vr_piper_joint_ik_axis_gripper_teleop.launch.py \
  left_can_port:=can0 \
  right_can_port:=can1 \
  debug_axis:=true
```

현재 launch 기본값:

```text
controller_axis_map=-z,-x,y
position_scale=0.8
orientation_scale=0.9
publish_rate_hz=60.0
max_joint_step_rad=0.03
joint_continuity_weight=0.35
gripper_axis_index=1
gripper_open_position_m=0.10
gripper_closed_position_m=0.0
gripper_max_step_m=0.006
gripper_smoothing_alpha=0.35
gripper_full_open_button_index=1
gripper_effort=2.0
home_button_index=2
home_both_arms=true
home_max_step_rad=0.03
```

처음 실제 로봇에서 확인할 때는 더 보수적인 값으로 시작하는 것을 권장합니다.

```bash
ros2 launch vr_udp_bridge vr_piper_joint_ik_axis_gripper_teleop.launch.py \
  left_can_port:=can0 \
  right_can_port:=can1 \
  position_scale:=0.4 \
  orientation_scale:=0.4 \
  max_joint_step_rad:=0.02 \
  joint_continuity_weight:=0.5 \
  gripper_max_step_m:=0.002 \
  gripper_effort:=1.5 \
  debug_axis:=true
```

## 조작 방법

팔 제어:

- `enable_button_index:=0` 버튼을 누르는 동안만 팔이 움직입니다.
- 버튼을 누르는 순간 현재 VR controller pose와 현재 Piper EE pose가 anchor로 저장됩니다.
- 버튼을 누른 채 컨트롤러를 움직이면 anchor 기준 변화량만 로봇에 반영됩니다.
- 버튼을 떼면 팔 목표 publish는 멈추고 anchor가 해제됩니다.
- 다시 누르면 현재 위치에서 새 anchor가 잡힙니다.

그리퍼 제어:

- `gripper_axis_index:=1`이므로 `Joy.axes[1]`을 사용합니다.
- `axes[1] > 0`: 한 방향으로 누적 이동합니다.
- `axes[1] < 0`: 반대 방향으로 누적 이동합니다.
- `abs(axes[1]) <= gripper_axis_deadzone`: 마지막 `joint7` 값을 유지합니다.
- `gripper_full_open_button_index:=1` 버튼을 누르면 즉시 완전 열림 `0.10m`로 이동합니다.
- 팔 enable 버튼을 누르지 않아도 그리퍼는 axes/button 입력만으로 독립 제어됩니다.

그리퍼만 제어할 때 `joint1~joint6`은 마지막 명령이 아니라 최신 `/joint_states_single` 피드백 값을 사용합니다. 그래서 그리퍼 명령이 이전 팔 자세로 로봇을 되돌리는 일을 줄입니다.

Home 버튼:

- `home_button_index:=2` 버튼을 누르고 있는 동안 현재 피드백 joint 자세에서 홈 자세 `[0, 0, 0, 0, 0, 0]` 방향으로 이동합니다.
- 버튼을 떼면 홈 이동을 중단하고, 그 순간의 현재 피드백 자세 기준으로 멈춥니다.
- 기본값은 `home_both_arms:=true`라서 어느 컨트롤러에서 눌러도 양쪽 Piper가 홈 방향으로 이동합니다.
- 홈 이동 중에는 VR pose 제어 anchor를 리셋하므로, 다시 조작하려면 enable 버튼을 새로 눌러 현재 위치에서 anchor를 다시 잡습니다.

## 좌표계

현재 확인된 축 매핑:

```text
controller_axis_map=-z,-x,y
```

의미:

```text
robot_x = -controller_z
robot_y = -controller_x
robot_z =  controller_y
```

축 방향이 반대면 코드 수정 없이 launch 인자만 바꿉니다.

```bash
ros2 launch vr_udp_bridge vr_piper_joint_ik_axis_gripper_teleop.launch.py \
  left_can_port:=can0 \
  right_can_port:=can1 \
  controller_axis_map:=z,-x,y \
  debug_axis:=true
```

## 튜닝 파라미터

팔 IK:

- `max_joint_step_rad`: 한 주기당 joint 최대 이동량입니다. 작을수록 부드럽고 느립니다.
- `joint_continuity_weight`: 이전 joint 자세와 가까운 IK 해를 선호하는 강도입니다.
- `ik_position_weight`: 위치 목표 추종 강도입니다.
- `ik_orientation_weight`: 자세 목표 추종 강도입니다.
- `position_scale`: VR 위치 변화량을 로봇 이동량에 반영하는 비율입니다.
- `orientation_scale`: VR 회전 변화량을 로봇 자세에 반영하는 비율입니다.

그리퍼:

- `gripper_axis_index`: 그리퍼 analog 제어에 사용할 joy axis index입니다.
- `gripper_axis_deadzone`: 0 근처 입력 무시 범위입니다.
- `gripper_max_step_m`: 한 publish 주기당 gripper 최대 변화량입니다.
- `gripper_smoothing_alpha`: 그리퍼 속도 명령 smoothing 값입니다.
- `gripper_open_position_m`: 완전 열림 위치입니다. 현재 기본 `0.10m`입니다.
- `gripper_closed_position_m`: 완전 닫힘 위치입니다.
- `gripper_full_open_button_index`: 완전 열림 버튼입니다. 끄려면 `-1`입니다.
- `gripper_effort`: 그리퍼 힘입니다. Piper 쪽에서 보통 `0.5~3.0` 범위로 사용합니다.
- `home_button_index`: 홈 이동에 사용할 joy button index입니다. 끄려면 `-1`입니다.
- `home_both_arms`: true이면 한쪽 컨트롤러 home 버튼으로 양쪽 Piper가 홈 방향으로 이동합니다.
- `home_max_step_rad`: 한 publish 주기당 홈 방향으로 이동할 최대 joint step입니다.

그리퍼를 더 천천히:

```bash
gripper_max_step_m:=0.002
```

그리퍼를 더 강하게:

```bash
gripper_effort:=2.5
```

최대 힘:

```bash
gripper_effort:=3.0
```

0 근처에서 떨리면:

```bash
gripper_axis_deadzone:=0.05
```

완전 열림 버튼을 끄려면:

```bash
gripper_full_open_button_index:=-1
```

Home 버튼을 바꾸려면:

```bash
home_button_index:=7
```

Home 버튼 기능을 끄려면:

```bash
home_button_index:=-1
```

홈 이동을 더 느리게 하려면:

```bash
home_max_step_rad:=0.01
```

## 토픽 확인

VR 입력:

```bash
ros2 topic echo /vr/left_controller/pose
ros2 topic echo /vr/left_controller/joy
```

Piper 피드백:

```bash
ros2 topic echo /left/end_pose
ros2 topic echo /left/joint_states_single
ros2 topic echo /right/joint_states_single
```

명령 확인:

```bash
ros2 topic echo /left/joint_ctrl_single
ros2 topic echo /right/joint_ctrl_single
```

`joint_ctrl_single.position[6]`이 gripper `joint7` 값이고, `effort[6]`이 `gripper_effort` 값입니다.

## 기존 실행 방식

그리퍼 analog 제어가 필요 없고 직접 IK만 확인하려면:

```bash
ros2 launch vr_udp_bridge vr_piper_joint_ik_teleop.launch.py \
  left_can_port:=can0 \
  right_can_port:=can1 \
  max_joint_step_rad:=0.03 \
  joint_continuity_weight:=0.35 \
  debug_axis:=true
```

Piper 내부 Cartesian IK를 사용하는 예전 PosCmd 방식:

```bash
ros2 launch vr_udp_bridge vr_piper_anchor_smooth_teleop.launch.py \
  left_can_port:=can0 \
  right_can_port:=can1 \
  controller_axis_map:=-z,-x,y \
  position_scale:=0.8 \
  orientation_scale:=0.9 \
  debug_axis:=true
```

이전 방식은 구조가 단순하지만 특정 자세에서 Piper 내부 IK가 다른 branch 해를 선택하면 로봇이 갑자기 움직일 수 있습니다. 실제 운용은 `vr_piper_joint_ik_axis_gripper_teleop.launch.py`를 메인으로 사용합니다.

## 문제 해결

빌드 캐시 경로 오류:

```bash
rm -rf build install log
colcon build
source install/setup.bash
```

VR pose는 들어오는데 팔이 움직이지 않는 경우:

- `enable_button_index` 버튼이 실제로 눌리는지 `/vr/left_controller/joy`에서 확인합니다.
- `/left/end_pose`, `/right/end_pose`가 publish되는지 확인합니다.
- `/left/joint_states_single`, `/right/joint_states_single`가 publish되는지 확인합니다.
- `tracking_valid`가 false로 들어오지 않는지 확인합니다.

그리퍼가 움직이지 않는 경우:

- `/vr/left_controller/joy`에서 `axes[1]` 값이 변하는지 확인합니다.
- `/left/joint_ctrl_single`에서 `position[6]` 값이 변하는지 확인합니다.
- `gripper_axis_deadzone`이 너무 크지 않은지 확인합니다.
- `gripper_full_open_button_index`가 joy button index와 맞는지 확인합니다.

부드러운 물체를 잡을 때 더 닫히지 않는 경우:

- `gripper_effort`를 올립니다.
- 예: `gripper_effort:=2.5` 또는 `gripper_effort:=3.0`

팔 움직임이 튀는 경우:

```bash
max_joint_step_rad:=0.02
joint_continuity_weight:=0.6
ik_orientation_weight:=0.3
```

팔 반응이 너무 느린 경우:

```bash
max_joint_step_rad:=0.05
joint_continuity_weight:=0.25
```

## 안전 주의

- 실제 로봇에서는 낮은 `position_scale`, `orientation_scale`, `max_joint_step_rad`에서 시작합니다.
- 그리퍼 힘은 낮은 값에서 시작해 필요할 때만 올립니다.
- 로봇 주변 작업 공간을 비우고 emergency stop을 준비합니다.
- `debug_axis:=true`로 한 축씩 움직이며 방향을 확인한 뒤 scale을 올립니다.
- 직접 IK 방식은 collision avoidance를 수행하지 않습니다.
- joint limit은 URDF/MoveIt 설정을 사용하지만 주변 물체와의 충돌은 별도로 막지 않습니다.
