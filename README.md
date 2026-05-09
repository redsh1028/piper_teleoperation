# piper_teleoperation

Piper arm 2대를 VR 컨트롤러로 텔레오퍼레이션하기 위한 ROS2 Humble 워크스페이스입니다.

이 프로젝트는 OpenVR/SteamVR에서 들어오는 VR 컨트롤러 pose와 버튼 정보를 UDP로 받아 ROS2 토픽으로 변환하고, 각 컨트롤러의 상대 움직임을 좌/우 Piper 로봇의 목표 자세로 변환합니다. 최종 제어는 Piper 내부 Cartesian IK에 맡기는 방식과, 이 프로젝트에서 직접 IK를 풀어 joint command를 publish하는 방식이 모두 포함되어 있습니다.

현재 권장 실행 방식은 `vr_piper_joint_ik_teleop.launch.py`입니다. 이 방식은 `/left/pos_cmd`, `/right/pos_cmd` 대신 `/left/joint_ctrl_single`, `/right/joint_ctrl_single`로 직접 joint 명령을 보내며, IK 해를 찾을 때 이전 joint 자세와 가장 가까운 해를 우선하도록 설계되어 특정 자세에서 로봇이 갑자기 크게 움직이는 branch jump를 줄입니다.

## 구성

```text
tele_ws/
├── send_openvr_udp.py
├── can_config.sh
├── find_all_can_port.sh
├── src/
│   ├── piper_ros/
│   │   └── src/piper/launch/start_dual_piper.launch.py
│   ├── piper_sdk/
│   └── vr_udp_bridge/
│       ├── launch/
│       │   ├── vr_piper_joint_ik_teleop.launch.py
│       │   ├── vr_piper_anchor_smooth_teleop.launch.py
│       │   └── vr_piper_anchor_teleop.launch.py
│       └── vr_udp_bridge/
│           ├── udp_to_pose.py
│           ├── vr_piper_joint_ik_teleop.py
│           └── vr_piper_anchor_teleop.py
```

주요 파일은 다음과 같습니다.

- `send_openvr_udp.py`: SteamVR/OpenVR 컨트롤러 정보를 UDP로 전송합니다.
- `udp_to_pose.py`: UDP JSON 패킷을 ROS2의 `/vr/...` pose, joy, status 토픽으로 변환합니다.
- `vr_piper_joint_ik_teleop.py`: VR 컨트롤러의 상대 motion을 Piper 목표 pose로 만들고, 직접 IK를 풀어 joint command를 publish합니다.
- `vr_piper_joint_ik_teleop.launch.py`: 실제 dual Piper 제어용 권장 launch 파일입니다.
- `start_dual_piper.launch.py`: 좌/우 Piper CAN 포트를 각각 namespace `/left`, `/right`로 실행합니다.

## 제어 방식

권장 노드인 `vr_piper_joint_ik_teleop.py`는 절대 VR 좌표를 로봇 좌표로 그대로 믿지 않습니다. 버튼을 누르는 순간을 anchor로 잡고, 그 이후 컨트롤러의 상대 변화량만 로봇 목표 자세에 반영합니다.

동작 흐름:

1. VR 컨트롤러 pose와 joy 입력을 `/vr/left_controller/...`, `/vr/right_controller/...`로 수신합니다.
2. enable 버튼을 누르는 순간 현재 VR controller pose와 현재 Piper end pose, 현재 joint state를 anchor로 저장합니다.
3. 버튼을 누르는 동안 controller anchor 기준 상대 위치/자세 변화량을 계산합니다.
4. `controller_axis_map`으로 VR 축을 로봇 조작 축에 맞춥니다.
5. 목표 end-effector pose를 계산합니다.
6. URDF의 `base_link -> gripper_base` 체인으로 FK/IK를 계산합니다.
7. `scipy.optimize.least_squares`로 IK를 풀되, 이전 joint command와 가까운 해를 비용 함수에 포함합니다.
8. 한 주기당 joint 변화량을 `max_joint_step_rad`로 제한한 뒤 `/left/joint_ctrl_single`, `/right/joint_ctrl_single`에 publish합니다.

즉, 목표 pose를 완벽히 맞추는 것보다 joint 움직임의 연속성과 안정성을 더 중요하게 둡니다.

## 주요 토픽

VR 입력:

```text
/vr/left_controller/pose
/vr/right_controller/pose
/vr/left_controller/joy
/vr/right_controller/joy
/vr/left_controller/status
/vr/right_controller/status
```

Piper 피드백:

```text
/left/end_pose
/right/end_pose
/left/joint_states_single
/right/joint_states_single
```

Piper 명령:

```text
/left/joint_ctrl_single
/right/joint_ctrl_single
/left/gripper_ctrl
/right/gripper_ctrl
```

기존 PosCmd 방식 launch를 사용할 경우에는 `/left/pos_cmd`, `/right/pos_cmd`가 사용됩니다.

## 설치 및 빌드

ROS2 Humble 환경을 기준으로 합니다.

```bash
cd ~/tele_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

필요한 Python 패키지가 부족하면 다음을 설치합니다.

```bash
pip3 install -r requirements.txt
```

직접 IK 노드는 다음 패키지를 사용합니다.

```text
numpy
scipy
urdf_parser_py
PyYAML
```

## CAN 포트 설정

두 대의 Piper를 연결한 뒤 CAN 포트를 확인합니다.

```bash
cd ~/tele_ws
bash find_all_can_port.sh
```

예시 출력:

```text
USB 포트 7-1:1.0에 인터페이스 can0를 삽입할 것을 추천합니다.
USB 포트 3-1:1.0에 인터페이스 can1를 삽입할 것을 추천합니다.
```

CAN 인터페이스를 활성화합니다.

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

환경에 맞게 `can_config.sh`를 수정해 자동 설정할 수도 있습니다.

```bash
sudo bash can_config.sh
```

## VR 데이터 송신

`send_openvr_udp.py`는 OpenVR을 사용해 SteamVR 컨트롤러 pose, velocity, button, trigger 정보를 UDP로 보냅니다.

Windows/SteamVR 쪽에서 실행하는 예시:

```bash
python3 send_openvr_udp.py --host <ROS_PC_IP> --port 5005 --rate 60
```

WSL2를 사용하는 경우 `--host`에는 WSL2 또는 ROS2가 실행되는 쪽에서 UDP를 받을 수 있는 IP를 넣습니다. 기본 포트는 `5005`입니다.

ROS2 쪽에서는 launch가 `udp_to_pose`를 함께 실행합니다. 별도로 확인하려면:

```bash
ros2 topic echo /vr/left_controller/pose
ros2 topic echo /vr/left_controller/joy
```

## 권장 실행 방법

현재 가장 잘 맞는 설정은 다음입니다.

```bash
cd ~/tele_ws
source install/setup.bash

ros2 launch vr_udp_bridge vr_piper_joint_ik_teleop.launch.py \
  left_can_port:=can0 \
  right_can_port:=can1 \
  max_joint_step_rad:=0.03 \
  joint_continuity_weight:=0.35 \
  debug_axis:=true
```

launch 기본값에는 이미 현재 맞는 좌표 설정이 들어 있습니다.

```text
controller_axis_map=-z,-x,y
position_scale=0.8
orientation_scale=0.9
```

처음 실제 로봇에서 테스트할 때는 더 보수적으로 시작하는 것을 권장합니다.

```bash
ros2 launch vr_udp_bridge vr_piper_joint_ik_teleop.launch.py \
  left_can_port:=can0 \
  right_can_port:=can1 \
  position_scale:=0.4 \
  orientation_scale:=0.4 \
  max_joint_step_rad:=0.02 \
  joint_continuity_weight:=0.5 \
  debug_axis:=true
```

## 조작 방법

기본 설정:

- `enable_button_index:=0`: 버튼 0을 누르는 동안만 로봇이 움직입니다.
- 버튼을 누른 순간 현재 컨트롤러 pose와 현재 로봇 EE pose가 anchor로 저장됩니다.
- 버튼을 누른 채 컨트롤러를 움직이면 anchor 기준 변화량만 로봇에 반영됩니다.
- 버튼을 떼면 anchor가 해제되고 joint command publish가 멈춥니다.
- 다시 누르면 현재 위치에서 새 anchor가 잡힙니다.
- `gripper_toggle_button_index:=1`: 버튼 1로 그리퍼 toggle 명령을 보냅니다.

이 방식에서는 컨트롤러를 로봇 앞에서 잡든 옆에서 잡든, 버튼을 누른 순간을 기준으로 상대 움직임이 계산됩니다.

## 좌표계 설정

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

축 방향이 반대로 느껴지면 코드를 수정하지 않고 launch 인자만 바꿔 테스트합니다.

```bash
ros2 launch vr_udp_bridge vr_piper_joint_ik_teleop.launch.py \
  left_can_port:=can0 \
  right_can_port:=can1 \
  controller_axis_map:=z,-x,y \
  debug_axis:=true
```

`debug_axis:=true`를 켜면 컨트롤러 변화량과 로봇 기준 변화량이 로그로 출력되어 축 확인이 쉽습니다.

## IK 튜닝 파라미터

`max_joint_step_rad`:

- 한 제어 주기에서 각 joint가 움직일 수 있는 최대 radian입니다.
- 작을수록 부드럽고 안전하지만 반응이 느립니다.
- 추천 시작값: `0.02` ~ `0.03`
- 빠르게 움직이고 싶으면 `0.05` 근처까지 올립니다.

`joint_continuity_weight`:

- 이전 joint 자세와 가까운 해를 선호하는 강도입니다.
- 클수록 branch jump가 줄어들지만 목표 pose 추종 정확도가 낮아질 수 있습니다.
- 추천값: `0.25` ~ `0.6`

`ik_position_weight`:

- 위치 목표를 맞추는 강도입니다.
- 위치를 더 잘 따라오게 하려면 올립니다.

`ik_orientation_weight`:

- 자세 목표를 맞추는 강도입니다.
- 자세가 너무 민감하면 낮추고, 자세 추종이 약하면 올립니다.

`orientation_scale`:

- VR 컨트롤러 회전 변화량을 로봇 EE 자세 변화량에 얼마나 반영할지 결정합니다.
- 너무 예민하면 `0.3` ~ `0.6`으로 낮춥니다.

`position_scale`:

- VR 컨트롤러 위치 변화량을 로봇 이동량에 얼마나 반영할지 결정합니다.
- 실제 로봇에서는 낮은 값에서 시작한 뒤 점진적으로 올립니다.

## 자주 쓰는 실행 예시

안전 확인용:

```bash
ros2 launch vr_udp_bridge vr_piper_joint_ik_teleop.launch.py \
  left_can_port:=can0 \
  right_can_port:=can1 \
  position_scale:=0.3 \
  orientation_scale:=0.3 \
  max_joint_step_rad:=0.02 \
  joint_continuity_weight:=0.6 \
  debug_axis:=true
```

현재 주 사용 설정:

```bash
ros2 launch vr_udp_bridge vr_piper_joint_ik_teleop.launch.py \
  left_can_port:=can0 \
  right_can_port:=can1 \
  max_joint_step_rad:=0.03 \
  joint_continuity_weight:=0.35 \
  debug_axis:=true
```

자세를 고정하고 위치만 테스트:

```bash
ros2 launch vr_udp_bridge vr_piper_joint_ik_teleop.launch.py \
  left_can_port:=can0 \
  right_can_port:=can1 \
  lock_orientation:=true \
  max_joint_step_rad:=0.03 \
  debug_axis:=true
```

## 동작 확인

VR 입력 확인:

```bash
ros2 topic echo /vr/left_controller/pose
ros2 topic echo /vr/right_controller/joy
```

Piper 피드백 확인:

```bash
ros2 topic echo /left/end_pose
ros2 topic echo /left/joint_states_single
```

joint command 확인:

```bash
ros2 topic echo /left/joint_ctrl_single
ros2 topic echo /right/joint_ctrl_single
```

버튼을 누르지 않았을 때는 joint command가 새로 publish되지 않아야 합니다. 버튼을 누르면 anchor reset 로그가 나오고 joint command가 publish됩니다.

## 기존 PosCmd 방식

기존 방식은 Piper ROS 노드의 Cartesian command 토픽으로 `PosCmd`를 보내고, Piper 내부 IK가 해를 선택합니다.

```bash
ros2 launch vr_udp_bridge vr_piper_anchor_smooth_teleop.launch.py \
  left_can_port:=can0 \
  right_can_port:=can1 \
  controller_axis_map:=-z,-x,y \
  position_scale:=0.8 \
  orientation_scale:=0.9 \
  debug_axis:=true
```

이 방식은 구조가 단순하고 반응이 빠르지만, 특정 자세에서 Piper 내부 IK가 다른 branch 해를 선택하면 로봇이 갑자기 크게 움직일 수 있습니다. 이 문제를 줄이기 위해 직접 IK 방식인 `vr_piper_joint_ik_teleop.launch.py`를 권장합니다.

## 문제 해결

`CMakeCache.txt directory is different` 오류:

워크스페이스 경로가 바뀐 상태에서 기존 build 캐시가 남아 있을 때 발생합니다.

```bash
rm -rf build install log
colcon build
```

`mujoco_model` 또는 설치 파일을 찾을 수 없는 오류:

패키지 내부 install 경로와 실제 디렉터리 이름이 다를 때 발생할 수 있습니다. 현재 워크스페이스에서는 `piper_description/mujoco` 경로를 사용합니다.

VR pose는 들어오는데 로봇이 움직이지 않는 경우:

```bash
ros2 topic echo /vr/left_controller/joy
ros2 topic echo /left/end_pose
ros2 topic echo /left/joint_states_single
```

확인할 것:

- enable 버튼 index가 맞는지 확인합니다.
- `/left/end_pose`, `/right/end_pose`가 publish되는지 확인합니다.
- `/left/joint_states_single`, `/right/joint_states_single`가 publish되는지 확인합니다.
- `tracking_valid`가 false로 들어오지 않는지 확인합니다.

방향이 반대인 경우:

`controller_axis_map`만 바꿔서 테스트합니다.

```bash
controller_axis_map:=z,-x,y
controller_axis_map:=-z,x,y
controller_axis_map:=-z,-x,-y
```

움직임이 튀는 경우:

```bash
max_joint_step_rad:=0.02
joint_continuity_weight:=0.6
ik_orientation_weight:=0.3
```

반응이 너무 느린 경우:

```bash
max_joint_step_rad:=0.05
joint_continuity_weight:=0.25
```

## 안전 주의

- 실제 로봇에서는 항상 낮은 `position_scale`, `orientation_scale`, `max_joint_step_rad`에서 시작합니다.
- 로봇 주변 작업 공간을 비우고 emergency stop을 준비합니다.
- `debug_axis:=true`로 한 축씩 움직이며 방향을 확인한 뒤 scale을 올립니다.
- 이 직접 IK 방식은 collision avoidance를 수행하지 않습니다.
- joint limit은 URDF/MoveIt 설정을 사용하지만, 주변 물체와의 충돌은 별도로 막지 않습니다.
