#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import sys  # 시스템 관련 기능을 제공하는 라이브러리
import termios  # 터미널 설정을 조작하는 라이브러리
import tty  # 터미널의 특수 입력 모드를 다루는 라이브러리
import select  # 비동기식 입출력을 처리하는 라이브러리
import time  # 시간 관련 기능을 제공하는 라이브러리
import rclpy  # ROS2 Python 라이브러리
from rclpy.node import Node  # ROS2 노드 객체를 사용하기 위한 모듈
from sensor_msgs.msg import JointState  # 관절 상태를 나타내는 메시지 타입
from std_msgs.msg import Bool  # 불리언 값 메시지 타입
import numpy as np  # radian 변환을 위해 numpy 사용

# 키보드 조작 안내 메시지
msg = """
ROS2 Teleop Keyboard Controller
---------------------------
이동 옵션 (관절 각도 제어 [j1, j2, j3, j4, j5, j6]):
    Q - 관절 1 오른쪽으로 (j1+)
    A - 관절 1 왼쪽으로 (j1-)
    W - 관절 2 위로 (j2+)
    S - 관절 2 아래로 (j2-)
    E - 관절 3 아래로 (j3+)
    D - 관절 3 위로 (j3-)
    T - 관절 4 오른쪽으로 (j4+)
    G - 관절 4 왼쪽으로 (j4-)
    Y - 관절 5 아래로 (j5+)
    H - 관절 5 위로 (j5-)
    U - 관절 6 오른쪽으로 (j6+)
    J - 관절 6 왼쪽으로 (j6-)

그리퍼 제어:
    Enter - 열기/닫기 토글

기타:
    Space - 초기 자세로 이동
    Esc - 종료
"""


class WegoPublisher(Node):
    """
    ROS2의 Node 클래스를 상속하여 키보드 입력을 통한 로봇 관절 제어를 수행하는 클래스
    """

    def __init__(self):
        super().__init__("wego_pub_keyboard_node")  # ROS2 노드 초기화
        print(msg)  # 조작 안내 메시지 출력

        # 터미널 입력 설정 변경 (원시 모드로 설정하여 키 입력을 바로 읽음)
        self.settings = termios.tcgetattr(sys.stdin)  # 기존 터미널 설정을 가져옴
        tty.setraw(sys.stdin.fileno())  # 터미널을 원시 모드로 설정하여 키 입력을 실시간으로 처리

        # ROS2 퍼블리셔 설정
        self.joint_pub = self.create_publisher(JointState, "joint_states", 1)  # 조인트 상태 퍼블리셔
        self.init_pos_pub = self.create_publisher(Bool, "init_pos", 1)  # 초기 자세 퍼블리셔
        self.gripper_pub = self.create_publisher(Bool, "gripper_ctrl", 1)  # 그리퍼 제어 퍼블리셔

        # 초기 조인트 상태 설정 (모든 조인트의 초기 위치는 0)
        self.msg = JointState()
        self.msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]  # 조인트 이름
        self.msg.position = [0.0] * 7  # 조인트 초기 위치 설정 (모두 0)
        self.step = 0.1  # 조인트 이동 단위 설정 (0.1 단위로 이동)

        self.gripper_flag = True  # 그리퍼 상태 초기값 (True: 닫힘)

    def get_key(self):
        """
        키보드 입력을 감지하는 함수
        select.select()를 사용하여 비동기적으로 키 입력을 감지합니다.
        """
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)  # 0.05초 동안 입력을 감지
        if rlist:
            return sys.stdin.read(1)  # 입력된 키 반환
        return ""  # 입력이 없으면 빈 문자열 반환

    def run(self):
        """
        키보드 입력을 받아 조인트를 제어하는 루프 실행
        - 키 입력을 감지하여 해당 조작을 수행하고, 로봇의 상태를 업데이트합니다.
        """
        try:
            while rclpy.ok():  # ROS2가 실행 중일 때 루프 유지
                key = self.get_key()  # 키보드 입력을 가져옵니다.
                if key:
                    sys.stdout.flush()  # 출력 버퍼를 플러시하여 즉시 화면에 표시

                    if key == "\x1b":  # ESC 키 입력 시 종료
                        print("\r종료 중...", flush=True)
                        break

                    elif key == "\r":  # Enter 키 입력 시 그리퍼 상태 변경
                        self.gripper_flag = not self.gripper_flag  # 그리퍼 상태 토글
                        print(f"\r그리퍼 상태: {'닫힘' if self.gripper_flag else '열림'}", flush=True)
                        self.gripper_pub.publish(Bool(data=self.gripper_flag))  # 그리퍼 상태 퍼블리시

                    elif key == " ":  # Space 키 입력 시 초기 자세로 이동
                        print("\r초기 자세로 이동", flush=True)
                        self.init_pos_pub.publish(Bool(data=True))  # 초기 자세 이동을 위한 메시지 퍼블리시
                        self.msg.position = [0.0] * 7  # 조인트 각도를 초기화

                    # 핵심 부분: 아래는 조인트 제어 키 입력 처리
                    elif key in "qawsedrftgyhuj":  # 조인트 제어 키 입력
                        """
                        각 조인트의 제어 키 입력을 처리하는 부분입니다.
                        사용자 입력에 따라 조인트가 이동하도록 설정합니다.
                        조인트 1: +- 154, 조인트 2: 0-195, 조인트3: -175-0, 조인트 4: -106-106, 조인트5: -75-75, 조인트 6: +-100 이게 조인트 한계야. 넘어가면 더 그냥 그 한계값으로 만들자.
                        """
                        direction = {
                            "q": (0, 1),  # 관절 1 (j1) 오른쪽으로 이동 (j1+)
                            "a": (0, -1),  # 관절 1 (j1) 왼쪽으로 이동 (j1-)
                            "w": (1, 1),  # 관절 2 (j2) 위로 이동 (j2+)
                            "s": (1, -1),  # 관절 2 (j2) 아래로 이동 (j2-)
                            "e": (2, 1),  # 관절 3 (j3) 아래로 이동 (j3+)
                            "d": (2, -1),  # 관절 3 (j3) 위로 이동 (j3-)
                            "r": (3, 1),  # 관절 4 (j4) 오른쪽으로 이동 (j4+)
                            "f": (3, -1),  # 관절 4 (j4) 왼쪽으로 이동 (j4-)
                            "t": (4, 1),  # 관절 5 (j5) 아래로 이동 (j5+)
                            "g": (4, -1),  # 관절 5 (j5) 위로 이동 (j5-)
                            "y": (5, 1),  # 관절 6 (j6) 오른쪽으로 이동 (j6+)
                            "h": (5, -1),  # 관절 6 (j6) 왼쪽으로 이동 (j6-)
                        }
                        # 조인트 한계 설정 (각 조인트의 최소, 최대 값) - radian 단위로 변환
                        joint_limits = [
                            (np.radians(-154), np.radians(154)),  # 조인트 1
                            (np.radians(0), np.radians(195)),  # 조인트 2
                            (np.radians(-175), np.radians(0)),  # 조인트 3
                            (np.radians(-106), np.radians(106)),  # 조인트 4
                            (np.radians(-75), np.radians(75)),  # 조인트 5
                            (np.radians(-100), np.radians(100)),  # 조인트 6
                        ]

                        # 입력된 키에 대해 해당하는 조인트 인덱스와 방향을 찾습니다.
                        idx, sign = direction[key]  # 'key'에 해당하는 조인트 번호와 이동 방향(1 또는 -1)을 설정

                        # 조인트 위치를 변경합니다.
                        # 현재 조인트 위치에 'sign' 값(이동 방향)을 'step' 만큼 더합니다.
                        new_position = self.msg.position[idx] + (sign * self.step)

                        # 조인트 한계를 초과하지 않도록 제한
                        min_limit, max_limit = joint_limits[idx]
                        if min_limit <= new_position <= max_limit:
                            self.msg.position[idx] = new_position
                            self.joint_pub.publish(self.msg)
                            print(f"\r{self.msg.name[idx]}: {self.msg.position[idx]:.2f}", flush=True)
                        else:
                            print(f"\r{self.msg.name[idx]} 한계 도달: {self.msg.position[idx]:.2f}", flush=True)

                        self.joint_pub.publish(self.msg)  # 변경된 조인트 상태를 퍼블리시
                        print(f"\r{self.msg.name[idx]}: {self.msg.position[idx]:.2f}", flush=True)

                time.sleep(0.05)  # 0.05초 대기하여 입력을 안정적으로 받음
        except Exception as e:
            print(f"\r오류 발생: {e}", flush=True)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)  # 터미널 설정 복원
            print("\r텔레오프 키보드 컨트롤러 종료", flush=True)
            rclpy.shutdown()  # ROS2 종료


def main(args=None):
    """
    ROS2 노드를 실행하는 메인 함수
    """
    rclpy.init(args=args)  # ROS2 초기화
    node = WegoPublisher()  # 노드 생성
    node.run()  # 키보드 입력 루프 시작
    node.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS2 종료


if __name__ == "__main__":
    main()  # 스크립트 실행 시 main() 함수 호출
