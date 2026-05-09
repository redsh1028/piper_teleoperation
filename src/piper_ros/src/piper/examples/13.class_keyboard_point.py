#!/usr/bin/env python3

# 필요한 라이브러리 임포트
import sys
import termios
import tty
import select
import time
import rclpy
from rclpy.node import Node
from piper_msgs.msg import PosCmd  # PosCmd 메시지 임포트 (로봇의 위치 및 명령 전달용)
from std_msgs.msg import Bool  # Bool 메시지 임포트 (그리퍼 제어용)

# 텔레오프 키보드 컨트롤러에 대한 설명 메시지
msg = """
ROS2 Teleop Keyboard Controller (Position Control)
---------------------------------------------------
이동 옵션 (XYZ 위치 조정 및 RPY 회전):
    Q - X 증가 (+)
    A - X 감소 (-)
    W - Y 증가 (+)
    S - Y 감소 (-)
    E - Z 증가 (+)
    D - Z 감소 (-)
    T - Roll 증가 (+)
    G - Roll 감소 (-)
    Y - Pitch 증가 (+)
    H - Pitch 감소 (-)
    U - Yaw 증가 (+)
    J - Yaw 감소 (-)

그리퍼 제어:
    Enter - 열기/닫기 토글

기타:
    Space - 초기 위치로 이동
    Esc - 종료
"""


# WegoPublisher 클래스 정의 (ROS2 노드)
class WegoPublisher(Node):
    def __init__(self):
        super().__init__("wego_pub_position_node")  # ROS2 노드 생성
        print(msg)  # 초기 설명 메시지 출력

        # 표준 입력을 원시 모드로 설정 (키보드 입력 받기 위해)
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        # 메시지 퍼블리셔들 생성
        self.pub = self.create_publisher(PosCmd, "pos_cmd", 10)  # 위치 명령 퍼블리셔
        self.init_pos_pub = self.create_publisher(Bool, "init_pos", 10)  # 초기 위치 명령 퍼블리셔
        self.gripper_pub = self.create_publisher(Bool, "gripper_ctrl", 10)  # 그리퍼 상태 제어 퍼블리셔

        # 초기 위치 및 상태 설정
        self.msg = PosCmd()
        self.init = [55.0, 0.0, 203.0, 0.0, 90.0, 0.0, 0.0]  # 초기 위치 값 (X, Y, Z, Roll, Pitch, Yaw, Gripper)
        self.mode = [0x01, 0x00]  # 모드 설정 (고정값)

        # 이동/회전 단위 설정
        self.step = 10.0  # 위치 이동 단위 (10mm)
        self.angle_step = 5.0  # 회전 각도 이동 단위 (5도)
        self.gripper_flag = True  # 그리퍼 상태 (True: 열림, False: 닫힘)

    def get_key(self):
        """키 입력을 감지하여 반환하는 함수"""
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            return sys.stdin.read(1)  # 입력된 키 반환
        return ""  # 입력이 없으면 빈 문자열 반환

    def update_and_publish(self):
        """위치와 회전 상태를 업데이트하고 퍼블리시하는 함수"""
        combined = self.init + self.mode  # 초기 위치와 모드를 합침
        self.msg.x, self.msg.y, self.msg.z = combined[0], combined[1], combined[2]
        self.msg.roll, self.msg.pitch, self.msg.yaw = combined[3], combined[4], combined[5]
        self.msg.gripper = combined[6]
        self.msg.mode1, self.msg.mode2 = combined[7], combined[8]

        # 퍼블리셔를 통해 메시지 전송
        self.pub.publish(self.msg)

        # 현재 위치 출력 (실시간 모니터링)
        print(
            f"\r현재 위치: X={self.msg.x:.1f}, Y={self.msg.y:.1f}, Z={self.msg.z:.1f}, Roll={self.msg.roll:.1f}, Pitch={self.msg.pitch:.1f}, Yaw={self.msg.yaw:.1f}",
            flush=True,
        )

    def run(self):
        """메인 루프 함수 - 키 입력을 감지하고 처리"""
        try:
            while rclpy.ok():  # ROS2가 실행 중이면 계속 반복
                key = self.get_key()  # 키 입력 받기
                if key:
                    sys.stdout.flush()
                    if key == "\x1b":  # ESC 키 (프로그램 종료)
                        print("\r종료 중...", flush=True)
                        break

                    elif key == "\r":  # Enter 키 (그리퍼 열기/닫기 토글)
                        self.gripper_flag = not self.gripper_flag
                        print(f"\r그리퍼 상태: {'열림' if self.gripper_flag else '닫힘'}", flush=True)
                        self.gripper_pub.publish(Bool(data=self.gripper_flag))

                    elif key == " ":  # Space 키 (초기 위치로 이동)
                        print("\r초기 위치로 이동", flush=True)
                        self.init_pos_pub.publish(Bool(data=True))
                        self.init = [55.0, 0.0, 203.0, 0.0, 90.0, 0.0, 0.0]
                        self.update_and_publish()

                    elif key in "qawsedrftgyhuj":  # **핵심 부분: 조인트 이동/회전 제어**
                        direction = {
                            "q": (0, 1, self.step),  # X 증가
                            "a": (0, -1, self.step),  # X 감소
                            "w": (1, 1, self.step),  # Y 증가
                            "s": (1, -1, self.step),  # Y 감소
                            "e": (2, 1, self.step),  # Z 증가
                            "d": (2, -1, self.step),  # Z 감소
                            "r": (3, 1, self.angle_step),  # Roll 증가
                            "f": (3, -1, self.angle_step),  # Roll 감소
                            "t": (4, 1, self.angle_step),  # Pitch 증가
                            "g": (4, -1, self.angle_step),  # Pitch 감소
                            "y": (5, 1, self.angle_step),  # Yaw 증가
                            "h": (5, -1, self.angle_step),  # Yaw 감소
                        }
                        # 각 조합에 맞는 조인트 이동/회전 방향을 설정
                        idx, sign, step = direction[key]
                        self.init[idx] += sign * step  # 해당 조인트 값에 이동/회전 단위 적용
                        self.update_and_publish()  # 위치 업데이트 후 퍼블리시

                time.sleep(0.05)  # 50ms 대기

        except Exception as e:  # 오류 처리
            print(f"\r오류 발생: {e}", flush=True)
        finally:
            # 키보드 설정 복구
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\r텔레오프 키보드 컨트롤러 종료", flush=True)
            rclpy.shutdown()  # ROS2 종료


# 메인 함수
def main(args=None):
    rclpy.init(args=args)  # ROS2 초기화
    node = WegoPublisher()  # WegoPublisher 노드 생성
    node.run()  # 텔레오프 실행
    node.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS2 종료


# 프로그램 시작점
if __name__ == "__main__":
    main()
