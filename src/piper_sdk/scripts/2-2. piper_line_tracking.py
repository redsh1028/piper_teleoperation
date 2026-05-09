#!/usr/bin/env python3
# -*-coding:utf8-*-

from typing import Optional
import time
from piper_sdk import *

class Piper_Line_Tracking:
    def __init__(self, piper: C_PiperInterface):
        self.piper = piper
        self.enable()

    def enable(self, timeout=5):
        """로봇팔 활성화 및 준비 완료 대기"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if all(getattr(self.piper.GetArmLowSpdInfoMsgs(), f"motor_{i}").foc_status.driver_enable_status for i in range(1, 7)):
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
                return
            time.sleep(1)
        print("시간초과. 프로그램을 종료합니다.")
        exit(0)

    def wait_for_motion_complete(self, target):
        """목표 위치 도달 대기"""
        while True:
            current_state = self.piper.GetArmEndPoseMsgs().end_pose
            current_pose = [current_state.X_axis, current_state.Y_axis, current_state.Z_axis, current_state.RX_axis, current_state.RY_axis, current_state.RZ_axis]
            if all(abs(current_pose[i] - target[i]) < 1000 for i in range(3)):
                print("포즈 목표 위치 도달")
                break
            time.sleep(0.1)

    def position_to_int(self, position, factor: int = 1000):
        """좌표를 정수로 변환"""
        return [int(val * factor) for val in position]

    def move(self, x, y, z, rx, ry, rz, mode='P'):
        """이동 함수 (P, L 모드 지원)"""
        x, y, z, rx, ry, rz = self.position_to_int([x, y, z, rx, ry, rz])
        mode_dict = {'P': 0x00, 'L': 0x02}
        self.piper.MotionCtrl_2(0x01, mode_dict[mode], 50 if mode == 'P' else 20)
        self.piper.EndPoseCtrl(x, y, z, rx, ry, rz)

    def initialize(self):
        """초기 위치로 이동 및 그리퍼 제어"""
        start_pose = [240, 27, 220, 180, 0, 180]
        if start_pose is None:
            print("캘리브레이션된 포즈 입력 실패")
            return False

        self.move(*start_pose, mode='P')
        self.wait_for_motion_complete(self.position_to_int(start_pose))

        # 그리퍼 조작
        print("\n=== 그리퍼 제어 ===")
        print("Enter를 누르면 그리퍼가 닫히고, 다시 Enter를 누르면 열립니다.")
        print("물체를 원하는 방식으로 잡았다면 'y'를 눌러주세요.")

        gripper_open = False
        while True:
            key = input()
            if key.lower() == 'y':
                print("다음 단계로 진행합니다...")
                break
            elif key.strip() == '':
                self.piper.GripperCtrl(0, 1000, 0x01, 0) if gripper_open else self.piper.GripperCtrl(int(0.8 * 1000 * 1000), 1000, 0x01, 0)
                print("그리퍼 닫힘" if gripper_open else "그리퍼 열림")
                gripper_open = not gripper_open
                time.sleep(0.5)
            else:
                print("잘못된 입력입니다. Enter 또는 'y'를 입력해주세요.")

        return start_pose

    def move_waypoints(self, pose):
        """웨이포인트 이동"""
        waypoints = [
            [240, 27, 220, pose[3], pose[4], pose[5]],
            [145, 20, 217, pose[3], pose[4], pose[5]],
            [142, -15, 217, pose[3], pose[4], pose[5]],
            [223, -7, 220, pose[3], pose[4], pose[5]],
            [203, -45, 218, pose[3], pose[4], pose[5]],
            [173, -25, 217, pose[3], pose[4], pose[5]],
            [122, -67, 217, pose[3], pose[4], pose[5]]
        ]
        for waypoint in waypoints:
            self.move(*waypoint, mode='L')
            self.wait_for_motion_complete(self.position_to_int(waypoint))

        time.sleep(0.005)

def main():
    """메인 실행 함수"""
    piper = C_PiperInterface("can0")  # piper 인터페이스 생성
    piper.ConnectPort()  # 포트 연결

    piper_line_tracking = Piper_Line_Tracking(piper)
    piper_line_tracking.enable()
    pose = piper_line_tracking.initialize()
    if pose:
        piper_line_tracking.move_waypoints(pose)

if __name__ == "__main__":
    main()
