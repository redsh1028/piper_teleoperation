#!/usr/bin/env python3
# -*-coding:utf8-*-

from typing import Optional
import time, math
from piper_sdk import *

class Move_L:
    def __init__(self, piper: C_PiperInterface):
        self.piper = piper
        self.factor = 1000  # 0.001 mm와 0.001 도를 기준으로 int값을 넘겨주기 위한 설정 (단위 변환용)
        self.position = [0, 0, 0, 0, 0, 0, 0]  # 초기 위치 설정
        self.count = 0  # 카운트 초기화

    def enable(self):
        """
        로봇팔을 활성화하고, 모터가 준비될 때까지 대기하는 함수입니다.
        """
        enable_flag = False
        elapsed_time_flag = False
        timeout = 5
        start_time = time.time()

        while not enable_flag:  # 로봇 팔이 활성화 될 때까지 반복
            elapsed_time = time.time() - start_time
            enable_flag = all(
                getattr(self.piper.GetArmLowSpdInfoMsgs(), f"motor_{i}").foc_status.driver_enable_status
                for i in range(1, 7)  # 6개의 모터가 모두 활성화되어야 함
            )
            self.piper.EnableArm(7)  # 팔 활성화
            self.piper.GripperCtrl(0, 1000, 0x01, 0)  # 그리퍼 활성화

            if elapsed_time > timeout:  # 시간 초과 시
                print("시간초과")
                elapsed_time_flag = True
                enable_flag = True
                break
            time.sleep(1)

        if elapsed_time_flag:  # 시간 초과 시
            print("프로그램을 종료합니다")
            exit(0)  # 프로그램 종료

    def convert(self, position):
        """
        현재 로봇팔의 위치(mm, deg)와 그리퍼 값(mm)을 변환하여 반환합니다.
        """
        return [round(pose * self.factor) for pose in position[:6]] + [round(position[6] * self.factor)]

    def wait_for_motion_complete(self, *target):
        """
        로봇 팔의 이동이 완료될 때까지 기다리는 함수입니다.
        """
        while True:
            current_state = self.piper.GetArmEndPoseMsgs().end_pose
            current_pose = [
                current_state.X_axis, current_state.Y_axis, current_state.Z_axis,
                current_state.RX_axis, current_state.RY_axis, current_state.RZ_axis
            ]
            # 현재 위치와 목표 위치가 같으면 이동 완료
            if all(abs(current_pose[i] - target[i]) < self.factor for i in range(6)):
                print("포즈 목표 위치 도달")
                break
            time.sleep(0.05)  # 50ms 대기

    def move(self, positions):
        """
        로봇팔이 설정된 위치로 순차적으로 이동하도록 하는 함수입니다.
        """
        for position in positions:
            converted_values = self.convert(position)  # 위치 변환
            position_values = converted_values[:6]  # 위치 값 (x, y, z, rx, ry, rz)
            gripper_value = converted_values[6]  # 그리퍼 값
            print(self.piper.GetArmEndPoseMsgs())  # 현재 로봇팔 포지션 출력
            
            self.piper.MotionCtrl_2(0x01, 0x02, 50, 0x00)  # 선형 보간 모드로 전환
            self.piper.EndPoseCtrl(*position_values)
            self.piper.GripperCtrl(gripper_value, 1000, 0x01, 0)
            self.wait_for_motion_complete(*position_values)
            time.sleep(0.005)

def main():
    piper = C_PiperInterface("can0")  # piper 인터페이스 생성
    piper.ConnectPort()  # 포트 연결

    move_l = Move_L(piper) # Move_L 객체 생성
    move_l.enable() # 로봇팔 활성화

    positions = [     # 이동할 목표 위치
        [60, 0, 250, 0, 85, 0, 00], # 웨이포인트 1
        [210, 0, 250, 0, 85, 0, 70], # 웨이포인트 2
        [210, 0, 400, 0, 85, 0, 0], # 웨이포인트 3
        [60, 0, 400, 0, 85, 0, 70], # 웨이포인트 4
    ]
    move_l.move(positions) # 이동 시작

if __name__ == "__main__":
    main()
