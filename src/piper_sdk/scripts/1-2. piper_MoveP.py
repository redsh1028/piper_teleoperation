#!/usr/bin/env python3
# -*-coding:utf8-*-

from typing import Optional
import time, math
from piper_sdk import *

class Move_P:
    def __init__(self, piper: C_PiperInterface):
        self.piper = piper
        self.factor = 1000  # 0.001 mm와 0.001 도를 기준으로 int값을 넘겨주기 때문에 설정 (단위 변환용)
        self.position = [0, 0, 0, 0, 0, 0, 0]  # 로봇팔의 초기 위치 (x, y, z, rx, ry, rz, gripper)
        self.gripper = 0        
        self.count = 0

    def enable(self):
        """
        로봇팔을 활성화하고, 모터가 준비될 때까지 대기하는 함수입니다.
        """
        enable_flag = False
        elapsed_time_flag = False
        timeout = 5  # 5초의 타임아웃 설정
        start_time = time.time()
        
        # 모터가 준비될 때까지 대기
        while not enable_flag:
            elapsed_time = time.time() - start_time
            enable_flag = all(
                getattr(self.piper.GetArmLowSpdInfoMsgs(), f"motor_{i}").foc_status.driver_enable_status 
                for i in range(1, 7)  # 6개의 모터 상태 확인
            )
            self.piper.EnableArm(7)  # 로봇 팔 활성화
            self.piper.GripperCtrl(0, 1000, 0x01, 0)  # 그리퍼 초기화
            
            if elapsed_time > timeout:  # 타임아웃 초과 시 종료
                print("시간초과")
                elapsed_time_flag = True
                enable_flag = True
                break
            time.sleep(1)

        if elapsed_time_flag:
            print("프로그램을 종료합니다")
            exit(0)

    def convert(self):
        """
        현재 로봇팔의 위치(mm, deg)와 그리퍼 값(mm)을 변환하여 반환합니다.
        """
        return [round(pose * self.factor) for pose in self.position] + [round(self.gripper * self.factor)]

    def move(self):
        """
        로봇팔이 설정된 위치로 순차적으로 이동하도록 하는 함수입니다.
        """
        while True:
            print(self.piper.GetArmEndPoseMsgs())  # 현재 로봇팔 포지션 출력
            self.count += 1
            print(self.count)
            
            # 웨이포인트 4개의 위치를 지정하고, 그리퍼의 상태를 변경
            if self.count == 1:
                print("웨이포인트 1")
                self.position = [60, 0, 250, 0, 85, 0]
                self.gripper = 0

            elif self.count == 500:
                print("웨이포인트 2")
                self.position = [210, 0, 250, 0, 85, 0]
                self.gripper = 70

            elif self.count == 1000:
                print("웨이포인트 3")
                self.position = [210, 0, 400, 0, 85, 0]
                self.gripper = 0

            elif self.count == 1500:
                print("웨이포인트 4")
                self.position = [60, 0, 400, 0, 85, 0]   
                self.gripper = 70 
            
            elif self.count == 2000:   
                self.count = 0  # 카운트를 리셋하여 반복

            # 변환된 값을 얻기 위해 convert 메서드 호출
            converted_values = self.convert()  # 위치 및 그리퍼 값 변환
            position_values = converted_values[:6]  # 위치 값 (x, y, z, rx, ry, rz)
            gripper_value = converted_values[6]  # 그리퍼 값
            print(converted_values)

            # 로봇팔 제어 명령
            self.piper.MotionCtrl_2(0x01, 0x00, 50, 0x00)  # 모션 제어 (속도 조절)
            self.piper.EndPoseCtrl(*position_values)  # 위치값을 바탕으로 로봇팔 이동
            self.piper.GripperCtrl(gripper_value, 1000, 0x01, 0)  # 그리퍼 제어
            time.sleep(0.005)

def main():
    piper = C_PiperInterface("can0")  # piper 인터페이스 생성
    piper.ConnectPort()  # 포트 연결

    move_p = Move_P(piper)  # Move_P 객체 생성
    move_p.enable()  # 로봇팔 활성화
    move_p.move()  # 이동 시작

if __name__ == "__main__":
    main()
