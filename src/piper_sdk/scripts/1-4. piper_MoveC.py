#!/usr/bin/env python3
# -*-coding:utf8-*-

from typing import Optional
import time, math
from piper_sdk import *

class Move_C:
    def __init__(self, piper: C_PiperInterface):
        self.piper = piper
        self.factor = 1000  # 0.001 mm와 0.001 도를 기준으로 int값을 넘겨주기 위한 설정 (단위 변환용)

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

    def convert(self, position):
        """
        현재 로봇팔의 위치(mm, deg)와 그리퍼 값(mm)을 변환하여 반환합니다.
        """
        return [round(pose * self.factor) for pose in position[:6]] + [round(position[6] * self.factor)]

    def move(self):
        """
        로봇팔이 출발점, 경유점, 도착점에 맞추어 곡선을 그리도록 하는 함수입니다.
        """

        positions = [
            [60, 0, 250, 0, 85, 0, 0x01], # 출발점
            [200, 0, 400, 0, 85, 0, 0x02], # 경유점
            [60, 0, 550, 0, 85, 0, 0x03], # 도착점
        ]

        self.piper.MotionCtrl_2(0x01, 0x03, 50, 0x00)
        for position in positions:
            converted_values = self.convert(position)  # 위치 변환
            self.piper.EndPoseCtrl(*converted_values[:-1])
            self.piper.MoveCAxisUpdateCtrl(position[-1])
            time.sleep(0.001)

def main():
    piper = C_PiperInterface("can0")  # piper 인터페이스 생성
    piper.ConnectPort()  # 포트 연결

    movec = Move_C(piper)
    movec.enable()
    movec.move()

if __name__ == "__main__":
    main()
