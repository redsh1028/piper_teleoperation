#!/usr/bin/env python3
# -*-coding:utf8-*-

from typing import Optional
import time, math
from piper_sdk import *
from piper_sdk.interface.piper_interface_v2 import C_PiperInterface_V2
import threading

class Move_J:
    def __init__(self, piper: C_PiperInterface_V2):
        self.piper = piper  # piper 인터페이스 객체 초기화
        self.factor = 1000  # 0.001 도를 기준으로 int값을 넘겨주기 때문에 설정 (단위 변환용)
        self.position = [0, 0, 0, 0, 0, 0, 0]  # 초기 위치 (각 관절)
        self.gripper = 0  # 초기 위치 (그리퍼)
        self.count = 0  # 이동 횟수 카운트 초기화

    def enable(self):
        """
        로봇팔을 활성화하고, 모터가 준비될 때까지 대기하는 함수입니다.
        """
        enable_flag = False  # 팔이 활성화되었는지 확인하는 플래그
        timeout = 5  # 타임아웃 시간 (초)
        start_time = time.time()  # 시작 시간 기록

        while not enable_flag:
            elapsed_time = time.time() - start_time  # 경과 시간 계산
            enable_flag = all(
                getattr(self.piper.GetArmLowSpdInfoMsgs(), f"motor_{i}").foc_status.driver_enable_status
                for i in range(1, 7)
            )
            self.piper.EnableArm(7)
            self.piper.GripperCtrl(0, 1000, 0x01, 0)
            
            if elapsed_time > timeout:
                print("시간 초과 - 프로그램 종료")
                exit(0)
            time.sleep(1)

    def convert(self):
        return [round(angle * self.factor) for angle in self.position] + [round(self.gripper * self.factor)]

    def move(self):
        """
        로봇팔이 설정된 위치로 순차적으로 이동하도록 하는 함수입니다.
        """
        while True:
            print(self.piper.GetArmJointMsgs())
            self.count += 1
            print(f"이동 카운트: {self.count}")

            waypoints = [
                ([0, 0, 0, 0, 0, 0], 0),
                ([30, 0, 0, 0, 0, 0], 0),
                ([0, 30, 0, 0, 0, 0], 0),
                ([0, 0, -30, 0, 0, 0], 0),
                ([0, 0, 0, 30, 0, 0], 0),
                ([0, 0, 0, 0, -30, 0], 0),
                ([0, 0, 0, 0, 0, 30], 0),
                ([0, 0, 0, 0, 0, 0], 70),
                ([30, 30, -30, 30, -30, 30], 0)
            ]
            
            if self.count >= len(waypoints) * 100:
                self.count = 0
            
            index = self.count // 100
            self.position, self.gripper = waypoints[index]
            
            converted_values = self.convert()
            self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            self.piper.JointCtrl(*converted_values[:6])
            self.piper.GripperCtrl(abs(converted_values[6]), 1000, 0x01, 0)
            time.sleep(0.005)

# 메인 함수
def main():
    piper_interfaces = [C_PiperInterface(f"can{i}") for i in range(4)]
    
    for piper in piper_interfaces:
        piper.ConnectPort()
    
    move_objects = [Move_J(piper) for piper in piper_interfaces]
    
    for move_obj in move_objects:
        move_obj.enable()
    
    threads = [threading.Thread(target=move_obj.move) for move_obj in move_objects]
    
    for thread in threads:
        thread.start()
    
    for thread in threads:
        thread.join()

if __name__ == "__main__":
    main()