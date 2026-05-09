#!/usr/bin/env python3
# -*- coding:utf8 -*-

from typing import Optional
import time
from piper_sdk import *

class WeGo:
    def __init__(self, piper: C_PiperInterface_V2):
        self.piper = piper
        self.factor = 1000  # 변환 계수 설정

        # 중첩 클래스 인스턴스 생성
        self.enable = self.Enable(self.piper)
        self.convert = self.Convert(self)
        self.check = self.Check(self)
        self.movej = self.MoveJ(self)
        self.movep = self.MoveP(self)
        self.movel = self.MoveL(self)
        self.movec = self.MoveC(self)

    class Enable:
        def __init__(self, piper: C_PiperInterface_V2):
            self.piper = piper

        def run(self):
            enable_flag = False
            elapsed_time_flag = False
            timeout = 5
            start_time = time.time()
            
            while not enable_flag:
                elapsed_time = time.time() - start_time
                enable_flag = all(
                    getattr(self.piper.GetArmLowSpdInfoMsgs(), f"motor_{i}").foc_status.driver_enable_status
                    for i in range(1, 7)
                )
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x01, 0)

                if elapsed_time > timeout:
                    print("시간 초과")
                    elapsed_time_flag = True
                    enable_flag = True
                    break
                time.sleep(1)

            if elapsed_time_flag:
                print("프로그램을 종료합니다")
                exit(0)

    class Convert:
        def __init__(self, outer):
            self.outer = outer

        def run(self, positions):
            """각도를 변환하는 함수 (포지션 + 그리퍼 포함)"""
            converted = [round(p * self.outer.factor) for p in positions]
            return converted


    class Check:
        def __init__(self, outer):
            self.outer = outer

        def joint(self, *target):
            while True:
                current_state = self.outer.piper.GetArmJointMsgs().joint_state
                current_pose = [
                    current_state.joint_1, current_state.joint_2, current_state.joint_3,
                    current_state.joint_4, current_state.joint_5, current_state.joint_6
                ]

                if all(abs(current_pose[i] - target[i]) < self.outer.factor/10 for i in range(6)):
                    print("조인트 목표 위치 도달")
                    print(self.outer.piper.GetArmJointMsgs())
                    break
                time.sleep(0.1)

        def pose(self, *target):
            while True:
                current_state = self.outer.piper.GetArmEndPoseMsgs().end_pose
                current_pose = [
                    current_state.X_axis, current_state.Y_axis, current_state.Z_axis,
                    current_state.RX_axis, current_state.RY_axis, current_state.RZ_axis
                ]

                if all(abs(current_pose[i] - target[i]) < self.outer.factor for i in range(6)):
                    print("포즈 목표 위치 도달")
                    print(self.outer.piper.GetArmEndPoseMsgs())
                    break
                time.sleep(0.1)

    class MoveJ:
        def __init__(self, outer):
            self.outer = outer

        def run(self, *position):
            """7개의 개별 값을 받아서 조인트 이동"""

            # 각도를 변환
            converted_values = self.outer.convert.run(position)
            joint_values = converted_values[:6]
            gripper_value = converted_values[6]
            
            # 명령 실행
            self.outer.piper.MotionCtrl_2(0x01, 0x01, 20, 0x00)
            self.outer.piper.JointCtrl(*joint_values)
            self.outer.piper.GripperCtrl(abs(gripper_value), 1000, 0x01, 0)
            self.outer.check.joint(*joint_values)            
            time.sleep(0.005)

    class MoveP:
        def __init__(self, outer):
            self.outer = outer

        def run(self, *position):
            """위치로 이동 (포지션 및 그리퍼 포함)"""

            # 각도를 변환
            converted_values = self.outer.convert.run(position)
            position_values = converted_values[:6]
            gripper_value = converted_values[6]
            
            # 명령 실행
            self.outer.piper.MotionCtrl_2(0x01, 0x00, 20, 0x00)
            self.outer.piper.EndPoseCtrl(*position_values)
            self.outer.piper.GripperCtrl(abs(gripper_value), 1000, 0x01, 0)
            self.outer.check.pose(*position_values)
            time.sleep(0.005)

    class MoveL:
        def __init__(self, outer):
            self.outer = outer

        def run(self, *position):
            """선형 이동 (포지션 및 그리퍼 포함)"""

            # 각도를 변환
            converted_values = self.outer.convert.run(position)
            position_values = converted_values[:6]
            gripper_value = converted_values[6]
            
            # 명령 실행
            self.outer.piper.MotionCtrl_2(0x01, 0x02, 20, 0x00)
            self.outer.piper.EndPoseCtrl(*position_values)
            self.outer.piper.GripperCtrl(abs(gripper_value), 1000, 0x01, 0)
            self.outer.check.pose(*position_values)
            time.sleep(0.005)

    class MoveC:
        def __init__(self, outer):
            self.outer = outer

        def run(self, positions):
            """원호 이동 (포지션 및 그리퍼 포함)"""

            self.outer.piper.MotionCtrl_2(0x01, 0x03, 20, 0x00)
            for position in positions:
                converted_values = self.outer.convert.run(position)
                self.outer.piper.EndPoseCtrl(*converted_values[:-1])
                self.outer.piper.MoveCAxisUpdateCtrl(position[-1])      
            time.sleep(0.005)
