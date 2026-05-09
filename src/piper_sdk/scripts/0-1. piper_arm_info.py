#!/usr/bin/env python3
# -*-coding:utf8-*-

"""
PiPER에 관한 대부분의 피드백을 해당 파일에서 확인할 수 있습니다.
한 개씩 주석을 해제하며 차근차근 확인하실 수 있습니다.
"""

from typing import Optional
import time, math
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface()
    piper.ConnectPort()
    while True:

        # # [1] PiPER 슬레이브암 관련 피드백 받는 함수
        # print(piper.GetArmStatus())
        # print(piper.GetArmEndPoseMsgs())
        # print(piper.GetArmJointMsgs())
        # print(piper.GetArmGripperMsgs())
        # print(piper.GetArmHighSpdInfoMsgs())
        # print(piper.GetArmLowSpdInfoMsgs())
        # print(piper.GetCurrentEndVelAndAccParam())
        # print(piper.GetCrashProtectionLevelFeedback())

        # # PiPER 특정 모터의 피드백 받는 함수
        # piper.SearchMotorMaxAngleSpdAccLimit(1,0x02)
        # print(piper.GetCurrentMotorMaxAccLimit())
        # print(piper.GetCurrentMotorAngleLimitMaxVel())

        # # PiPER 전체 모터의 피드백 받는 함수
        # piper.SearchAllMotorMaxAngleSpd()
        # print(piper.GetAllMotorAngleLimitMaxSpd())

        # piper.SearchAllMotorMaxAccLimit()
        # print(piper.GetAllMotorMaxAccLimit())

        # # [2] PiPER 마스터암 관련 피드백 받는 함수
        # print(piper.GetArmJointCtrl())
        # print(piper.GetArmGripperCtrl())
        # print(piper.GetArmCtrlCode151())

        # # [3] PiPER 기타 피드백 받는 함수
        # PiPER Forward Kinematics 피드백 받는 함수
        # print(f"feedback:{piper.GetFK('feedback')}")
        # print(f"control:{piper.GetFK('control')}")

        # CAN통신 FPS 피드백 받는 함수
        # print(f"can: {piper.GetCanFps()}")
        # print(f"all_fps: {piper.GetCanFps()}")
        # print(f"status: {piper.GetArmStatus().Hz}")
        # print(f"end_pose: {piper.GetArmEndPoseMsgs().Hz}")
        # print(f"joint_states: {piper.GetArmJointMsgs().Hz}")
        # print(f"gripper_msg: {piper.GetArmGripperMsgs().Hz}")
        # print(f"high_spd: {piper.GetArmHighSpdInfoMsgs().Hz}")
        # print(f"low_spd: {piper.GetArmLowSpdInfoMsgs().Hz}")
        # print(f"joint_ctrl: {piper.GetArmJointCtrl().Hz}")
        # print(f"gripper_ctrl: {piper.GetArmGripperCtrl().Hz}")
        # print(f"ctrl_151: {piper.GetArmCtrlCode151().Hz}")
        time.sleep(0.005)
        pass
