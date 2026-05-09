#!/usr/bin/env python3
# -*-coding:utf8-*-

"""
PiPER의 세부 사항을 설정할 수 있는 함수들의 모음입니다.
해당 함수를 적용 후, 0-1. piper_arm_info.py에서 확인이 가능합니다.
"""

from typing import (
    Optional,
)
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2()
    piper.ConnectPort()

    # # PiPER 모터 최대 속도 설정
    # # 각 조인트의 모터 최대 속도는 3.0 rad/s까지입니다.
    # # 모터 속도 범위는 입력 단위에 의해, 0-3000까지 입니다.
    # piper.MotorMaxSpdSet(1, 300)
    # time.sleep(0.1)        
    # piper.MotorMaxSpdSet(2, 300)    
    # time.sleep(0.1)           
    # piper.MotorMaxSpdSet(3, 300)
    # time.sleep(0.1)       
    # piper.MotorMaxSpdSet(4, 300)
    # time.sleep(0.1)        
    # piper.MotorMaxSpdSet(5, 300)
    # time.sleep(0.1)
    # piper.MotorMaxSpdSet(6, 300)
    # time.sleep(0.1)

    # # PiPER 모터 최대 가속도 설정
    # # 각 조인트의 모터 최대 가속도는 5.0 rad/s^2입니다.
    # # 모터 가속도 범위는 입력 단위에 의해, 0-500까지 입니다.    
    # piper.JointMaxAccConfig(1,500)
    # time.sleep(0.1)
    # piper.JointMaxAccConfig(2,500)
    # time.sleep(0.1)
    # piper.JointMaxAccConfig(3,500)
    # time.sleep(0.1)
    # piper.JointMaxAccConfig(4,500)
    # time.sleep(0.1)
    # piper.JointMaxAccConfig(5,500)
    # time.sleep(0.1)
    # piper.JointMaxAccConfig(6,500)        
    # time.sleep(0.1)

    # # PiPER Crash Protection 설정
    # piper.CrashProtectionConfig(8,8,8,8,8,8)
    # piper.CrashProtectionConfig(0,0,0,0,0,0)        
    # piper.ArmParamEnquiryAndConfig(0x02, 0x00, 0x00, 0x00, 0x03)

    # # PiPER 그리퍼 세부 세팅
    # piper.GripperTeachingPendantParamConfig(100, 70)