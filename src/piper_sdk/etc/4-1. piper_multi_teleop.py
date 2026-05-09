#!/usr/bin/env python3
# -*-coding:utf8-*-

from typing import (
    Optional,
)
import time
import math
from piper_sdk import *

def enable_fun(piper:C_PiperInterface):
    enable_flag = False
    elapsed_time_flag = False
    timeout = 5 
    start_time = time.time()
    while not (enable_flag):
        elapsed_time = time.time() - start_time
        enable_flag = piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
        piper.GripperCtrl(0,1000,0x01, 0)
        if elapsed_time > timeout:
            print("Timeout....")
            elapsed_time_flag = True
            enable_flag = True
            break
        time.sleep(1)
        pass
    if(elapsed_time_flag):
        print("The program automatically enables timeout, exit the program")
        exit(0)

def send_to_arms(piper2, piper3, piper4, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, gripper):
    # `piper2`, `piper3`, `piper4`에만 조인트와 그리퍼 값을 전송
    piper2.MotionCtrl_2(0x01, 0x01, 100, 0xAD)
    piper2.JointCtrl(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6)
    piper2.GripperCtrl(abs(gripper), 1000, 0x01, 0)

    piper3.MotionCtrl_2(0x01, 0x01, 100, 0xAD)
    piper3.JointCtrl(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6)
    piper3.GripperCtrl(abs(gripper), 1000, 0x01, 0)

    piper4.MotionCtrl_2(0x01, 0x01, 100, 0xAD)
    piper4.JointCtrl(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6)
    piper4.GripperCtrl(abs(gripper), 1000, 0x01, 0)

if __name__ == "__main__":
    # 4개의 CAN 포트를 설정
    piper1 = C_PiperInterface("can0")
    piper2 = C_PiperInterface("can1")
    piper3 = C_PiperInterface("can2")
    piper4 = C_PiperInterface("can3")

    piper1.ConnectPort()
    piper2.ConnectPort()
    piper3.ConnectPort()
    piper4.ConnectPort()

    # 모든 포트에 대해 EnableArm 호출
    piper1.EnableArm(7)
    piper2.EnableArm(7)
    piper3.EnableArm(7)    
    piper4.EnableArm(7)

    # # 각 포트에 대해 Enable 설정
    # enable_fun(piper=piper1)
    # enable_fun(piper=piper2)
    # enable_fun(piper=piper3)
    # enable_fun(piper=piper4)

    while True:
        # `piper1`에서 값을 받아옴
        joint_1 = piper1.GetArmJointCtrl().joint_ctrl.joint_1
        joint_2 = piper1.GetArmJointCtrl().joint_ctrl.joint_2
        joint_3 = piper1.GetArmJointCtrl().joint_ctrl.joint_3
        joint_4 = piper1.GetArmJointCtrl().joint_ctrl.joint_4
        joint_5 = piper1.GetArmJointCtrl().joint_ctrl.joint_5
        joint_6 = piper1.GetArmJointCtrl().joint_ctrl.joint_6
        gripper = piper1.GetArmGripperCtrl().gripper_ctrl.grippers_angle

        # piper2, piper3, piper4에만 조인트 값과 그리퍼 값 전송
        send_to_arms(piper2, piper3, piper4, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, gripper)

        time.sleep(0.005)
