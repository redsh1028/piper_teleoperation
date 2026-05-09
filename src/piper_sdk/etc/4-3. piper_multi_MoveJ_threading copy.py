#!/usr/bin/env python3
# -*-coding:utf8-*-

from typing import Optional
import time, math
from piper_sdk import *
from piper_sdk.interface.piper_interface_v2 import C_PiperInterface_V2
import threading

def wait_for_threads_ready(threads):
    while not all(thread.is_alive() for thread in threads):
        time.sleep(0.1)

class Move_J:
    def __init__(self, piper: C_PiperInterface_V2):
        self.piper = piper
        self.factor = 1000
        self.position = [0, 0, 0, 0, 0, 0, 0]
        self.gripper = 0
        self.count = 0

    def enable(self):
        enable_flag = False
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
                print("시간 초과 - 프로그램 종료")
                exit(0)
            time.sleep(1)

    def convert(self):
        return [round(angle * self.factor) for angle in self.position] + [round(self.gripper * self.factor)]

    def move(self):
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
            
            if self.count >= len(waypoints) * 50:
                self.count = 0
            
            index = self.count // 50
            self.position, self.gripper = waypoints[index]
            
            converted_values = self.convert()
            self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            self.piper.JointCtrl(*converted_values[:6])
            self.piper.GripperCtrl(abs(converted_values[6]), 1000, 0x01, 0)
            time.sleep(0.005)

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
    
    wait_for_threads_ready(threads)
    
    for thread in threads:
        thread.join()

if __name__ == "__main__":
    main()
