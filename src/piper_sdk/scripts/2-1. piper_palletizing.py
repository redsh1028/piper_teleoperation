#!/usr/bin/env python3
# -*-coding:utf8-*-

from typing import Optional
import time, math
from piper_sdk import *

class PiPER_Palletizing:
    def __init__(self, piper: C_PiperInterface):
        self.piper = piper
        self.enable()

    def enable(self, timeout=5):
        """로봇팔을 활성화하고, 모터가 준비될 때까지 대기"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if all(getattr(self.piper.GetArmLowSpdInfoMsgs(), f"motor_{i}").foc_status.driver_enable_status for i in range(1, 7)):
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
                return
            time.sleep(1)
        print("시간초과")
        exit(0)

    def wait_for_motion_complete(self, target):
        """목표 위치에 도달할 때까지 대기"""
        while True:
            current_state = self.piper.GetArmEndPoseMsgs().end_pose
            current_pose = [
                current_state.X_axis, current_state.Y_axis, current_state.Z_axis,
                current_state.RX_axis, current_state.RY_axis, current_state.RZ_axis,
            ]
            if all(abs(current_pose[i] - target[i]) < 1000 for i in range(3)):
                print("목표 위치 도달")
                break
            time.sleep(0.1)

    def position_to_int(self, position):
        """좌표 값을 정수 단위로 변환"""
        return [int(coord * 1000) for coord in position]

    def move_p(self, x, y, z, rx, ry, rz):
        """Point-to-Point 모션 명령"""
        x, y, z, rx, ry, rz = self.position_to_int([x, y, z, rx, ry, rz])
        self.piper.MotionCtrl_2(0x01, 0x00, 50)
        self.piper.EndPoseCtrl(x, y, z, rx, ry, rz)

    def move_l(self, x, y, z, rx, ry, rz):
        """직선 보간 모션 명령"""
        x, y, z, rx, ry, rz = self.position_to_int([x, y, z, rx, ry, rz])
        self.piper.MotionCtrl_2(0x01, 0x02, 20)
        self.piper.EndPoseCtrl(x, y, z, rx, ry, rz)

    def pick_and_place(self, pick_pos, place_pos, gripper_open: int = 70000, gripper_close: int = 25000):
        """피킹 앤 플레이스 동작을 순차적으로 수행함."""

        # Z값에 30(임의로 설정)을 더한 시작 위치와 준비 위치
        start_pos = [pick_pos[0], pick_pos[1], pick_pos[2] + 30, *pick_pos[3:]]
        ready_pos = [place_pos[0], place_pos[1], place_pos[2] + 30, *place_pos[3:]]

        # 1. 시작 위치 근처로 이동
        self.move_p(*start_pos)
        self.wait_for_motion_complete(self.position_to_int(start_pos))

        # 2. 그리퍼 열기
        self.piper.GripperCtrl(gripper_open, 1000, 0x01, 0)

        # 3. 집기 위치로 낮게 이동
        self.move_l(*pick_pos)    
        self.wait_for_motion_complete(self.position_to_int(pick_pos))

        # 4. 그리퍼로 집기
        self.piper.GripperCtrl(gripper_close, 1000, 0x01, 0)
        time.sleep(1)  # 그리퍼가 물체를 잡을 시간

        # 5. 다시 올리기
        self.move_l(*start_pos)
        self.wait_for_motion_complete(self.position_to_int(start_pos))

        # 6. 배치 준비 위치로 이동 (Z값 고정)
        self.move_p(*ready_pos)
        self.wait_for_motion_complete(self.position_to_int(ready_pos))

        # 7. 물체 배치 위치로 낮게 이동
        self.move_l(*place_pos)
        self.wait_for_motion_complete(self.position_to_int(place_pos))

        # 8. 그리퍼 열어서 물체 놓기
        self.piper.GripperCtrl(gripper_open, 1000, 0x01, 0)

        # 9. 다시 올리기
        self.move_l(*ready_pos)
        self.wait_for_motion_complete(self.position_to_int(ready_pos))

        # 10. 그리퍼 닫기
        self.piper.GripperCtrl(0, 1000, 0x01, 0)

    def run(self):
        """pick_and_place 동작을 수행할 웨이포인트 시퀀스를 실행"""
        # 큐브 초기 위치 (좌측 하단)
        cube_position = [182, -128, 165, -180, 0, -172]
        x_offset = 11    # mm
        y_offset = 27    # mm 
        cube_width = 30  # mm
        cube_length = 50 # mm

        waypoints = [
            [268, 40, 170, 180, 0, -175],
            [265, 38 - (cube_width + y_offset), 170, 180, 0, -175],
            [263, 36 - 2*(cube_width + y_offset), 170, 180, 0, -175],
            [265 - (cube_length + x_offset), 38, 170, 180, 0, -175],
            [263 - (cube_length + x_offset), 37 - (cube_width + y_offset), 170, 180, 0, -175],
            [260 - (cube_length + x_offset), 36 - 2*(cube_width + y_offset), 170, 180, 0, -175]
        ]

        for index, waypoint in enumerate(waypoints):
            self.pick_and_place(cube_position, waypoint)
            print(f"{index+1} 박스 입니다.")
            time.sleep(0.005)

def main():
    piper = C_PiperInterface("can0")  # piper 인터페이스 생성
    piper.ConnectPort()  # 포트 연결

    pallet = PiPER_Palletizing(piper)
    pallet.run()

if __name__ == "__main__":
    main()
