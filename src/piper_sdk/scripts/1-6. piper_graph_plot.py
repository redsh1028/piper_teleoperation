#!/usr/bin/env python3
# -*- coding:utf8 -*-

import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from piper_sdk import C_PiperInterface

class RobotDataCollector:
    def __init__(self):
        # 데이터 저장 리스트 초기화
        self.time_stamps, self.x_positions, self.y_positions, self.z_positions = [], [], [], []
        self.rx_angles, self.ry_angles, self.rz_angles = [], [], []
        self.J1_angles, self.J2_angles, self.J3_angles, self.J4_angles, self.J5_angles, self.J6_angles = [], [], [], [], [], []
        self.gripper_angles, self.gripper_efforts = [], []

        self.piper = C_PiperInterface()  # Piper 인터페이스 생성
        self.piper.ConnectPort()  # 포트 연결
        self.fig, self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax5_right = self.init_plots()

        self.start_time = time.time()  # 시작 시간 기록

    def init_plots(self):
        """ 그래프 초기화 함수 """
        plt.ion()  # 실시간 그래프 모드 활성화
        fig = plt.figure(figsize=(15, 5))  # 그래프 크기 설정
        ax1 = fig.add_subplot(321, projection='3d', title="3D Trajectory", xlabel="X (mm)", ylabel="Y (mm)", zlabel="Z (mm)")
        ax2 = fig.add_subplot(322, title="Position Over Time", xlabel="Time (s)", ylabel="Position (mm)")
        ax3 = fig.add_subplot(323, title="Rotation Over Time", xlabel="Time (s)", ylabel="Rotation (°)")
        ax4 = fig.add_subplot(324, title="Joint Angle Over Time", xlabel="Time (s)", ylabel="Joint Angle (°)")
        # Gripper State over time with dual y-axes
        ax5 = fig.add_subplot(313)  # Gripper State Over Time (왼쪽 y축)
        ax5.set_title("Gripper State Over Time")
        ax5.set_xlabel("Time (s)")
        ax5.set_ylabel("Gripper Angle Distance (mm)", color='blue')

        ax5_right = ax5.twinx()  # 오른쪽 y축 추가
        ax5_right.set_ylabel("Gripper Effort", color='red')
        return fig, ax1, ax2, ax3, ax4, ax5, ax5_right

    def update_plots(self):
        """ 그래프 업데이트 함수 """
        # 3D Trajectory (3D 궤적)
        self.ax1.clear()
        self.ax1.plot(self.x_positions, self.y_positions, self.z_positions, color="blue")

        # Position over time (시간에 따른 위치 변화)
        self.ax2.clear()
        self.ax2.plot(self.time_stamps, self.x_positions, label="X")
        self.ax2.plot(self.time_stamps, self.y_positions, label="Y")
        self.ax2.plot(self.time_stamps, self.z_positions, label="Z")
        self.ax2.legend()

        # Rotation over time (시간에 따른 회전 각도 변화)
        self.ax3.clear()
        self.ax3.plot(self.time_stamps, self.rx_angles, label="RX")
        self.ax3.plot(self.time_stamps, self.ry_angles, label="RY")
        self.ax3.plot(self.time_stamps, self.rz_angles, label="RZ")
        self.ax3.legend()

        # Joint Angle over time (시간에 따른 관절 각도 변화)
        self.ax4.clear()
        for i, joint in enumerate([self.J1_angles, self.J2_angles, self.J3_angles, self.J4_angles, self.J5_angles, self.J6_angles], start=1):
            self.ax4.plot(self.time_stamps, joint, label=f"Joint {i}")
        self.ax4.legend()

        # Gripper State over time (그리퍼 상태 변화)
        self.ax5.clear()
        self.ax5.plot(self.time_stamps, self.gripper_angles, color='blue', label="Angle")
        self.ax5_right.clear()
        self.ax5_right.plot(self.time_stamps, self.gripper_efforts, color='red', label="Effort")
        self.ax5.legend(loc='upper left')
        self.ax5_right.legend(loc='upper right')

        plt.pause(0.01)  # 그래프 업데이트 잠시 대기

    def save_data_to_file(self, file_name):
        """ 데이터 파일 저장 함수 """
        with open(file_name, "w", encoding="utf-8") as file:
            # 파일에 헤더 작성
            file.write("Time (s), X (mm), Y (mm), Z (mm), RX (°), RY (°), RZ (°), J1 (°), J2 (°), J3 (°), J4 (°), J5 (°), J6 (°), Gripper Angle (mm), Gripper Effort\n")
            # 데이터 항목을 파일에 기록
            for i in range(len(self.time_stamps)):
                file.write(f"{self.time_stamps[i]:.3f}, {self.x_positions[i]:.3f}, {self.y_positions[i]:.3f}, {self.z_positions[i]:.3f}, "
                           f"{self.rx_angles[i]:.3f}, {self.ry_angles[i]:.3f}, {self.rz_angles[i]:.3f}, {self.J1_angles[i]:.3f}, {self.J2_angles[i]:.3f}, "
                           f"{self.J3_angles[i]:.3f}, {self.J4_angles[i]:.3f}, {self.J5_angles[i]:.3f}, {self.J6_angles[i]:.3f}, {self.gripper_angles[i]:.3f}, {self.gripper_efforts[i]:.3f}\n")

    def collect_data(self):
        """ 데이터 수집 및 그래프 업데이트 함수 """
        try:
            while True:
                # 데이터 수집
                arm_pose = self.piper.GetArmEndPoseMsgs().end_pose  # 로봇의 끝자세 (포즈) 가져오기
                arm_joint = self.piper.GetArmJointMsgs().joint_state  # 로봇 관절 상태 가져오기
                gripper = self.piper.GetArmGripperMsgs().gripper_state  # 그리퍼 상태 가져오기
                timestamp = time.time() - self.start_time  # 현재 시간 계산

                # 위치 및 회전 정보 저장
                self.x_positions.append(arm_pose.X_axis / 1000.0)
                self.y_positions.append(arm_pose.Y_axis / 1000.0)
                self.z_positions.append(arm_pose.Z_axis / 1000.0)
                self.rx_angles.append(arm_pose.RX_axis / 1000.0)
                self.ry_angles.append(arm_pose.RY_axis / 1000.0)
                self.rz_angles.append(arm_pose.RZ_axis / 1000.0)
                # 관절 각도 정보 저장
                self.J1_angles.append(arm_joint.joint_1 / 1000.0)
                self.J2_angles.append(arm_joint.joint_2 / 1000.0)
                self.J3_angles.append(arm_joint.joint_3 / 1000.0)
                self.J4_angles.append(arm_joint.joint_4 / 1000.0)
                self.J5_angles.append(arm_joint.joint_5 / 1000.0)
                self.J6_angles.append(arm_joint.joint_6 / 1000.0)
                # 그리퍼 상태 저장
                self.gripper_angles.append(gripper.grippers_angle / 1000.0)
                self.gripper_efforts.append(gripper.grippers_effort / 1000.0)
                self.time_stamps.append(timestamp)  # 시간 저장

                # 그래프 업데이트
                self.update_plots()
                time.sleep(0.01)  # 0.01초 대기
        except KeyboardInterrupt:
            # 키보드 인터럽트 발생 시 처리
            print("데이터 수집 완료")
            self.save_data_to_file("robot_data.csv")  # 데이터 저장
            print("데이터 저장 완료")
            plt.ioff()  # 인터랙티브 모드 종료
            plt.show()  # 그래프 표시

if __name__ == "__main__":
    collector = RobotDataCollector()  # 객체 생성
    collector.collect_data()  # 데이터 수집 및 그래프 업데이트 시작
