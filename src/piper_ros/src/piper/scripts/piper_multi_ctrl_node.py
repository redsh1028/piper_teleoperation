#!/usr/bin/env python3
# -*-coding:utf8-*-

from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time, math
import threading
from piper_sdk import *
from piper_sdk import C_PiperInterface_V2
from piper_msgs.msg import PiperStatusMsg, PosCmd
from geometry_msgs.msg import Pose
from numpy import clip


class C_PiperRosNode(Node):
    def __init__(self) -> None:
        super().__init__("piper_multi_ctrl_node")

        # 4개의 Piper 인터페이스 인스턴스를 생성
        self.piper_interfaces = [C_PiperInterface_V2(f"can{i}") for i in range(4)]

        for piper in self.piper_interfaces:
            piper.ConnectPort()

        self.declare_parameter("auto_enable", True)
        self.declare_parameter("gripper_exist", True)
        self.declare_parameter("rviz_ctrl_flag", False)
        self.declare_parameter("debug_flag", False)

        self.auto_enable = self.get_parameter("auto_enable").get_parameter_value().bool_value
        self.gripper_exist = self.get_parameter("gripper_exist").get_parameter_value().bool_value
        self.rviz_ctrl_flag = self.get_parameter("rviz_ctrl_flag").get_parameter_value().bool_value
        self.debug = self.get_parameter("debug_flag").get_parameter_value().bool_value

        # joint
        self.joint_states = JointState()
        self.joint_states.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 7
        self.joint_states.effort = [0.0] * 7

        self.__enable_flag = False

        # MoveC를 위한 count 변수 지정
        self.count = 0
        self.position_state = []
        self.position_states = []

        # subscriber
        self.create_subscription(PosCmd, "pos_cmd", self.pos_callback, 1)
        self.create_subscription(JointState, "joint_states", self.joint_callback, 1)

        self.publisher_thread = threading.Thread(target=self.publish_thread)
        self.publisher_thread.start()

    def GetEnableFlag(self):
        return self.__enable_flag

    def publish_thread(self):
        rate = self.create_rate(200)  # 200 Hz
        enable_flag = False
        timeout = 5
        start_time = time.time()
        elapsed_time_flag = False
        while rclpy.ok():
            if self.auto_enable:
                while not enable_flag:
                    elapsed_time = time.time() - start_time
                    enable_flag = all(
                        piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status
                        and piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status
                        and piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status
                        and piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status
                        and piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status
                        and piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                        for piper in self.piper_interfaces
                    )

                    for piper in self.piper_interfaces:
                        piper.EnableArm(7)
                        piper.GripperCtrl(0, 1000, 0x01, 0)

                    if enable_flag:
                        self.__enable_flag = True
                    if elapsed_time > timeout:
                        print("Timeout....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(0.1)
            if elapsed_time_flag:
                print("The program automatically enables timeout, exit the program")
                exit(0)

            rate.sleep()

    def pos_callback(self, pos_data):
        if pos_data.mode1 is None:
            pos_data.mode1 = 0x01  # mode1 디폴트 값
        if pos_data.mode2 is None:
            pos_data.mode2 = 0x00  # mode2 디폴트 값 (MoveP)

        if self.debug:
            self.get_logger().info(f"Received PosCmd:")
            self.get_logger().info(f"x: {pos_data.x}")
            self.get_logger().info(f"y: {pos_data.y}")
            self.get_logger().info(f"z: {pos_data.z}")
            self.get_logger().info(f"roll: {pos_data.roll}")
            self.get_logger().info(f"pitch: {pos_data.pitch}")
            self.get_logger().info(f"yaw: {pos_data.yaw}")
            self.get_logger().info(f"gripper: {pos_data.gripper}")
            self.get_logger().info(f"mode1: {pos_data.mode1}")
            self.get_logger().info(f"mode2: {pos_data.mode2}")

        x = round(pos_data.x * 1000)
        y = round(pos_data.y * 1000)
        z = round(pos_data.z * 1000)
        rx = round(pos_data.roll * 1000)
        ry = round(pos_data.pitch * 1000)
        rz = round(pos_data.yaw * 1000)

        self.position_state = [x, y, z, rx, ry, rz]

        if pos_data.mode2 == 0x03:
            self.position_states.append(self.position_state)

        if self.GetEnableFlag():
            for piper in self.piper_interfaces:  # 각 Piper 인스턴스에 대해 처리
                piper.MotionCtrl_1(0x00, 0x00, 0x00)
                piper.MotionCtrl_2(pos_data.mode1, pos_data.mode2, 50)

            if pos_data.mode2 == 0x03:
                piper.EndPoseCtrl(*self.position_states[self.count])
                self.count += 0x01
                piper.MoveCAxisUpdateCtrl(self.count)
                time.sleep(0.001)
            else:
                piper.EndPoseCtrl(*self.position_state)

                gripper = round(pos_data.gripper * 1000 * 1000)
                if pos_data.gripper > 80000:
                    gripper = 80000
                if pos_data.gripper < 0:
                    gripper = 0
                if self.gripper_exist:
                    piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)
                piper.MotionCtrl_2(0x01, 0x00, 50)

        # 3회 반복 후 초기화
        if self.count == 3:
            self.count = 0

    def joint_callback(self, joint_data):
        factor = 57324.840764  # 1000*180/3.14
        joint_positions = {}
        joint_6 = 0

        for idx, joint_name in enumerate(joint_data.name):
            # self.get_logger().info(f"{joint_name}: {joint_data.position[idx]}")
            joint_positions[joint_name] = round(joint_data.position[idx] * factor)

        gripper = round(joint_data.position[6] * 1000 * 1000)
        gripper = clip(gripper, 0, 70000)

        if self.GetEnableFlag():
            if joint_data.velocity != []:
                all_zeros = all(v == 0 for v in joint_data.velocity)
            else:
                all_zeros = True

            if not all_zeros:
                lens = len(joint_data.velocity)
                if lens == 7:
                    vel_all = 30
                    vel_all = clip(vel_all, 0, 100)
                    for piper in self.piper_interfaces:
                        piper.MotionCtrl_2(0x01, 0x01, vel_all)
                else:
                    for piper in self.piper_interfaces:
                        piper.MotionCtrl_2(0x01, 0x01, 30)
            else:
                for piper in self.piper_interfaces:
                    piper.MotionCtrl_2(0x01, 0x01, 30)

            # Send joint control commands to each Piper interface
            for piper in self.piper_interfaces:
                piper.JointCtrl(
                    joint_positions.get("joint1", 0),
                    joint_positions.get("joint2", 0),
                    joint_positions.get("joint3", 0),
                    joint_positions.get("joint4", 0),
                    joint_positions.get("joint5", 0),
                    joint_positions.get("joint6", 0),
                )

            # Gripper control
            if self.gripper_exist:
                if len(joint_data.effort) >= 7:
                    gripper_effort = clip(joint_data.effort[6], 0.5, 3)
                    # self.get_logger().info(f"gripper_effort: {gripper_effort}")
                    if not math.isnan(gripper_effort):
                        gripper_effort = round(gripper_effort * 1000)
                    else:
                        # self.get_logger().warning("Gripper effort is NaN, using default value.")
                        gripper_effort = 0
                    for piper in self.piper_interfaces:
                        piper.GripperCtrl(abs(joint_6), gripper_effort, 0x01, 0)
                else:
                    for piper in self.piper_interfaces:
                        piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)


def main(args=None):
    rclpy.init(args=args)
    piper_node = C_PiperRosNode()
    rclpy.spin(piper_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
