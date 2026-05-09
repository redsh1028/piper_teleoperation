#!/usr/bin/env python3
# -*-coding:utf8-*-

from typing import (
    Optional,
)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import threading
import math
from piper_sdk import *
from piper_sdk import C_PiperInterface_V2
from piper_msgs.msg import PiperStatusMsg, PosCmd
from piper_msgs.srv import Enable
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
from numpy import clip


class C_PiperRosNode(Node):
    def __init__(self) -> None:
        super().__init__("piper_single_ctrl_node")

        self.declare_parameter("can_port", "can0")
        self.declare_parameter("auto_enable", False)
        self.declare_parameter("gripper_exist", True)
        self.declare_parameter("rviz_ctrl_flag", False)
        self.declare_parameter("debug_flag", True)

        self.can_port = self.get_parameter("can_port").get_parameter_value().string_value
        self.auto_enable = self.get_parameter("auto_enable").get_parameter_value().bool_value
        self.gripper_exist = self.get_parameter("gripper_exist").get_parameter_value().bool_value
        self.rviz_ctrl_flag = self.get_parameter("rviz_ctrl_flag").get_parameter_value().bool_value
        self.debug = self.get_parameter("debug_flag").get_parameter_value().bool_value  # Debug 값을 가져옴

        # Log messages based on debug flag
        if self.debug:
            self.get_logger().info(f"can_port is {self.can_port}")
            self.get_logger().info(f"auto_enable is {self.auto_enable}")
            self.get_logger().info(f"gripper_exist is {self.gripper_exist}")
            self.get_logger().info(f"rviz_ctrl_flag is {self.rviz_ctrl_flag}")

        # Publishers
        self.joint_pub = self.create_publisher(JointState, "joint_states_single", 1)
        self.arm_status_pub = self.create_publisher(PiperStatusMsg, "arm_status", 1)
        self.end_pose_pub = self.create_publisher(Pose, "end_pose", 1)
        self.init_pos_pub = self.create_publisher(Bool, "init_pos", 1)

        self.init_pos_msg = Bool()
        self.init_pos_msg.data = True  # 원하는 값을 설정
        self.init_pos_pub.publish(self.init_pos_msg)

        # Service
        self.motor_srv = self.create_service(Enable, "enable_srv", self.handle_enable_service)

        # Joint
        self.joint_states = JointState()
        self.joint_states.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 7
        self.joint_states.effort = [0.0] * 7

        # Enable flag
        self.__enable_flag = False

        # Create piper class and open CAN interface
        self.piper = C_PiperInterface_V2(can_name=self.can_port)
        self.piper.ConnectPort()

        # MoveC를 위한 count 변수 지정
        self.count = 0
        self.position_state = []
        self.position_states = []

        # Start subscription thread
        self.create_subscription(PosCmd, "pos_cmd", self.pos_callback, 1)
        self.create_subscription(JointState, "joint_ctrl_single", self.joint_callback, 1)
        self.create_subscription(Bool, "enable_flag", self.enable_callback, 1)
        self.create_subscription(Bool, "init_pos", self.init_callback, 1)
        self.create_subscription(Bool, "gripper_ctrl", self.gripper_callback, 1)

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
                while not (enable_flag):
                    elapsed_time = time.time() - start_time
                    enable_flag = (
                        self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status
                        and self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status
                        and self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status
                        and self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status
                        and self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status
                        and self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                    )
                    self.piper.EnableArm(7)
                    self.piper.GripperCtrl(0, 1000, 0x01, 0)
                    if enable_flag:
                        self.__enable_flag = True
                    if elapsed_time > timeout:
                        print("시간초과")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(0.1)
                    pass
            if elapsed_time_flag:
                print("프로그램을 종료합니다")
                exit(0)

            self.PublishArmState()
            self.PublishArmJointAndGirpper()
            self.PubilshArmEndPose()

            rate.sleep()

    def PublishArmState(self):
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        arm_status.arm_status = self.piper.GetArmStatus().arm_status.arm_status
        arm_status.mode_feedback = self.piper.GetArmStatus().arm_status.mode_feed
        arm_status.teach_status = self.piper.GetArmStatus().arm_status.teach_status
        arm_status.motion_status = self.piper.GetArmStatus().arm_status.motion_status
        arm_status.trajectory_num = self.piper.GetArmStatus().arm_status.trajectory_num
        arm_status.err_code = self.piper.GetArmStatus().arm_status.err_code
        arm_status.joint_1_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)

    def PublishArmJointAndGirpper(self):
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_1: float = (self.piper.GetArmJointMsgs().joint_state.joint_1 / 1000) * 0.017444
        joint_2: float = (self.piper.GetArmJointMsgs().joint_state.joint_2 / 1000) * 0.017444
        joint_3: float = (self.piper.GetArmJointMsgs().joint_state.joint_3 / 1000) * 0.017444
        joint_4: float = (self.piper.GetArmJointMsgs().joint_state.joint_4 / 1000) * 0.017444
        joint_5: float = (self.piper.GetArmJointMsgs().joint_state.joint_5 / 1000) * 0.017444
        joint_6: float = (self.piper.GetArmJointMsgs().joint_state.joint_6 / 1000) * 0.017444
        gripper: float = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1000000
        vel_0: float = self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed / 1000
        vel_1: float = self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed / 1000
        vel_2: float = self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed / 1000
        vel_3: float = self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed / 1000
        vel_4: float = self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed / 1000
        vel_5: float = self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed / 1000
        effort_6: float = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort / 1000
        self.joint_states.position = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, gripper]  # Example values
        self.joint_states.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5, 0.0]  # Example values
        self.joint_states.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, effort_6]
        self.joint_pub.publish(self.joint_states)

    def PubilshArmEndPose(self):
        endpos = Pose()
        endpos.position.x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis / 1000000
        endpos.position.y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis / 1000000
        endpos.position.z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis / 1000000
        roll = self.piper.GetArmEndPoseMsgs().end_pose.RX_axis / 1000
        pitch = self.piper.GetArmEndPoseMsgs().end_pose.RY_axis / 1000
        yaw = self.piper.GetArmEndPoseMsgs().end_pose.RZ_axis / 1000
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        quaternion = R.from_euler("xyz", [roll, pitch, yaw]).as_quat()
        endpos.orientation.x = quaternion[0]
        endpos.orientation.y = quaternion[1]
        endpos.orientation.z = quaternion[2]
        endpos.orientation.w = quaternion[3]
        self.end_pose_pub.publish(endpos)

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
            self.piper.MotionCtrl_1(0x00, 0x00, 0x00)
            self.piper.MotionCtrl_2(pos_data.mode1, pos_data.mode2, 50)

            if pos_data.mode2 == 0x03:
                self.piper.EndPoseCtrl(*self.position_states[self.count])
                self.count += 0x01
                self.piper.MoveCAxisUpdateCtrl(self.count)
                time.sleep(0.001)
            else:
                self.piper.EndPoseCtrl(*self.position_state)

                gripper = round(pos_data.gripper * 1000 * 1000)
                if pos_data.gripper > 70000:
                    gripper = 70000
                if pos_data.gripper < 0:
                    gripper = 0
                if self.gripper_exist:
                    self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)

        # 3회 반복 후 초기화
        if self.count == 3:
            self.count = 0

    def joint_callback(self, joint_data):
        factor = 57324.840764  # 1000*180/3.14
        joint_positions = {}

        for idx, joint_name in enumerate(joint_data.name):
            if self.debug:
                self.get_logger().info(f"{joint_name}: {joint_data.position[idx]}")
            joint_positions[joint_name] = round(joint_data.position[idx] * factor)

        if len(joint_data.position) == 6:
            joint_data.position = list(joint_data.position) + [0.0, 0.0]
        elif len(joint_data.position) == 7:
            joint_data.position = list(joint_data.position) + [0.0]
        
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
                    self.piper.MotionCtrl_2(0x01, 0x01, vel_all)
                else:
                    self.piper.MotionCtrl_2(0x01, 0x01, 30)
            else:
                self.piper.MotionCtrl_2(0x01, 0x01, 30)

            self.piper.JointCtrl(
                joint_positions.get("joint1", 0),
                joint_positions.get("joint2", 0),
                joint_positions.get("joint3", 0),
                joint_positions.get("joint4", 0),
                joint_positions.get("joint5", 0),
                joint_positions.get("joint6", 0),
            )

            if self.gripper_exist:
                if len(joint_data.effort) >= 7:
                    gripper_effort = clip(joint_data.effort[6], 0.5, 3)
                    if self.debug:
                        self.get_logger().info(f"gripper_effort: {gripper_effort}")
                    if not math.isnan(gripper_effort):
                        gripper_effort = round(gripper_effort * 1000)
                    else:
                        if self.debug:
                            self.get_logger().warning("Gripper effort is NaN, using default value.")
                        gripper_effort = 0
                    self.piper.GripperCtrl(abs(gripper), gripper_effort, 0x01, 0)
                else:
                    self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)

    def enable_callback(self, enable_flag: Bool):
        if self.debug:
            self.get_logger().info(f"Received enable flag:")
            self.get_logger().info(f"enable_flag: {enable_flag.data}")
        if enable_flag.data:
            self.__enable_flag = True
            self.piper.EnableArm(7)
            if self.gripper_exist:
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
        else:
            self.__enable_flag = False
            self.piper.DisableArm(7)
            if self.gripper_exist:
                self.piper.GripperCtrl(0, 1000, 0x00, 0)

    # init_pos 토픽 콜백 함수 추가
    def init_callback(self, init_flag: Bool):
        """Callback to move the robot to initial position when init_pos is triggered."""
        if init_flag.data:  # If True, move to initial position
            if self.debug:
                self.get_logger().info("Moving to initial position...")
            # Reset joint positions to 0
            initial_joint_positions = [0, 0, 0, 0, 0, 0]
            self.piper.MotionCtrl_2(0x01, 0x01, 50)
            self.piper.GripperCtrl(0, 1000, 0x01, 0)
            self.piper.JointCtrl(*initial_joint_positions)

    # gripper_ctrl 토픽 콜백 함수 추가
    def gripper_callback(self, gripper_flag: Bool):
        """Callback to open or close the gripper when gripper_ctrl is triggered."""
        if gripper_flag.data:
            if self.debug:
                self.get_logger().info("Close the gripper...")
            self.gripper_min = 0
            self.piper.MotionCtrl_2(0x01, self.piper.GetArmStatus().arm_status.mode_feed, 100)
            self.piper.GripperCtrl(self.gripper_min, 1000, 0x01, 0)

        else:
            if self.debug:
                self.get_logger().info("Open the gripper...")
            self.gripper_max = 70 * 1000 * 1000
            self.piper.MotionCtrl_2(0x01, self.piper.GetArmStatus().arm_status.mode_feed, 100)
            self.piper.GripperCtrl(self.gripper_max, 1000, 0x01, 0)

    def handle_enable_service(self, req, resp):
        if self.debug:
            self.get_logger().info(f"Received request:: {req.enable_request}")
        enable_flag = False
        loop_flag = False
        timeout = 5
        start_time = time.time()
        elapsed_time_flag = False
        while not (loop_flag):
            elapsed_time = time.time() - start_time
            enable_list = []
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status)
            if req.enable_request:
                enable_flag = all(enable_list)
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
            else:
                enable_flag = any(enable_list)
                self.piper.DisableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x02, 0)
            if self.debug:
                self.get_logger().info(f"Enable State: {enable_flag}")
            self.__enable_flag = enable_flag
            if enable_flag == req.enable_request:
                loop_flag = True
                enable_flag = True
            else:
                loop_flag = False
                enable_flag = False
            if elapsed_time > timeout:
                if self.debug:
                    self.get_logger().info(f"Timeout....")
                elapsed_time_flag = True
                enable_flag = False
                loop_flag = True
                break
            time.sleep(0.5)
        resp.enable_response = enable_flag
        if self.debug:
            self.get_logger().info(f"Returning response: {resp.enable_response}")
        return resp


def main(args=None):
    rclpy.init(args=args)
    piper_single_node = C_PiperRosNode()
    try:
        rclpy.spin(piper_single_node)
    except KeyboardInterrupt:
        pass
    finally:
        piper_single_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
