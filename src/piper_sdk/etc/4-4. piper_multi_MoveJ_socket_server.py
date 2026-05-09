import json
import socket
import time
from piper_sdk import *
from piper_sdk.interface.piper_interface_v2 import C_PiperInterface_V2

# 로봇 팔 클래스
class RobotArm:
    def __init__(self, piper: C_PiperInterface_V2):
        self.piper = piper

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

    def move(self, joint_data):
        # 클라이언트에서 받은 joint_data로 로봇 팔을 움직임
        converted_values = self.convert(joint_data)
        self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        self.piper.JointCtrl(*converted_values[:6])
        self.piper.GripperCtrl(abs(converted_values[6]), 1000, 0x01, 0)
    
    def convert(self, joint_data):
        factor = 1000
        return [round(angle * factor) for angle in joint_data[:6]] + [round(joint_data[6] * factor)]

# 서버 코드
def start_server():
    host = 'localhost'
    port = 12345

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)

    print(f"서버 시작: {host}:{port}")

    piper_interfaces = [C_PiperInterface(f"can{i}") for i in range(4)]
    for piper in piper_interfaces:
        piper.ConnectPort()

    robot_arms = [RobotArm(piper) for piper in piper_interfaces]
    for arm in robot_arms:
        arm.enable()

    while True:
        try:
            client_socket, client_address = server_socket.accept()
            print(f"클라이언트 {client_address} 연결됨")

            while True:
                data = client_socket.recv(1024)
                if not data:
                    break

                try:
                    # 받은 데이터를 JSON으로 파싱하여 조인트 값 처리
                    joint_data = json.loads(data.decode('utf-8'))
                    print(f"받은 데이터: {joint_data}")

                    # 모든 로봇 팔에 동일한 조인트 데이터를 보내어 동기화
                    for arm in robot_arms:
                        arm.move(joint_data)

                    client_socket.send("명령 전송 완료".encode('utf-8'))
                except json.JSONDecodeError as e:
                    print(f"JSON 파싱 오류: {e}")
                    client_socket.send("잘못된 데이터 형식".encode('utf-8'))
        except Exception as e:
            print(f"클라이언트 처리 중 오류: {e}")
        finally:
            client_socket.close()

if __name__ == "__main__":
    start_server()
