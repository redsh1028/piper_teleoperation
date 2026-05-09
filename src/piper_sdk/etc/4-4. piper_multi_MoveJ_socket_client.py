import json
import socket
import time

# 웨이포인트 정의 (6개의 조인트 각도와 그리퍼 값)
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

def start_client():
    host = 'localhost'
    port = 12345

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client_socket.connect((host, port))
        print("서버와 연결 성공!")

        for waypoint in waypoints:
            joint_data = waypoint[0] + [waypoint[1]]
            print(f"전송 중: {joint_data}")

            # 데이터를 JSON 형식으로 서버에 전송
            client_socket.send(json.dumps(joint_data).encode('utf-8'))

            # 서버로부터 응답 받기
            response = client_socket.recv(1024)
            print(f"서버 응답: {response.decode('utf-8')}")

            time.sleep(1)
    except Exception as e:
        print(f"서버 연결 오류: {e}")
    finally:
        client_socket.close()

if __name__ == "__main__":
    start_client()
