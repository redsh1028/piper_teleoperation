#!/usr/bin/env python3
# -*-coding:utf8-*-

from typing import Optional
from piper_sdk import *
from WeGo_MetaClass import WeGo

"""
메타클래스를 활용하여 포지션 값만 입력하면 로봇팔이 자동으로 목표 위치로 이동가능
"""

class MoveAll:
    def __init__(self, piper:C_PiperInterface):
        # piper 인스턴스 생성 및 연결
        self.piper = piper
        self.piper.ConnectPort()  # 포트 연결

        # WeGo 인스턴스 생성 및 로봇팔 활성화
        self.wego = WeGo(self.piper)
        self.wego.enable.run()
        
        self.position = [0,0,0,0,0,0,0]
        self.positions = [
            [0,0,0,0,0,0,0x01],
            [0,0,0,0,0,0,0x02],
            [0,0,0,0,0,0,0x03]
        ]

    def move(self):
        # movej => 조인트 각도(Deg): [조인트1, 조인트2, 조인트3, 조인트4, 조인트5, 조인트6, 그리퍼] 
        self.position = [0, 0, 0, 0, 0, 0, 0]
        self.wego.movej.run(*self.position)

        # movep / l => 포지션(mm, Deg) : [X 좌표값, Y 좌표값, Z 좌표값, RX (회전) 좌표값, RY (회전) 좌표값, RZ (회전) 좌표값, 그리퍼]
        self.position = [190, 0, 400, 0, 85, 0, 70]
        self.wego.movep.run(*self.position)  

        self.position = [190, 0, 250, 0, 85, 0, 0]
        self.wego.movel.run(*self.position)  

        # movec => 출발점, 경유점, 도착점 : [3개 점을 하나의 리스트로 전달]
        self.positions = [
            [60, 0, 250, 0, 85, 0, 0x01],  # 출발점
            [200, 0, 400, 0, 85, 0, 0x02],  # 경유점
            [60, 0, 550, 0, 85, 0, 0x03],  # 도착점
        ]
        self.wego.movec.run(self.positions)

if __name__ == "__main__":
    piper = C_PiperInterface("can0") # piper 인스턴스를 만들어서 전달
    moveall = MoveAll(piper) # MoveAll 인스턴스 생성
    
    moveall.move() # 로봇 이동

