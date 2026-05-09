#!/usr/bin/env python3
# -*-coding:utf8-*-

"""
해당 파일을 실행하기 전에 로봇팔을 안전한 위치로 이동시키고, PiPER를 비활성화해야 합니다.
0-2. piper_emergency_stop.py 파일을 클릭 후 Master-Slave 모드 변환을 해주세요.
"""

from typing import Optional
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2()
    piper.ConnectPort()
    # piper.MasterSlaveConfig(0xFA, 0, 0, 0)  # Mater
    piper.MasterSlaveConfig(0xFC, 0, 0, 0)  # Slave
