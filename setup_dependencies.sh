#!/bin/bash

echo "🔄 시스템 패키지 업데이트 중..."
sudo apt update
sudo apt dist-upgrade -y

echo "🔄 rosdep 설정 확인 및 업데이트..."
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init
fi
rosdep update

echo "📦 필수 패키지 설치 중..."
sudo apt install -y can-utils ethtool iproute2 python3-rosdep python3-colcon-common-extensions python3-colcon-mixin python3-vcstool

echo "🤖 ROS2 관련 패키지 설치 중..."
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager
sudo apt install -y ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-xacro
sudo apt install -y ros-humble-moveit ros-humble-moveit-visual-tools

echo "🔧 CMake 최신 버전 설치 중..."
sudo apt install -y cmake

echo "🐍 Python 패키지 설치 중..."
pip3 install --upgrade pip
if [ -f "requirements.txt" ]; then
    pip3 install -r requirements.txt
else
    echo "⚠️ requirements.txt 파일이 존재하지 않습니다. Python 패키지 설치를 건너뜁니다."
fi

echo "✅ 모든 의존성이 설치되었습니다!"
