cmake_minimum_required(VERSION 3.5)
project(piper_mujoco)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

install(PROGRAMS
  scripts/piper_mujoco_ctrl.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
