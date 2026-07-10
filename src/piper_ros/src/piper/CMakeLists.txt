cmake_minimum_required(VERSION 3.8)
project(piper)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_python_install_package(scripts)

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/piper_single_ctrl_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
