cmake_minimum_required(VERSION 3.8)
project(piper)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# C++ 코드 빌드
add_executable(planning_around_object cpp/planning_around_object.cpp)
target_include_directories(planning_around_object PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(planning_around_object
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
)

install(TARGETS planning_around_object
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python modules
ament_python_install_package(scripts)

# Python 스크립트 설치 (예시)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/piper_multi_ctrl_node.py
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/piper_single_ctrl_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

