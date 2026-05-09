# C++ 코드가 있는 디렉토리 내 CMake 설정
add_executable(planning_around_object planning_around_object.cpp)
target_include_directories(planning_around_object PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  planning_around_object
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
)


