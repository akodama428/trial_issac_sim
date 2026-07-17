cmake_minimum_required(VERSION 3.16)
project(franka_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest QUIET)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(
  franka_ros2_control
  SHARED
  src/isaac_sim_hardware_interface.cpp
)

target_include_directories(
  franka_ros2_control
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  franka_ros2_control
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface franka_ros2_control_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS franka_ros2_control
  EXPORT export_franka_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES franka_ros2_control_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(franka_ros2_control)
ament_export_targets(export_franka_ros2_control)

ament_package()
