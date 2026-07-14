cmake_minimum_required(VERSION 3.16)
project(franka_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest QUIET)
find_package(hardware_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
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

add_library(
  local_safety_observation
  STATIC
  src/local_safety_observation.cpp
)
target_include_directories(
  local_safety_observation
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(local_safety_observation Eigen3::Eigen)

add_executable(
  local_safety_observation_node
  src/local_safety_observation_node.cpp
)
target_include_directories(
  local_safety_observation_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(local_safety_observation_node local_safety_observation)
ament_target_dependencies(
  local_safety_observation_node
  moveit_core
  moveit_ros_planning
  rclcpp
  std_msgs
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
  local_safety_observation
  EXPORT export_franka_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS local_safety_observation_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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

if(BUILD_TESTING AND ament_cmake_gtest_FOUND)
  ament_add_gtest(
    test_local_safety_observation
    tests/test_local_safety_observation.cpp
  )
  if(TARGET test_local_safety_observation)
    target_link_libraries(test_local_safety_observation local_safety_observation)
    target_include_directories(
      test_local_safety_observation
      PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  endif()
endif()

ament_package()
