cmake_minimum_required(VERSION 3.16)
project(franka_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest QUIET)
find_package(control_msgs QUIET)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(trajectory_msgs QUIET)
find_package(yaml_cpp_vendor REQUIRED)

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

add_library(
  motion_command_executor_core
  STATIC
  src/motion_command_executor_core.cpp
)

target_include_directories(
  motion_command_executor_core
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(motion_command_executor_core yaml-cpp)

if(control_msgs_FOUND AND trajectory_msgs_FOUND)
  add_executable(
    motion_command_executor_node
    src/motion_command_executor_node.cpp
  )

  target_include_directories(
    motion_command_executor_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(
    motion_command_executor_node
    motion_command_executor_core
  )

  ament_target_dependencies(
    motion_command_executor_node
    control_msgs
    rclcpp
    rclcpp_action
    std_msgs
    trajectory_msgs
  )
else()
  message(WARNING "control_msgs or trajectory_msgs not found; skipping motion_command_executor_node build.")
endif()

pluginlib_export_plugin_description_file(hardware_interface franka_ros2_control_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS franka_ros2_control
  motion_command_executor_core
  EXPORT export_franka_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(TARGET motion_command_executor_node)
  install(
    TARGETS motion_command_executor_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

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
    test_motion_command_executor_core
    tests/test_motion_command_executor_core.cpp
  )
  if(TARGET test_motion_command_executor_core)
    target_link_libraries(test_motion_command_executor_core motion_command_executor_core)
    target_include_directories(
      test_motion_command_executor_core
      PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  endif()
endif()

ament_package()
