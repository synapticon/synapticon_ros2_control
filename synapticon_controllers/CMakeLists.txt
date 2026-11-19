# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

cmake_minimum_required(VERSION 3.8)
project(synapticon_controllers)

# colcon sets no build type, which means no optimisation at all. The real-time loop needs one.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(synapticon_control_msgs REQUIRED)
find_package(synapticon_hardware REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/motor_command_controller.cpp
  src/drive_controller.cpp
  src/drive_controller_status.cpp
  src/drive_controller_requests.cpp
  src/drive_controller_fault_reaction.cpp
  src/ethercat_bus_monitor_controller.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  controller_interface::controller_interface diagnostic_updater::diagnostic_updater
  hardware_interface::hardware_interface pluginlib::pluginlib rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools synapticon_hardware::synapticon_hardware ${diagnostic_msgs_TARGETS}
  ${std_msgs_TARGETS} ${synapticon_control_msgs_TARGETS})

pluginlib_export_plugin_description_file(
  controller_interface synapticon_controllers_plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  controller_interface diagnostic_msgs diagnostic_updater hardware_interface pluginlib rclcpp
  rclcpp_lifecycle realtime_tools std_msgs synapticon_control_msgs
  synapticon_hardware)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # Every controller loads in a controller manager.
  ament_add_gmock(test_load_controllers test/test_load_controllers.cpp)
  target_link_libraries(test_load_controllers
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )

  # The controllers on their own, against fake interfaces.
  foreach(test_name test_fault_reaction test_drive_controller_writes test_motor_command_controller
      test_ethercat_bus_monitor_controller test_realtime_diagnostics)
    ament_add_gmock(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
  endforeach()
endif()

ament_package()
