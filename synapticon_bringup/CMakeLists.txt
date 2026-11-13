# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

cmake_minimum_required(VERSION 3.8)
project(synapticon_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(ros_testing REQUIRED)

  # The controller manager comes up with the 2 DOF configuration on a dummy network interface.
  install(PROGRAMS test/setup_test_environment.sh DESTINATION share/${PROJECT_NAME}/test)
  ament_add_gtest_executable(test_controller_manager_integration test/test_controller_manager_integration.cpp)
  target_link_libraries(test_controller_manager_integration rclcpp::rclcpp)
  add_ros_test(test/simple_test_with_virtual_eth.launch.py
    TIMEOUT 30 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()

ament_package()
