# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

cmake_minimum_required(VERSION 3.8)
project(synapticon_hardware)

# colcon sets no build type, which means no optimisation at all. The cyclic exchange needs one.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(_deps hardware_interface pluginlib rclcpp rclcpp_lifecycle)

find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${_deps} soem_source)
  find_package(${dep} REQUIRED)
endforeach()

# SynapticonSystemInterface, the ros2_control hardware interface on the SOEM master.
add_library(${PROJECT_NAME} SHARED src/synapticon_interface.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${_deps})
target_link_libraries(${PROJECT_NAME} soem_source::soem)

pluginlib_export_plugin_description_file(hardware_interface synapticon_interface_plugin.xml)

# Runs one drive without ros2_control, to check the connection.
add_executable(torque_control_executable test/drive_check.cpp)
target_link_libraries(torque_control_executable soem_source::soem)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME} torque_control_executable
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${_deps} soem_source)

ament_package()
