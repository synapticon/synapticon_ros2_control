# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

cmake_minimum_required(VERSION 3.8)
project(synapticon_ethercat_soem)

# colcon sets no build type, which means no optimisation at all. The real-time loop needs one.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(soem_source REQUIRED)
find_package(synapticon_hardware REQUIRED)
# Reads the drives' slave_config YAML.
find_package(yaml-cpp REQUIRED)

set(_deps pluginlib rclcpp synapticon_hardware)

add_library(${PROJECT_NAME} SHARED
  src/coe_encoding.cpp
  src/error_list.cpp
  src/ethercat_soem_hal.cpp
  src/process_data_mapping.cpp
  src/process_image.cpp
  src/slave_config.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  pluginlib::pluginlib rclcpp::rclcpp synapticon_hardware::synapticon_hardware soem_source::soem yaml-cpp::yaml-cpp)

# The HAL, loaded by SynapticonSystemInterface as `ethercat_soem`.
pluginlib_export_plugin_description_file(synapticon_hardware ethercat_soem_hal_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Starts a shell or a command with the raw socket capability the HAL needs.
install(PROGRAMS scripts/net_raw_shell.sh DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  # The slave_config, the process image and its mapping, the object dictionary encodings, SOEM's error list and the
  # HAL without a bus.
  foreach(test_name test_slave_config test_process_image test_process_data_mapping test_coe_encoding test_error_list
                    test_ethercat_soem_hal)
    ament_add_gmock(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME} ament_index_cpp::ament_index_cpp)
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  endforeach()

  # Hardware-in-the-loop suite, installed but not registered with ctest: it needs a drive and moves it.
  ament_add_gmock_executable(ethercat_soem_hil_test test/ethercat_soem_hil_test.cpp SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(ethercat_soem_hil_test ${PROJECT_NAME} ament_index_cpp::ament_index_cpp)
  install(TARGETS ethercat_soem_hil_test DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${_deps} soem_source yaml-cpp)
ament_package()
