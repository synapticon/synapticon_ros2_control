# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

cmake_minimum_required(VERSION 3.8)
project(synapticon_ethercat_icube)

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

# The EtherCAT ICube HAL needs the ethercat_driver_ros2 stack at build time and an IgH EtherCAT master at
# runtime. It is built only with SYNAPTICON_HAL_ETHERCAT_ICUBE=ON (environment variable or CMake option), so a plain
# build needs neither. The package manifest's dependencies follow the same environment variable.
if("$ENV{SYNAPTICON_HAL_ETHERCAT_ICUBE}" STREQUAL "ON")
  set(_hal_default ON)
else()
  set(_hal_default OFF)
endif()
option(SYNAPTICON_HAL_ETHERCAT_ICUBE "Build the EtherCAT ICube HAL" ${_hal_default})

if(NOT SYNAPTICON_HAL_ETHERCAT_ICUBE)
  message(STATUS "${PROJECT_NAME}: built only with SYNAPTICON_HAL_ETHERCAT_ICUBE=ON, nothing to build")
  ament_package()
  return()
endif()

find_package(ethercat_driver REQUIRED)
# Found explicitly, because the exported target of ethercat_generic_cia402_drive refers to ethercat_generic_slave.
find_package(ethercat_generic_cia402_drive REQUIRED)
find_package(ethercat_generic_slave REQUIRED)
find_package(ethercat_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(synapticon_hardware REQUIRED)

set(_deps ethercat_driver ethercat_generic_cia402_drive ethercat_generic_slave ethercat_interface pluginlib rclcpp
  synapticon_hardware)

add_library(${PROJECT_NAME} SHARED
  src/ethercat_icube_hal.cpp
  src/ethercat_icube_setup.cpp
  src/synapticon_drive.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
# ethercat_driver and ethercat_interface export no CMake targets, only their include directories and libraries.
target_include_directories(${PROJECT_NAME} PUBLIC ${ethercat_driver_INCLUDE_DIRS} ${ethercat_interface_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC
  ethercat_generic_cia402_drive::ethercat_generic_cia402_drive ethercat_generic_slave::ethercat_generic_slave
  pluginlib::pluginlib rclcpp::rclcpp synapticon_hardware::synapticon_hardware ${ethercat_driver_LIBRARIES}
  ${ethercat_interface_LIBRARIES})

# The HAL, loaded by SynapticonSystemInterface as `ethercat_icube`, and the slave plugin the EtherCAT driver loads
# per drive.
pluginlib_export_plugin_description_file(synapticon_hardware ethercat_icube_hal_plugin.xml)
pluginlib_export_plugin_description_file(ethercat_interface synapticon_drive_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  # Setup parsing, without a bus.
  ament_add_gmock(test_ethercat_icube_hal test/test_ethercat_icube_hal.cpp)
  target_link_libraries(test_ethercat_icube_hal ${PROJECT_NAME})
  target_include_directories(test_ethercat_icube_hal PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

  # The bring-up checks and the PDO scaling, against a scripted object dictionary instead of a bus.
  ament_add_gmock(test_ethercat_icube_bringup test/test_ethercat_icube_bringup.cpp)
  target_link_libraries(test_ethercat_icube_bringup ${PROJECT_NAME})
  target_include_directories(test_ethercat_icube_bringup PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

  # Hardware-in-the-loop suite, installed but not registered with ctest: it needs a drive and moves it.
  ament_add_gmock_executable(ethercat_icube_hil_test test/ethercat_icube_hil_test.cpp SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(ethercat_icube_hil_test ${PROJECT_NAME} ament_index_cpp::ament_index_cpp)
  install(TARGETS ethercat_icube_hil_test DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(${_deps})
ament_package()
