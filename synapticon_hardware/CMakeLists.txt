# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

cmake_minimum_required(VERSION 3.8)
project(synapticon_hardware)

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
find_package(ament_index_cpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(transmission_interface REQUIRED)
# For the robot definition YAML that the EtherCAT HALs read.
find_package(yaml-cpp REQUIRED)

set(_sources src/cia402/cia402_hal.cpp src/command_fallback.cpp src/drive_joint_map.cpp src/ethercat/bus_config.cpp
  src/ethercat/drive_checks.cpp src/ethercat/drive_units.cpp src/ethercat/ethercat_pdo_hal.cpp
  src/ethercat/joint_config.cpp src/ethercat/robot_config.cpp src/handle_access_report.cpp src/interface_binding.cpp
  src/mock/mock_cia402_hal.cpp src/mock/mock_hal.cpp src/mock/simulated_drive.cpp src/synapticon_interface.cpp
  src/transmission_set.cpp)

# The library: hardware interface, HAL interface and everything the HALs share.
add_library(${PROJECT_NAME} SHARED ${_sources})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  hardware_interface::hardware_interface pluginlib::pluginlib rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle
  transmission_interface::transmission_interface yaml-cpp::yaml-cpp)

# The plugins: the hardware interface for ros2_control, and the HALs it can load.
pluginlib_export_plugin_description_file(
  hardware_interface synapticon_interface_plugin.xml)
pluginlib_export_plugin_description_file(
  synapticon_hardware synapticon_hal_plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# The slave_config generator, and the example slave_config it made.
install(PROGRAMS scripts/esi_to_slave_config.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# drive_check: try one drive through any HAL, without ros2_control.
add_executable(drive_check src/drive_check.cpp)
target_link_libraries(drive_check ${PROJECT_NAME} ament_index_cpp::ament_index_cpp)
install(TARGETS drive_check DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  transmission_interface
  yaml-cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(synapticon_control_msgs REQUIRED)

  # Add test/<path>.cpp as a gmock test with the file's name. HEADERS: it needs only the headers, not the library.
  # USES: the other targets it needs.
  function(hardware_test path)
    cmake_parse_arguments(test "HEADERS" "" "USES" ${ARGN})
    get_filename_component(name ${path} NAME)
    ament_add_gmock(${name} test/${path}.cpp)
    if(test_HEADERS)
      target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    else()
      target_link_libraries(${name} ${PROJECT_NAME})
    endif()
    if(test_USES)
      target_link_libraries(${name} ${test_USES})
    endif()
  endfunction()

  # The parts that live in headers only.
  hardware_test(cia402/test_state_machine HEADERS)
  hardware_test(test_command_shaping HEADERS)
  hardware_test(test_hal_config HEADERS)
  hardware_test(cia402/test_setpoint_forwarding HEADERS)
  hardware_test(ethercat/test_firmware_version HEADERS)
  # The parts in the library.
  hardware_test(test_interface_binding)
  hardware_test(test_link_health)
  hardware_test(test_hal_bench)
  hardware_test(test_handle_access_report)
  hardware_test(test_drive_joint_map)
  hardware_test(test_command_fallback)
  hardware_test(test_transmission_set)
  hardware_test(cia402/test_drive_action_walk USES ${synapticon_control_msgs_TARGETS})
  hardware_test(cia402/test_cia402_hal_failures)
  hardware_test(ethercat/test_bus_config)
  hardware_test(ethercat/test_drive_checks)
  hardware_test(ethercat/test_drive_units)
  hardware_test(ethercat/test_ethercat_pdo_hal)
  hardware_test(ethercat/test_joint_config)
  hardware_test(ethercat/test_robot_config)
  # The slave_config generator, against a small ESI file built from the example slave_config.
  ament_add_pytest_test(test_esi_to_slave_config test/ethercat/test_esi_to_slave_config.py TIMEOUT 60)
  # The mock HALs, the simulated drive, and the hardware interface on top of the mock.
  hardware_test(test_mock_hal)
  hardware_test(test_mock_cia402_hal)
  target_include_directories(test_mock_cia402_hal PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  hardware_test(test_synapticon_interface)
  # drive_check on a simulated drive.
  foreach(check read timing hold move)
    ament_add_test(drive_check_${check} GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 60
      COMMAND $<TARGET_FILE:drive_check> ${check} --hal mock_cia402 --duration 1)
  endforeach()
  ament_add_test(drive_check_sigterm GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 60
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test/test_drive_check_sigterm.sh $<TARGET_FILE:drive_check>)
  ament_add_test(drive_check_options GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 60
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test/test_drive_check_options.sh $<TARGET_FILE:drive_check>)

  # The drive action suite of the hardware tests, on simulated drives, in real time.
  ament_add_gmock(test_mock_cia402_drive_actions test/test_mock_cia402_drive_actions.cpp
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 600)
  target_link_libraries(test_mock_cia402_drive_actions ${PROJECT_NAME})

  # How long the interface layer takes for many joints, without a bus. A tool to run by hand: timing limits in a test
  # would fail at random on a machine without real-time scheduling.
  add_executable(timing_interface_access test/timing_interface_access.cpp)
  target_link_libraries(timing_interface_access ${PROJECT_NAME})
endif()

ament_package()
