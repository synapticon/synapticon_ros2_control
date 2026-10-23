# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

cmake_minimum_required(VERSION 3.8)
project(synapticon_bringup NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# The Python module the example launch uses.
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Running without transmissions, and the example robot generator.
  ament_add_pytest_test(test_without_transmissions test/test_without_transmissions.py TIMEOUT 60)
  ament_add_pytest_test(test_generate_example_robot test/test_generate_example_robot.py TIMEOUT 60)

  # The geared example on the mock HAL, with and without its transmission.
  add_launch_test(test/test_transmission_mock.py TIMEOUT 120)
  add_launch_test(test/test_disable_transmissions_mock.py TIMEOUT 120)

  # The hardware tests of the example launch, on simulated drives, in CSP and in impedance mode.
  foreach(mode csp impedance)
    if(mode STREQUAL "impedance")
      set(_hil_env "SYNAPTICON_HIL_HAL=mock_cia402;SYNAPTICON_HIL_IMPEDANCE=1")
    else()
      set(_hil_env "SYNAPTICON_HIL_HAL=mock_cia402")
    endif()
    add_launch_test(test/hil_ros2_control_test.py TARGET test_ros2_control_path_mock_cia402_${mode} TIMEOUT 180)
    set_tests_properties(test_ros2_control_path_mock_cia402_${mode} PROPERTIES ENVIRONMENT "${_hil_env}")
    add_launch_test(test/hil_bringup_test.py TARGET test_bringup_mock_cia402_${mode} TIMEOUT 180)
    set_tests_properties(test_bringup_mock_cia402_${mode} PROPERTIES ENVIRONMENT "${_hil_env};SYNAPTICON_HIL_ROBOT=2dof")
  endforeach()

  # For real drives, the same tests and their runner are installed but not run by colcon test: they need drives on
  # the bus, and one of them moves a joint.
  install(FILES test/hil_ros2_control_test.py test/hil_bringup_test.py DESTINATION share/${PROJECT_NAME}/test)
  install(PROGRAMS scripts/run_hil_tests.py DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
