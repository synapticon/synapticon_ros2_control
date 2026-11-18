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

  set(_next_domain_id 80)
  # Give test `target` its own ROS domain and discovery on this machine only, so that it neither reaches nor hears
  # another ROS stack. ARGN: more environment entries of the test.
  function(isolate_launch_test target)
    set_property(TEST ${target} APPEND PROPERTY ENVIRONMENT
      "ROS_DOMAIN_ID=${_next_domain_id}" "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" ${ARGN})
    math(EXPR _next_domain_id "${_next_domain_id} + 1")
    set(_next_domain_id ${_next_domain_id} PARENT_SCOPE)
  endfunction()

  # Running without transmissions, and the example robot generator.
  ament_add_pytest_test(test_without_transmissions test/test_without_transmissions.py TIMEOUT 60)
  ament_add_pytest_test(test_generate_example_robot test/test_generate_example_robot.py TIMEOUT 60)
  ament_add_pytest_test(test_hil_test_gains test/test_hil_test_gains.py TIMEOUT 60)

  # The geared example on the mock HAL, with and without its transmission.
  add_launch_test(test/test_transmission_mock.py TARGET test_transmission_mock TIMEOUT 120)
  isolate_launch_test(test_transmission_mock)
  add_launch_test(test/test_disable_transmissions_mock.py TARGET test_disable_transmissions_mock TIMEOUT 120)
  isolate_launch_test(test_disable_transmissions_mock)

  # The hardware tests of the example launch, on simulated drives, in CSP and in impedance mode.
  foreach(mode csp impedance)
    if(mode STREQUAL "impedance")
      set(_hil_env "SYNAPTICON_HIL_HAL=mock_cia402" "SYNAPTICON_HIL_IMPEDANCE=1")
    else()
      set(_hil_env "SYNAPTICON_HIL_HAL=mock_cia402")
    endif()
    add_launch_test(test/hil_ros2_control_test.py TARGET test_ros2_control_path_mock_cia402_${mode} TIMEOUT 180)
    isolate_launch_test(test_ros2_control_path_mock_cia402_${mode} ${_hil_env})
    add_launch_test(test/hil_bringup_test.py TARGET test_bringup_mock_cia402_${mode} TIMEOUT 180)
    isolate_launch_test(test_bringup_mock_cia402_${mode} ${_hil_env} "SYNAPTICON_HIL_ROBOT=2dof")
  endforeach()
  # The multi-joint test on two simulated drives, with a short soak. The overrun limit is off: the timing of a
  # build machine says nothing about the drives.
  add_launch_test(test/hil_multi_joint_test.py TARGET test_multi_joint_mock_cia402 TIMEOUT 180)
  isolate_launch_test(test_multi_joint_mock_cia402 SYNAPTICON_HIL_HAL=mock_cia402 SYNAPTICON_HIL_ROBOT=2dof
    SYNAPTICON_HIL_SOAK_S=5 SYNAPTICON_HIL_MAX_OVERRUNS=100000)

  # For real drives, the same tests and their runner are installed but not run by colcon test: they need drives on
  # the bus, and one of them moves a joint.
  install(FILES test/hil_ros2_control_test.py test/hil_bringup_test.py test/hil_multi_joint_test.py
    DESTINATION share/${PROJECT_NAME}/test)
  install(PROGRAMS scripts/run_hil_tests.py DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
