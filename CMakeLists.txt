cmake_minimum_required(VERSION 3.8)
project(synapticon_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

# SOEM library
set(OS "linux")
set(OS_LIBS pthread rt)
file(GLOB OSAL_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/osal/osal.h ${CMAKE_CURRENT_SOURCE_DIR}/src/osal/${OS}/*.h)
file(GLOB OSAL_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/osal/${OS}/*.c)
file(GLOB OSHW_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/oshw/${OS}/*.h)
file(GLOB OSHW_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/oshw/${OS}/*.c)
file(GLOB SOEM_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/soem/*.h)
file(GLOB SOEM_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/soem/*.c)

add_library(synapticon_soem STATIC
  ${SOEM_SOURCES}
  ${OSAL_SOURCES}
  ${OSHW_SOURCES}
  ${OSHW_EXTRA_SOURCES})
# This is related to linking this STATIC library against SHARED libraries
# See https://stackoverflow.com/a/38297422/3499467
set_property(TARGET synapticon_soem PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(synapticon_soem ${OS_LIBS})
target_include_directories(synapticon_soem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/osal>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/osal/${OS}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/oshw/${OS}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/soem>
  $<INSTALL_INTERFACE:include/synapticon_ros2_control>
)

# ros2_control plugin library
add_library(
  synapticon_ros2_control
  SHARED
  src/synapticon_interface.cpp
)
target_include_directories(synapticon_ros2_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/osal>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/osal/${OS}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/oshw/${OS}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/soem>
)
target_link_libraries(
  synapticon_ros2_control
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp_lifecycle::rclcpp_lifecycle
  synapticon_soem
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface synapticon_ros2_control.xml)

# A simple executable to test Synapticon connection without ros2_control
add_executable(torque_control_executable src/torque_control_executable.cpp)
target_link_libraries(
  torque_control_executable
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp_lifecycle::rclcpp_lifecycle
  synapticon_soem
)

install(
  DIRECTORY include/
  DESTINATION include/synapticon_ros2_control
)
install(
  TARGETS synapticon_ros2_control synapticon_soem torque_control_executable
  EXPORT synapticon_soem
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES
  ${SOEM_HEADERS}
  ${OSAL_HEADERS}
  ${OSHW_HEADERS}
  DESTINATION include/synapticon_ros2_control
)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/synapticon_ros2_control
)
install(
  DIRECTORY bringup/launch bringup/config scripts
  DESTINATION share/synapticon_ros2_control
)
install(
  DIRECTORY test/
  DESTINATION share/synapticon_ros2_control/test
)

ament_export_targets(synapticon_soem HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  # Test that controller_manager launching succeeds
  ament_add_gtest_executable(test_controller_manager_integration
    test/test_controller_manager_integration.cpp)
  ament_target_dependencies(test_controller_manager_integration
    ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_ros_test(test/launch/simple_test_with_virtual_eth.launch.py
    TIMEOUT 30 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()

ament_package()
