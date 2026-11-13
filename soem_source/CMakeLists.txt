cmake_minimum_required(VERSION 3.8)
project(soem_source C)

# colcon sets no build type, which means no optimisation at all. The real-time loop needs one.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)

# The SOEM library for Linux from the vendored sources, built as upstream builds it: a static library, here with
# position-independent code so that it links into the plugin libraries.
set(_os linux)
file(GLOB _sources src/soem/*.c src/osal/${_os}/*.c src/oshw/${_os}/*.c)
file(GLOB _headers src/soem/*.h src/osal/osal.h src/osal/${_os}/*.h src/oshw/${_os}/*.h)

add_library(soem STATIC ${_sources})
set_target_properties(soem PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(soem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/soem>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/osal>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/osal/${_os}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/oshw/${_os}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(soem PUBLIC pthread rt)

# The headers include each other by file name, so they are installed into one folder.
install(FILES ${_headers} DESTINATION include/${PROJECT_NAME})
install(
  TARGETS soem
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
