# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

cmake_minimum_required(VERSION 3.8)
project(synapticon_description NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes ros2_control urdf DESTINATION share/${PROJECT_NAME})

ament_package()
