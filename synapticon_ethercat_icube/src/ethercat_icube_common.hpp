// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Shared pieces of the EtherCAT ICube HAL sources: the logger, and the names of the code shared with the other
/// EtherCAT HALs.

#include "synapticon_hardware/ethercat/bus_config.hpp"
#include "synapticon_hardware/ethercat/joint_config.hpp"
#include "synapticon_hardware/ethercat/pdo_channels.hpp"
#include "synapticon_hardware/ethercat/robot_config.hpp"
#include "synapticon_hardware/hal_config.hpp"

namespace synapticon_ethercat_icube
{

using synapticon_hardware::param_or;
using synapticon_hardware::read_flag;
using synapticon_hardware::ethercat::apply_robot_config;
using synapticon_hardware::ethercat::bus_from;
using synapticon_hardware::ethercat::for_each_input_slot;
using synapticon_hardware::ethercat::for_each_output_slot;
using synapticon_hardware::ethercat::joint_from;
using synapticon_hardware::ethercat::JointConfig;
using synapticon_hardware::ethercat::shared_drive_error;

constexpr const char* LOGGER_NAME = "EthercatIcubeHal";

}  // namespace synapticon_ethercat_icube
