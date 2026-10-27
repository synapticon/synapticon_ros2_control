// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Shared pieces of the EtherCAT ICube HAL sources: the logger, and the names of the code shared with the other
/// EtherCAT HALs.

#include "synapticon_hardware/hal_config.hpp"

// The code both EtherCAT HALs share.
namespace synapticon_hardware::ethercat
{
}  // namespace synapticon_hardware::ethercat

namespace synapticon_ethercat_icube
{

using namespace synapticon_hardware::ethercat;
using synapticon_hardware::param_or;
using synapticon_hardware::read_flag;

constexpr const char* LOGGER_NAME = "EthercatIcubeHal";

}  // namespace synapticon_ethercat_icube
