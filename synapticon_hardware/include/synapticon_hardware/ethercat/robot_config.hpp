// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Merging a robot-definition YAML into the configuration of an EtherCAT HAL.

#include <string>

#include "synapticon_hardware/hal_config.hpp"

namespace synapticon_hardware::ethercat
{

/// Hardware parameter naming the robot-definition YAML.
inline constexpr const char* ROBOT_CONFIG_PARAM = "robot_config";

/// Hardware parameters with the defaults for joints that do not set their own.
inline constexpr const char* DEFAULT_SLAVE_CONFIG_PARAM = "default_slave_config";
inline constexpr const char* DEFAULT_MODE_PARAM = "mode_of_operation";

/// @brief Fill `config` from the robot-definition YAML its `robot_config` parameter names, if any.
///
/// The YAML's `bus` entries fill the hardware parameters, and each `joints` entry fills the
/// parameters of the joint with the same name. A parameter `config` already has wins over the file.
/// Afterwards, a joint without a slave_config or mode_of_operation takes the hardware parameters
/// `default_slave_config` and `mode_of_operation`.
/// @return false, with `error` set, when the file cannot be read, is not a robot definition, or has
///         no entry for one of the joints in `config`.
bool apply_robot_config(HalConfig& config, std::string& error);

}  // namespace synapticon_hardware::ethercat
