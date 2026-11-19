// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Merging a robot-definition YAML into the configuration of an EtherCAT HAL.

#include <array>
#include <string>

#include "synapticon_hardware/hal_config.hpp"

namespace synapticon_hardware::ethercat
{

/// Hardware parameter naming the robot-definition YAML.
inline constexpr const char* ROBOT_CONFIG_PARAM = "robot_config";

/// Hardware parameters with the defaults for joints that do not set their own.
inline constexpr const char* DEFAULT_SLAVE_CONFIG_PARAM = "default_slave_config";
inline constexpr const char* DEFAULT_MODE_PARAM = "mode_of_operation";

/// The robot definition's section with the values a command falls back to where it leaves them out.
inline constexpr const char* COMMAND_DEFAULTS_SECTION = "command_defaults";
/// The joint parameters that section gives a default for. In the hardware parameters, each one is prefixed with
/// `default_`.
inline constexpr std::array<const char*, 3> COMMAND_DEFAULT_KEYS{ "stiffness", "damping", "max_joint_torque" };
/// The section's switch, kept in the hardware parameters under its own name.
inline constexpr const char* KEEP_LAST_GIVEN_PARAM = "keep_last_given";

/// @brief The first step of apply_robot_config(): merge the YAML into `config`, without the defaults.
bool merge_robot_config(HalConfig& config, std::string& error);

/// @brief The second step of apply_robot_config(): give every joint without its own value of a joint parameter the
/// hardware-wide default of it.
void apply_joint_defaults(HalConfig& config);

/// @brief Fill `config` from the robot-definition YAML its `robot_config` parameter names, if any.
///
/// The YAML's `bus` entries fill the hardware parameters, and each `joints` entry fills the
/// parameters of the joint with the same name. The `command_defaults` entries become the hardware
/// parameters `default_<key>`, and `keep_last_given` keeps its name. A parameter `config` already
/// has wins over the file. Afterwards, a joint without a slave_config or mode_of_operation takes the
/// hardware parameters `default_slave_config` and `mode_of_operation`, and a joint without one of
/// COMMAND_DEFAULT_KEYS takes `default_<key>`.
/// @return false, with `error` set, when the file cannot be read, is not a robot definition, or has
///         no entry for one of the joints in `config`.
bool apply_robot_config(HalConfig& config, std::string& error);

}  // namespace synapticon_hardware::ethercat
