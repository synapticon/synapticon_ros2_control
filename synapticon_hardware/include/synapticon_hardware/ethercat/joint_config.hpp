// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The drive settings of one joint for the EtherCAT HALs, taken from the joint's parameters.

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "synapticon_hardware/hal_config.hpp"

namespace synapticon_hardware::ethercat
{

/// @brief Address and run settings of the drive that moves one joint.
///
/// Each member comes from the joint parameter of the same name, except `name` and `slave_config_file` (parameter
/// `slave_config`). The comment of each member gives its value when the parameter is missing.
struct JointConfig
{
  /// Joint name, for log messages.
  std::string name;
  /// EtherCAT station alias, 0 when missing. With 0 the ring position alone finds the drive.
  uint16_t alias{ 0 };
  /// Ring position, counted from 0 at the first drive, 0 when missing.
  uint16_t position{ 0 };
  /// The drive's slave_config (PDO mapping and startup SDOs). Has no default.
  std::string slave_config_file;
  /// 0x6060 value, -6 when missing. 8 (CSP) and 9 (CSV) are sent to the drive. For -6 (impedance), the drive
  /// takes the 0x6060 default of its slave_config.
  int mode_of_operation{ -6 };
  /// Torque ceiling in Nm, sent in place of the drive's 0x2054 while the HAL runs. NaN when missing: the drive keeps
  /// its own.
  double max_joint_torque{ std::numeric_limits<double>::quiet_NaN() };
  /// True when missing. False: the drive is brought up and read, but never powered.
  bool energize{ true };
  /// False when missing. True for a drive mounted in reverse: position, velocity and torque change sign on the way
  /// between joint and drive. Gains and limits do not.
  bool invert{ false };
};

/// @brief The JointConfig of joint `name`, from its parameters `params`.
/// @throws std::invalid_argument when `slave_config` is missing or a value is not allowed. The message names the
/// parameter.
JointConfig joint_from(const std::string& name, const ParameterMap& params);

/// @brief An error message when two of `joints` have the same alias and position, so both would command one drive.
/// It names the first such pair. Nothing when all addresses differ.
std::optional<std::string> shared_drive_error(const std::vector<JointConfig>& joints);

}  // namespace synapticon_hardware::ethercat
