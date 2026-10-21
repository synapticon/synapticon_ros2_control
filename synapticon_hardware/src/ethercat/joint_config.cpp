// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Parsing of a joint's drive parameters, with a clear error for every value that cannot be used.

#include "synapticon_hardware/ethercat/joint_config.hpp"

#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "synapticon_hardware/ethercat/coe_objects.hpp"

namespace synapticon_hardware::ethercat
{

namespace
{
/// Parameter `what` as an EtherCAT address, which has 16 bits and no sign.
uint16_t address(const std::string& what, const std::string& text)
{
  if (text.empty() || text.front() == '-' || text.front() == '+')
  {
    throw std::invalid_argument(what + " must be a whole number in 0..65535, got '" + text + "'");
  }
  const unsigned long number = std::stoul(text);
  if (number > std::numeric_limits<uint16_t>::max())
  {
    throw std::invalid_argument(what + " must be a whole number in 0..65535, got " + text);
  }
  return static_cast<uint16_t>(number);
}

/// The mode of operation in `text`. Only the modes of `mode` pass.
int supported_mode(const std::string& text)
{
  switch (const int requested = std::stoi(text))
  {
    case mode::IMPEDANCE:
    case mode::CSP:
    case mode::CSV:
      return requested;
    default:
      throw std::invalid_argument("mode_of_operation " + text +
                                  " cannot be run. Allowed are -6 (impedance), 8 (CSP) and 9 (CSV).");
  }
}
}  // namespace

JointConfig joint_from(const std::string& name, const ParameterMap& params)
{
  JointConfig drive;
  drive.name = name;
  drive.slave_config_file = param_or(params, "slave_config");
  if (drive.slave_config_file.empty())
  {
    throw std::invalid_argument("slave_config is missing. Each drive needs the path of its slave_config file.");
  }
  drive.mode_of_operation = supported_mode(param_or(params, "mode_of_operation", "-6"));
  drive.alias = address("alias", param_or(params, "alias", "0"));
  drive.position = address("position", param_or(params, "position", "0"));
  drive.max_joint_torque = read_non_negative("max_joint_torque", "Nm", param_or(params, "max_joint_torque"));
  drive.stiffness = read_non_negative("stiffness", "Nm/rad", param_or(params, "stiffness"));
  drive.damping = read_non_negative("damping", "Nm s/rad", param_or(params, "damping"));
  drive.energize = read_flag(param_or(params, "energize", "true"));
  drive.invert = read_flag(param_or(params, "invert", "false"));
  return drive;
}

std::optional<std::string> shared_drive_error(const std::vector<JointConfig>& joints)
{
  std::map<std::pair<uint16_t, uint16_t>, const JointConfig*> owner;
  for (const JointConfig& joint : joints)
  {
    const auto [it, first] = owner.try_emplace({ joint.alias, joint.position }, &joint);
    if (!first)
    {
      return "two joints, '" + it->second->name + "' and '" + joint.name + "', address one drive (alias " +
             std::to_string(joint.alias) + ", position " + std::to_string(joint.position) +
             "). A joint without a 'position' parameter is at position 0.";
    }
  }
  return std::nullopt;
}

}  // namespace synapticon_hardware::ethercat
