// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The values a command falls back to where it leaves out the stiffness, the damping or the torque limit.

#include "synapticon_hardware/command_fallback.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "synapticon_hardware/ethercat/robot_config.hpp"
#include "synapticon_hardware/hal_config.hpp"

namespace synapticon_hardware
{

namespace
{
/// The command fields that can fall back, each with its place in CommandFallback.
constexpr std::array<std::pair<double DriveCommand::*, double CommandFallback::*>, 3> FIELDS{ {
    { &DriveCommand::stiffness, &CommandFallback::stiffness },
    { &DriveCommand::damping, &CommandFallback::damping },
    { &DriveCommand::max_joint_torque, &CommandFallback::max_joint_torque },
} };

/// The fallback values in the parameters of joint `name`.
CommandFallback fallback_of(const std::string& name, const ParameterMap& params)
{
  CommandFallback values;
  values.stiffness = read_non_negative(name + ": stiffness", "Nm/rad", param_or(params, "stiffness"));
  values.damping = read_non_negative(name + ": damping", "Nm s/rad", param_or(params, "damping"));
  values.max_joint_torque = read_non_negative(name + ": max_joint_torque", "Nm", param_or(params, "max_joint_torque"));
  return values;
}
}  // namespace

FallbackConfig read_command_fallbacks(const hardware_interface::HardwareInfo& info)
{
  HalConfig all{ info.hardware_parameters, {} };
  for (const auto& joint : info.joints)
  {
    all.joints.push_back(JointSpec{ joint.name, joint.parameters });
  }
  if (std::string error; !ethercat::apply_robot_config(all, error))
  {
    throw std::invalid_argument(error);
  }
  FallbackConfig config;
  try
  {
    config.keep_last_given = read_flag(param_or(all.hardware_parameters, ethercat::KEEP_LAST_GIVEN_PARAM, "false"));
  }
  catch (const std::invalid_argument& error)
  {
    throw std::invalid_argument(std::string(ethercat::KEEP_LAST_GIVEN_PARAM) + ": " + error.what());
  }
  for (const auto& joint : all.joints)
  {
    config.joints.push_back(fallback_of(joint.name, joint.parameters));
  }
  return config;
}

void UnspecifiedCommandFill::configure(FallbackConfig config)
{
  config_ = std::move(config);
  reset();
}

void UnspecifiedCommandFill::reset()
{
  last_given_ = config_.joints;
}

const CommandFallback& UnspecifiedCommandFill::source(std::size_t joint) const
{
  return config_.keep_last_given ? last_given_[joint] : config_.joints[joint];
}

void UnspecifiedCommandFill::fill_joints(std::vector<DriveCommand>& commands)
{
  for (std::size_t joint = 0; joint < commands.size() && joint < last_given_.size(); ++joint)
  {
    for (const auto& [command_field, fallback_field] : FIELDS)
    {
      double& value = commands[joint].*command_field;
      if (std::isnan(value))
      {
        value = source(joint).*fallback_field;
      }
      else
      {
        last_given_[joint].*fallback_field = value;
      }
    }
  }
}

void UnspecifiedCommandFill::fill_drives(std::vector<DriveCommand>& commands,
                                         const std::vector<std::size_t>& drive_to_joint) const
{
  for (std::size_t drive = 0; drive < commands.size() && drive < drive_to_joint.size(); ++drive)
  {
    const std::size_t joint = drive_to_joint[drive];
    if (joint >= last_given_.size())
    {
      continue;
    }
    for (const auto& [command_field, fallback_field] : FIELDS)
    {
      double& value = commands[drive].*command_field;
      if (std::isnan(value))
      {
        value = source(joint).*fallback_field;
      }
    }
  }
}

bool UnspecifiedCommandFill::any_configured() const
{
  for (const CommandFallback& joint : config_.joints)
  {
    for (const auto& [command_field, fallback_field] : FIELDS)
    {
      if (!std::isnan(joint.*fallback_field))
      {
        return true;
      }
    }
  }
  return false;
}

}  // namespace synapticon_hardware
