// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief EthercatIcubeHal's configuration: the bus and joint parameters of the description, parsed and checked.

#include <charconv>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "synapticon_hardware/ethercat/bus_config.hpp"
#include "synapticon_hardware/ethercat/joint_config.hpp"
#include "ethercat_icube_common.hpp"
#include "synapticon_hardware/ethercat/robot_config.hpp"
#include "synapticon_ethercat_icube/ethercat_icube_hal.hpp"

namespace synapticon_ethercat_icube
{

namespace
{
using Parameters = std::unordered_map<std::string, std::string>;

/// The IgH master index written in `text`: digits only.
/// @throws std::invalid_argument for anything else.
unsigned int master_index(const std::string& text)
{
  unsigned int index = 0;
  const char* const end = text.data() + text.size();
  const auto [stop, error] = std::from_chars(text.data(), end, index);
  if (error != std::errc{} || stop != end)
  {
    throw std::invalid_argument("master_id '" + text + "' is not a master index such as 0 or 1");
  }
  return index;
}

/// The shared bus settings, and the IgH master from `master_id` (the first master when it is missing).
EthercatConfig icube_bus_from(const Parameters& hardware)
{
  EthercatConfig bus{ bus_from(hardware) };
  if (const auto it = hardware.find("master_id"); it != hardware.end())
  {
    bus.master_id = master_index(it->second);
  }
  return bus;
}

/// The joint `name` with its drive from the shared joint parameters, and the slave plugin to load.
EthercatJointConfig ethercat_joint_from(const std::string& name, const Parameters& params)
{
  EthercatJointConfig joint{ joint_from(name, params) };
  if (const std::string plugin = param_or(params, "plugin"); !plugin.empty())
  {
    joint.plugin = plugin;
  }
  return joint;
}
}  // namespace

bool EthercatIcubeHal::begin_setup()
{
  if (bus_configured_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "setup() refused: the HAL is brought up, disconnect() first.");
    return false;
  }
  set_up_ = false;
  config_ = EthercatConfig{};
  joints_.clear();
  return true;
}

bool EthercatIcubeHal::setup(EthercatConfig config, std::vector<EthercatJointConfig> joints)
{
  const auto logger = rclcpp::get_logger(LOGGER_NAME);
  if (!begin_setup())
  {
    return false;
  }
  if (joints.empty())
  {
    RCLCPP_ERROR(logger, "setup() refused: the configuration has no joints.");
    return false;
  }
  if (const auto shared = shared_drive_error({ joints.begin(), joints.end() }))
  {
    RCLCPP_ERROR(logger, "setup() refused: %s", shared->c_str());
    return false;
  }
  // The bring-up reads its SDOs by absolute ring position, which an alias would shift to another drive.
  for (const auto& joint : joints)
  {
    if (joint.alias != 0)
    {
      RCLCPP_ERROR(logger, "setup() refused: joint '%s' has alias %u, this HAL addresses drives by ring position only.",
                   joint.name.c_str(), joint.alias);
      return false;
    }
  }
  config_ = std::move(config);
  joints_ = std::move(joints);
  set_up_ = true;
  return true;
}

bool EthercatIcubeHal::setup(const HalConfig& description)
{
  const auto logger = rclcpp::get_logger(LOGGER_NAME);
  if (!begin_setup())
  {
    return false;
  }
  HalConfig config = description;
  std::string error;
  if (!apply_robot_config(config, error))
  {
    RCLCPP_ERROR(logger, "setup(): %s", error.c_str());
    return false;
  }

  EthercatConfig bus;
  try
  {
    bus = icube_bus_from(config.hardware_parameters);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "Bus parameter rejected: %s", e.what());
    return false;
  }

  std::vector<EthercatJointConfig> joints;
  for (const auto& joint : config.joints)
  {
    try
    {
      joints.push_back(ethercat_joint_from(joint.name, joint.parameters));
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger, "Joint '%s': %s", joint.name.c_str(), e.what());
      return false;
    }
  }
  return setup(std::move(bus), std::move(joints));
}

}  // namespace synapticon_ethercat_icube
