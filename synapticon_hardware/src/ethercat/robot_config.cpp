// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Merging a robot-definition YAML into the configuration of an EtherCAT HAL.

#include "synapticon_hardware/ethercat/robot_config.hpp"

#include <exception>
#include <string>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

#include "synapticon_hardware/hal_config.hpp"

namespace synapticon_hardware::ethercat
{

namespace
{

using ParamMap = std::unordered_map<std::string, std::string>;

/// Copy every scalar entry of `node` except `name` into `params`, keeping the entries `params`
/// already has. A null or empty value counts as absent.
void merge_scalars(const YAML::Node& node, ParamMap& params)
{
  for (const auto& entry : node)
  {
    const std::string key = entry.first.as<std::string>();
    if (key == "name" || !entry.second.IsScalar() || params.count(key) != 0)
    {
      continue;
    }
    const std::string value = entry.second.as<std::string>();
    if (!value.empty())
    {
      params[key] = value;
    }
  }
}

/// Merge the file's `bus` map and its `joints` list into `config`, matching joints by name.
bool merge_file(const YAML::Node& root, HalConfig& config, std::string& error)
{
  if (!root.IsMap())
  {
    error = "it is not a YAML map";
    return false;
  }
  if (const YAML::Node bus = root["bus"]; bus && bus.IsMap())
  {
    merge_scalars(bus, config.hardware_parameters);
  }
  const YAML::Node entries = root["joints"];
  if (!entries || !entries.IsSequence())
  {
    error = "it has no 'joints' list";
    return false;
  }
  for (auto& joint : config.joints)
  {
    bool found = false;
    for (const auto& entry : entries)
    {
      if (entry.IsMap() && entry["name"] && entry["name"].as<std::string>() == joint.name)
      {
        merge_scalars(entry, joint.parameters);
        found = true;
        break;
      }
    }
    if (!found)
    {
      error = "it has no entry for joint '" + joint.name + "'";
      return false;
    }
  }
  return true;
}

/// Give every joint without its own slave_config or mode_of_operation the hardware-wide default.
void apply_joint_defaults(HalConfig& config)
{
  const std::string slave_config = param_or(config.hardware_parameters, DEFAULT_SLAVE_CONFIG_PARAM);
  const std::string mode = param_or(config.hardware_parameters, DEFAULT_MODE_PARAM);
  for (auto& joint : config.joints)
  {
    if (!slave_config.empty() && param_or(joint.parameters, "slave_config").empty())
    {
      joint.parameters["slave_config"] = slave_config;
    }
    if (!mode.empty() && param_or(joint.parameters, "mode_of_operation").empty())
    {
      joint.parameters["mode_of_operation"] = mode;
    }
  }
}

}  // namespace

bool apply_robot_config(HalConfig& config, std::string& error)
{
  const std::string path = param_or(config.hardware_parameters, ROBOT_CONFIG_PARAM);
  if (!path.empty())
  {
    try
    {
      if (!merge_file(YAML::LoadFile(path), config, error))
      {
        error = "robot_config '" + path + "': " + error;
        return false;
      }
    }
    catch (const std::exception& e)
    {
      error = "robot_config '" + path + "' could not be read: " + e.what();
      return false;
    }
  }
  apply_joint_defaults(config);
  return true;
}

}  // namespace synapticon_hardware::ethercat
