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
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "synapticon_hardware/hal_config.hpp"

namespace synapticon_hardware::ethercat
{

namespace
{

using ParamMap = std::unordered_map<std::string, std::string>;

/// Copy every scalar entry of `node` except `name` into `params`, with `prefix` before its key, keeping the
/// entries `params` already has. A null or empty value counts as absent.
void merge_scalars(const YAML::Node& node, ParamMap& params, const std::string& prefix = {})
{
  for (const auto& entry : node)
  {
    const std::string key = entry.first.as<std::string>();
    const std::string target = key == KEEP_LAST_GIVEN_PARAM ? key : prefix + key;
    if (key == "name" || !entry.second.IsScalar() || params.count(target) != 0)
    {
      continue;
    }
    const std::string value = entry.second.as<std::string>();
    if (!value.empty())
    {
      params[target] = value;
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
  if (const YAML::Node defaults = root[COMMAND_DEFAULTS_SECTION]; defaults && defaults.IsMap())
  {
    merge_scalars(defaults, config.hardware_parameters, "default_");
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

}  // namespace

void apply_joint_defaults(HalConfig& config)
{
  std::vector<std::pair<std::string, std::string>> defaults{ { "slave_config", DEFAULT_SLAVE_CONFIG_PARAM },
                                                             { "mode_of_operation", DEFAULT_MODE_PARAM } };
  for (const char* key : COMMAND_DEFAULT_KEYS)
  {
    defaults.emplace_back(key, std::string("default_") + key);
  }
  for (const auto& [key, hardware_key] : defaults)
  {
    const std::string value = param_or(config.hardware_parameters, hardware_key);
    for (auto& joint : config.joints)
    {
      if (!value.empty() && param_or(joint.parameters, key).empty())
      {
        joint.parameters[key] = value;
      }
    }
  }
}

bool merge_robot_config(HalConfig& config, std::string& error)
{
  const std::string path = param_or(config.hardware_parameters, ROBOT_CONFIG_PARAM);
  if (path.empty())
  {
    return true;
  }
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
  return true;
}

bool apply_robot_config(HalConfig& config, std::string& error)
{
  if (!merge_robot_config(config, error))
  {
    return false;
  }
  apply_joint_defaults(config);
  return true;
}

}  // namespace synapticon_hardware::ethercat
