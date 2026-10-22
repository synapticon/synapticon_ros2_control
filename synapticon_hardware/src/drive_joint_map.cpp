// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Which URDF joints have a drive, and what the HAL gets to know about them.

#include "synapticon_hardware/drive_joint_map.hpp"

#include <algorithm>
#include <string>
#include <unordered_set>

namespace synapticon_hardware
{

namespace
{
/// The joints that a transmission moves without being one of its actuators. They have no drive.
std::unordered_set<std::string> joints_without_drive(const hardware_interface::HardwareInfo& info)
{
  std::unordered_set<std::string> moved;
  std::unordered_set<std::string> actuators;
  for (const auto& transmission : info.transmissions)
  {
    std::for_each(transmission.joints.begin(), transmission.joints.end(),
                  [&](const auto& joint) { moved.insert(joint.name); });
    std::for_each(transmission.actuators.begin(), transmission.actuators.end(),
                  [&](const auto& actuator) { actuators.insert(actuator.name); });
  }
  for (const auto& name : actuators)
  {
    moved.erase(name);
  }
  return moved;
}
}  // namespace

DriveJointMap build_drive_joint_map(const hardware_interface::HardwareInfo& info)
{
  const auto without_drive = joints_without_drive(info);
  DriveJointMap map;
  map.joint_to_drive.reserve(info.joints.size());
  for (std::size_t joint = 0; joint < info.joints.size(); ++joint)
  {
    const bool driven = without_drive.find(info.joints[joint].name) == without_drive.end();
    map.joint_to_drive.push_back(driven ? static_cast<int>(map.drive_to_joint.size()) : -1);
    if (driven)
    {
      map.drive_to_joint.push_back(joint);
    }
  }
  return map;
}

HalConfig build_hal_config(const hardware_interface::HardwareInfo& info, const DriveJointMap& map)
{
  HalConfig config;
  config.hardware_parameters = info.hardware_parameters;
  for (const std::size_t joint : map.drive_to_joint)
  {
    const auto& described = info.joints[joint];
    config.joints.push_back(JointSpec{ described.name, described.parameters });
  }
  return config;
}

}  // namespace synapticon_hardware
