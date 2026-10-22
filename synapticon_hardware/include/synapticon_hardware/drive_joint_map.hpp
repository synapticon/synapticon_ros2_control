// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Which URDF joints have a drive of their own.

#include <cstddef>
#include <vector>

#include <hardware_interface/hardware_info.hpp>

#include "synapticon_hardware/hal_config.hpp"

namespace synapticon_hardware
{

/// @brief Which URDF joints have a drive, and which drive each one is.
///
/// Most joints have a drive. A joint that a transmission moves, without being one of its actuators, has none: its
/// motion comes from the actuators. Drives are numbered in the order of their joints in the URDF.
struct DriveJointMap
{
  std::vector<std::size_t> drive_to_joint;  ///< For each drive, the index of its URDF joint.
  std::vector<int> joint_to_drive;          ///< For each URDF joint, the index of its drive, or -1.

  std::size_t drive_count() const
  {
    return drive_to_joint.size();
  }
  bool has_drive(std::size_t joint) const
  {
    return joint_to_drive[joint] >= 0;
  }
};

/// Find the drives of the joints that `info` describes.
DriveJointMap build_drive_joint_map(const hardware_interface::HardwareInfo& info);

/// What the HAL gets: the `<hardware>` parameters, and the name and parameters of every joint with a drive.
HalConfig build_hal_config(const hardware_interface::HardwareInfo& info, const DriveJointMap& map);

}  // namespace synapticon_hardware
