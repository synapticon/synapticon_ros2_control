// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The values a command falls back to where it leaves out the stiffness, the damping or the torque limit.

#include <cstddef>
#include <limits>
#include <vector>

#include <hardware_interface/hardware_info.hpp>

#include "synapticon_hardware/drive_data.hpp"

namespace synapticon_hardware
{

/// @brief One joint's fallback values, NaN where none is configured.
struct CommandFallback
{
  double stiffness{ std::numeric_limits<double>::quiet_NaN() };         ///< [Nm/rad]
  double damping{ std::numeric_limits<double>::quiet_NaN() };           ///< [Nm s/rad]
  double max_joint_torque{ std::numeric_limits<double>::quiet_NaN() };  ///< [Nm]
};

/// @brief What UnspecifiedCommandFill works with.
struct FallbackConfig
{
  std::vector<CommandFallback> joints;  ///< One per URDF joint, in URDF order.
  bool keep_last_given{ false };        ///< See UnspecifiedCommandFill.
};

/// @brief The FallbackConfig of the joints in `info`.
///
/// A joint's value comes from its own parameter, else from its entry in the robot definition that the `robot_config`
/// hardware parameter names, else from the definition's `command_defaults` section.
/// @throws std::invalid_argument when the robot definition cannot be read or a value is not allowed.
FallbackConfig read_command_fallbacks(const hardware_interface::HardwareInfo& info);

/// @brief Fills the stiffness, damping and torque limit that a command leaves out (NaN).
///
/// By default a missing value gets the joint's fallback. A command then never inherits a value from an earlier,
/// unrelated command. With keep_last_given, a missing value repeats the joint's last given one instead, which starts
/// as the fallback after every reset(). A joint without a fallback keeps NaN, and the HAL leaves the drive's value.
class UnspecifiedCommandFill
{
public:
  void configure(FallbackConfig config);

  /// Forget the given values, so every joint starts from its fallback again.
  void reset();

  /// Fill the joint commands, one per URDF joint, before the transmissions. Allocates nothing.
  void fill_joints(std::vector<DriveCommand>& commands);

  /// Fill what is still missing after the transmissions, from the values of each drive's own joint.
  /// `drive_to_joint` gives the URDF joint of each drive command. Allocates nothing.
  void fill_drives(std::vector<DriveCommand>& commands, const std::vector<std::size_t>& drive_to_joint) const;

  /// Whether any joint has a fallback value.
  bool any_configured() const;

  /// Whether a missing value repeats the last given one.
  bool keeps_last_given() const
  {
    return config_.keep_last_given;
  }

private:
  /// The value a missing field of `joint` gets.
  const CommandFallback& source(std::size_t joint) const;

  FallbackConfig config_;
  std::vector<CommandFallback> last_given_;
};

}  // namespace synapticon_hardware
