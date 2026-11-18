// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief What the hardware sends when a controller leaves a command open.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "synapticon_hardware/drive_data.hpp"

namespace synapticon_hardware
{

/// A command that gives nothing: every value NaN, "not given", and no drive action. Where the hardware fills it in,
/// the joint holds, without velocity or effort, with the configured gains and torque limit.
inline DriveCommand not_given_command()
{
  constexpr double NOT_GIVEN = std::numeric_limits<double>::quiet_NaN();
  DriveCommand command{};
  command.position = NOT_GIVEN;
  command.velocity = NOT_GIVEN;
  command.effort = NOT_GIVEN;
  command.stiffness = NOT_GIVEN;
  command.damping = NOT_GIVEN;
  command.max_joint_torque = NOT_GIVEN;
  command.drive_action = DriveAction::NONE;
  return command;
}

/// Every command set to not_given_command(). Used at activation, so that a joint holds until a controller commands
/// it. Keeps the size of `commands`, so handles that point into it stay valid.
inline void seed_not_given(std::vector<DriveCommand>& commands)
{
  std::fill(commands.begin(), commands.end(), not_given_command());
}

/// @brief Where each joint holds when its command leaves the position open (NaN).
///
/// While a joint's drive has no power, the hold position follows the measured position, so a drive that gets power
/// holds where the joint is, also after it was moved by hand. Once the drive has power, the hold position stays at
/// the last valid position command, so a loaded joint does not sink. Every valid position command moves it there.
// The rule for missing and invalid values may change with issue #88.
class HoldReference
{
public:
  /// Start from the measured positions. Allocates, so call it outside the control loop.
  void reset(const std::vector<DriveState>& states)
  {
    positions_.resize(states.size());
    std::transform(states.begin(), states.end(), positions_.begin(),
                   [](const DriveState& state) { return state.position; });
  }

  /// One cycle. `powered` has one flag per joint: whether its drive may have power. Allocates nothing.
  void update(const std::vector<DriveCommand>& commands, const std::vector<DriveState>& states,
              const std::vector<uint8_t>& powered)
  {
    for (std::size_t j = 0; j < positions_.size(); ++j)
    {
      if (std::isfinite(commands[j].position))
      {
        positions_[j] = commands[j].position;
      }
      else if (powered[j] == 0u)
      {
        positions_[j] = states[j].position;
      }
    }
  }

  /// The hold position of every joint.
  const std::vector<double>& positions() const
  {
    return positions_;
  }

private:
  std::vector<double> positions_;
};

/// `command` with its gaps (NaN) filled: no position means "hold at `hold_position`", no velocity or effort means 0.
/// Gains and the torque limit keep NaN, "not given", which a later step fills with the configured defaults.
/// An infinite value is no gap but an invalid command. It passes through, so that the HAL refuses it.
// The rule for missing and invalid values may change with issue #88.
inline DriveCommand filled_in(DriveCommand command, double hold_position)
{
  const auto or_else = [](double value, double fallback) { return std::isnan(value) ? fallback : value; };
  command.position = or_else(command.position, hold_position);
  command.velocity = or_else(command.velocity, 0.0);
  command.effort = or_else(command.effort, 0.0);
  return command;
}

/// The commands of every joint with their gaps filled, written to `sanitised`. `hold_positions` has one entry per
/// joint, see HoldReference. It allocates nothing once `sanitised` has the right size, so it is safe in the control
/// loop.
inline void sanitise_commands(const std::vector<DriveCommand>& commands, const std::vector<double>& hold_positions,
                              std::vector<DriveCommand>& sanitised)
{
  sanitised.resize(commands.size());
  std::transform(commands.begin(), commands.end(), hold_positions.begin(), sanitised.begin(), filled_in);
}

}  // namespace synapticon_hardware
