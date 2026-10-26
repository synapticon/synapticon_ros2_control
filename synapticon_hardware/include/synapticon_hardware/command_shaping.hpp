// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief What the hardware sends when a controller leaves a command open.

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "synapticon_hardware/drive_data.hpp"

namespace synapticon_hardware
{

/// The command that keeps a joint where it is: its measured position, no velocity and no effort. Gains and the
/// torque limit are NaN, "not given", so the configured defaults apply.
inline DriveCommand hold_command(const DriveState& state)
{
  constexpr double NOT_GIVEN = std::numeric_limits<double>::quiet_NaN();
  DriveCommand hold{};
  hold.position = state.position;
  hold.stiffness = NOT_GIVEN;
  hold.damping = NOT_GIVEN;
  hold.max_joint_torque = NOT_GIVEN;
  return hold;
}

/// `command` with its gaps (NaN) filled: no position means "stay where you are", no velocity or effort means 0.
/// Gains and the torque limit keep NaN, "not given", which a later step fills with the configured defaults.
/// An infinite value is no gap but an invalid command. It passes through, so that the HAL refuses it.
inline DriveCommand filled_in(DriveCommand command, const DriveState& state)
{
  const auto or_else = [](double value, double fallback) { return std::isnan(value) ? fallback : value; };
  command.position = or_else(command.position, state.position);
  command.velocity = or_else(command.velocity, 0.0);
  command.effort = or_else(command.effort, 0.0);
  return command;
}

/// The hold command of every joint. Used at activation, so that a drive that gets power later stays put.
inline void seed_hold_commands(const std::vector<DriveState>& states, std::vector<DriveCommand>& commands)
{
  commands.resize(states.size());
  std::transform(states.begin(), states.end(), commands.begin(), hold_command);
}

/// The commands of every joint with their gaps filled, written to `sanitised`. It allocates nothing once
/// `sanitised` has the right size, so it is safe in the control loop.
inline void sanitise_commands(const std::vector<DriveCommand>& commands, const std::vector<DriveState>& states,
                              std::vector<DriveCommand>& sanitised)
{
  sanitised.resize(commands.size());
  std::transform(commands.begin(), commands.end(), states.begin(), sanitised.begin(), filled_in);
}

}  // namespace synapticon_hardware
