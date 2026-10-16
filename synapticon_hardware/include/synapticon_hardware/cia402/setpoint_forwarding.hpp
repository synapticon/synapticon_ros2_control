// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Whether a motion controller's setpoints reach a drive: not while it is under a fault reaction.
///
/// From a stop action or a reported Fault until the next enable, the drive's buffer stops taking the controller's
/// setpoints, and the frames carry the last target the drive got. The controller may stream on, and the first
/// setpoint the drive gets after the enable is the one streamed at that moment.

#include "synapticon_hardware/cia402/state_machine.hpp"
#include "synapticon_hardware/drive_data.hpp"

namespace synapticon_hardware
{

/// @brief Whether `action` stops the drive, which ends the forwarding.
inline bool is_stop_action(DriveAction action)
{
  return action == DriveAction::QUICK_STOP || action == DriveAction::DISABLE || action == DriveAction::DISABLE_VOLTAGE;
}

/// @brief Whether a drive in `state` runs its own fault reaction, which stops it too.
inline bool is_fault_state(cia402::DeviceState state)
{
  return state == cia402::DeviceState::FAULT || state == cia402::DeviceState::FAULT_REACTION_ACTIVE;
}

/// @brief Whether one joint's setpoints are forwarded. A joint starts forwarding, like a drive never stopped.
struct SetpointForwarding
{
  bool forwarding{ true };
  /// The state reported last, to tell entering Fault from staying in it.
  cia402::DeviceState last_state{ cia402::DeviceState::UNKNOWN };

  /// An accepted action: a stop ends the forwarding, an enable resumes it, the others (a fault reset among them)
  /// leave it. Returns whether it changed.
  bool on_action(DriveAction action)
  {
    if (is_stop_action(action))
    {
      return change_to(false);
    }
    return change_to(forwarding || action == DriveAction::ENABLE);
  }

  /// A reported state: entering Fault ends the forwarding. No state resumes it, only an enable does, since a drive
  /// leaving Fault has not been told to run. Returns whether it changed.
  bool on_state(cia402::DeviceState state)
  {
    const bool entered_fault = is_fault_state(state) && !is_fault_state(last_state);
    last_state = state;
    return change_to(forwarding && !entered_fault);
  }

private:
  bool change_to(bool value)
  {
    const bool changed = value != forwarding;
    forwarding = value;
    return changed;
  }
};

}  // namespace synapticon_hardware
