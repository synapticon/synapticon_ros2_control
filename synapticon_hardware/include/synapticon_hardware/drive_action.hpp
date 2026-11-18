// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The drive action a joint can request, with its configuration names.

#include <cstdint>
#include <optional>
#include <string_view>

namespace synapticon_hardware
{

/// @brief What a caller asks this drive's CiA 402 state machine to do.
///
/// Carried on a single `uint8` command interface, so two actions can never be asserted at once
/// and no priority rule is needed. A *change* of value is the request, so repeating a value does
/// not re-trigger it.
enum class DriveAction : uint8_t
{
  NONE = 0,         ///< No request this cycle.
  ENABLE,           ///< Walk to Operation Enabled.
  SWITCH_ON,        ///< Walk to Switched On.
  SHUTDOWN,         ///< Walk to Ready to Switch On.
  QUICK_STOP,       ///< Walk to Quick Stop Active and stay energised.
  DISABLE,          ///< Walk to Switch-on-Disabled, via the drive's quick stop ramp.
  DISABLE_VOLTAGE,  ///< Walk to Switch-on-Disabled directly, without a ramp.
  RESET_FAULT,      ///< Clear a Fault (Fault to Switch-on-Disabled). In Fault Reaction Active it waits for Fault.
  // Codes >= 128 are reserved for vendor actions declared in the drive's slave_config.
};

/// One entry of DRIVE_ACTION_NAMES: a DriveAction and its name in a configuration.
struct DriveActionName
{
  const char* name;
  DriveAction action;
};

/// Name and code table for DriveAction, so a new action reaches every configuration path at once.
inline constexpr DriveActionName DRIVE_ACTION_NAMES[] = {
  { "none", DriveAction::NONE },
  { "enable", DriveAction::ENABLE },
  { "switch_on", DriveAction::SWITCH_ON },
  { "shutdown", DriveAction::SHUTDOWN },
  { "quick_stop", DriveAction::QUICK_STOP },
  { "disable", DriveAction::DISABLE },
  { "disable_voltage", DriveAction::DISABLE_VOLTAGE },
  { "reset_fault", DriveAction::RESET_FAULT },
};

/// @brief Resolve a configured action name, or nothing if the name is unknown.
inline std::optional<DriveAction> action_from_string(std::string_view name)
{
  for (const auto& entry : DRIVE_ACTION_NAMES)
  {
    if (name == entry.name)
    {
      return entry.action;
    }
  }
  return std::nullopt;
}

/// @brief The configuration name of an action, for logging and diagnostics.
inline const char* action_to_string(DriveAction action)
{
  for (const auto& entry : DRIVE_ACTION_NAMES)
  {
    if (action == entry.action)
    {
      return entry.name;
    }
  }
  return "unknown";
}

}  // namespace synapticon_hardware
