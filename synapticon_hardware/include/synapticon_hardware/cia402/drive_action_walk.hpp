// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief What each DriveAction asks of the CiA 402 state machine: the state to walk to, the route, and whether a
/// HAL carries it out.

#include <array>
#include <cstddef>

#include "synapticon_hardware/cia402/state_machine.hpp"
#include "synapticon_hardware/drive_data.hpp"

namespace synapticon_hardware
{

/// @brief A CiA 402 walk: the state to end in, the route, and whether it may leave Fault.
///
/// The target alone is not enough, since three actions end in Switch On Disabled and differ in the route or in the
/// fault reset. A target of UNKNOWN means the action is no walk.
struct DriveWalk
{
  cia402::DeviceState target{ cia402::DeviceState::UNKNOWN };
  cia402::WalkPath path{ cia402::WalkPath::RAMPED };
  cia402::FaultExit fault_exit{ cia402::FaultExit::HOLD };
};

namespace detail
{
/// What one action asks for, and how a HAL treats it.
struct ActionMeaning
{
  DriveWalk walk;
  bool carried_out;   ///< A HAL runs the walk. SWITCH_ON and SHUTDOWN have one but no caller.
  bool needs_energy;  ///< Only an energisable joint can do it (Quick Stop Active needs Operation Enabled).
};

using cia402::DeviceState;
using cia402::FaultExit;
using cia402::WalkPath;

/// Indexed by the DriveAction code.
///
/// DISABLE and DISABLE_VOLTAGE end in the same state by different routes: DISABLE ramps down through Quick Stop
/// before the voltage goes, DISABLE_VOLTAGE cuts it at once. The ramp is the default, since cutting the torque of a
/// powered drive lets a loaded joint without brake drop. RESET_FAULT is the only action that leaves Fault: the
/// stop actions find a faulted drive de-energised already, and an enable needs a reset first.
constexpr std::array<ActionMeaning, 8> ACTION_MEANINGS = { {
    /* NONE            */ { {}, false, false },
    /* ENABLE          */ { { DeviceState::OPERATION_ENABLED, WalkPath::RAMPED }, true, true },
    /* SWITCH_ON       */ { { DeviceState::SWITCHED_ON, WalkPath::RAMPED }, false, false },
    /* SHUTDOWN        */ { { DeviceState::READY_TO_SWITCH_ON, WalkPath::RAMPED }, false, false },
    /* QUICK_STOP      */ { { DeviceState::QUICK_STOP_ACTIVE, WalkPath::RAMPED }, true, true },
    /* DISABLE         */ { { DeviceState::SWITCH_ON_DISABLED, WalkPath::RAMPED }, true, false },
    /* DISABLE_VOLTAGE */ { { DeviceState::SWITCH_ON_DISABLED, WalkPath::DIRECT }, true, false },
    /* RESET_FAULT     */ { { DeviceState::SWITCH_ON_DISABLED, WalkPath::RAMPED, FaultExit::RESET }, true, false },
} };

/// The meaning of `action`, or that of NONE for a code outside the table.
inline const ActionMeaning& meaning(DriveAction action)
{
  const auto code = static_cast<std::size_t>(action);
  return ACTION_MEANINGS[code < ACTION_MEANINGS.size() ? code : 0];
}
}  // namespace detail

/// @brief The walk `action` asks for.
inline DriveWalk drive_walk(DriveAction action)
{
  return detail::meaning(action).walk;
}

/// @brief Whether a HAL carries out `action` as a walk over several cycles.
inline bool is_walk_action(DriveAction action)
{
  return detail::meaning(action).carried_out;
}

/// @brief Whether `action` needs an energisable drive, so a monitored joint cannot do it.
inline bool needs_energised_drive(DriveAction action)
{
  return detail::meaning(action).needs_energy;
}

}  // namespace synapticon_hardware
