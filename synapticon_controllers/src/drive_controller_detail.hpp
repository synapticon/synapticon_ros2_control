// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Tables and helpers of the DriveController sources: state and mode names, request precedence, constants.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "synapticon_controllers/drive_controller.hpp"
#include "synapticon_hardware/cia402/state_machine.hpp"

namespace synapticon_controllers::drive_controller_detail
{
using cia402::DeviceState;

/// The diagnostics state interfaces of every joint, in the order of JointSlots::diagnostics.
constexpr std::array<const char*, 4> DIAGNOSTICS_FIELDS = { "dc_link_voltage", "temperature_inverter",
                                                            "temperature_core", "motor_current" };

/// One control period at `rate_hz`, rounded up to whole milliseconds. Zero when the rate is not known (0).
inline std::chrono::milliseconds control_period(unsigned int rate_hz)
{
  return rate_hz == 0 ? std::chrono::milliseconds(0) : std::chrono::milliseconds((1000 + rate_hz - 1) / rate_hz);
}

/// How long a service callback waits for update() to answer a fault reaction query: a few control periods, since
/// the answer normally comes within one, and never less than 100 ms.
inline std::chrono::milliseconds answer_timeout(unsigned int rate_hz)
{
  constexpr int PERIODS = 3;
  return std::max(std::chrono::milliseconds(100), PERIODS * control_period(rate_hz));
}

/// How often ~/drive_status repeats every drive's state.
constexpr std::chrono::seconds DRIVE_STATUS_PERIOD{ 1 };

/// Minimum gap between two throttled reports of the update loop.
constexpr int LOG_THROTTLE_MS = 5000;

/// EtherCAT AL state OP. A slave's PDO data is current only while it is online and in OP.
constexpr uint8_t AL_STATE_OP = 8;

/// The reported state of a CiA 402 device_state value. The CiA 402 states keep their value.
inline ReportedState reported_from_device_state(uint8_t device_state)
{
  static_assert(static_cast<int>(DeviceState::NOT_READY_TO_SWITCH_ON) == 1 && static_cast<int>(DeviceState::FAULT) == 8);
  if (device_state >= static_cast<uint8_t>(DeviceState::NOT_READY_TO_SWITCH_ON) &&
      device_state <= static_cast<uint8_t>(DeviceState::FAULT))
  {
    return static_cast<ReportedState>(device_state - 1);
  }
  return ReportedState::UNDEFINED;
}

/// The readable name of a reported state, for the diagnostics message and the drive status.
inline const char* state_label(ReportedState state)
{
  switch (state)
  {
    case ReportedState::DISCONNECTED:
      return "Disconnected";
    case ReportedState::UNDEFINED:
      return "Undefined";
    default:
      return cia402::device_state_name(static_cast<DeviceState>(static_cast<int>(state) + 1));
  }
}

/// The diagnostics level of a reported state: OK only in Operation Enabled, ERROR in Fault or off the bus.
inline uint8_t state_level(ReportedState state)
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  if (state == ReportedState::FAULT || state == ReportedState::DISCONNECTED)
  {
    return Status::ERROR;
  }
  return state == ReportedState::OPERATION_ENABLED ? Status::OK : Status::WARN;
}

/// A value and its name, for the lookup tables below.
template <typename T>
struct Named
{
  T value;
  const char* name;
};

template <typename T, std::size_t N>
const char* name_in(const std::array<Named<T>, N>& table, T value, const char* fallback)
{
  const auto it = std::find_if(table.begin(), table.end(), [value](const auto& entry) { return entry.value == value; });
  return it == table.end() ? fallback : it->name;
}

/// The name of `value` in `table`, or its number.
template <typename T, std::size_t N>
std::string name_or_code(const std::array<Named<T>, N>& table, T value)
{
  const char* name = name_in(table, value, nullptr);
  return name ? std::string(name) : "code " + std::to_string(static_cast<int>(value));
}

constexpr std::array<Named<int8_t>, 9> MODE_NAMES = { {
    { 0, "none" },
    { 1, "profile position" },
    { 3, "profile velocity" },
    { 4, "profile torque" },
    { 6, "homing" },
    { 7, "interpolated position" },
    { 8, "CSP" },
    { 9, "CSV" },
    { -6, "impedance" },
} };

constexpr std::array<Named<uint8_t>, 4> AL_STATE_NAMES = { {
    { 1, "INIT" },
    { 2, "PREOP" },
    { 4, "SAFEOP" },
    { 8, "OP" },
} };

constexpr std::array<Named<JointCondition>, 4> CONDITION_NAMES = { {
    { JointCondition::HEALTHY, "healthy" },
    { JointCondition::FAULTED, "fault" },
    { JointCondition::UNREACHABLE, "unreachable" },
    { JointCondition::FOLLOWING_ERROR, "following_error" },
} };

/// The error_report text (0x203F:1). The state interface carries its 8 bytes as the bit pattern of a double, padded
/// with NUL characters.
inline std::string decode_error_report(double raw)
{
  char bytes[sizeof(raw) + 1] = {};
  std::memcpy(bytes, &raw, sizeof(raw));
  return std::string(bytes);
}

/// The answer to a request that names a joint this controller does not have.
inline std::string unknown_joint_message(const std::string& joint)
{
  return "Refused: no joint named '" + joint + "' in this controller.";
}

/// Whether the hardware carries out this action. The services refuse the other known names up front.
inline bool is_forwarded_action(DriveAction action)
{
  return std::find(REQUESTABLE_ACTIONS.begin(), REQUESTABLE_ACTIONS.end(), action) != REQUESTABLE_ACTIONS.end();
}

/// The rank of an action when several requests name the same joint in one cycle, higher wins. It depends on the
/// action and not on who asked, so a request to stop never loses to a request to run. NONE ranks lowest.
inline int precedence(DriveAction action)
{
  // Indexed by DriveAction: NONE, ENABLE, SWITCH_ON, SHUTDOWN, QUICK_STOP, DISABLE, DISABLE_VOLTAGE, RESET_FAULT.
  constexpr std::array<int, 8> RANKS = { -1, 1, 2, 3, 4, 5, 6, 0 };
  const auto code = static_cast<std::size_t>(action);
  return code < RANKS.size() ? RANKS[code] : -1;
}

/// True when a request for `joints` names `joint`. A request that names no joint is for all of them.
inline bool names_joint(const std::vector<std::string>& joints, const std::string& joint)
{
  return joints.empty() || std::find(joints.begin(), joints.end(), joint) != joints.end();
}

/// The joints of two requests for the same action together. A request that names no joint is for all of them.
inline std::vector<std::string> merged_joints(const std::vector<std::string>& first,
                                              const std::vector<std::string>& second)
{
  if (first.empty() || second.empty())
  {
    return {};
  }
  std::vector<std::string> joints = first;
  for (const auto& joint : second)
  {
    if (std::find(joints.begin(), joints.end(), joint) == joints.end())
    {
      joints.push_back(joint);
    }
  }
  return joints;
}

using RequestSnapshot = std::array<std::shared_ptr<ActionRequest>, NUM_REQUEST_SLOTS>;
/// One action per request source, NONE where the source asks nothing of the joint.
using SourceActions = std::array<DriveAction, NUM_REQUEST_SLOTS>;

/// What each source's request asks of `joint`.
inline SourceActions requested_of(const RequestSnapshot& requests, const std::string& joint)
{
  SourceActions actions;
  actions.fill(DriveAction::NONE);
  for (std::size_t s = 0; s < requests.size(); ++s)
  {
    if (requests[s] && names_joint(requests[s]->joints, joint))
    {
      actions[s] = requests[s]->action;
    }
  }
  return actions;
}

/// The highest-ranked of `actions`, NONE when there is none.
inline DriveAction strongest(const SourceActions& actions)
{
  return *std::max_element(actions.begin(), actions.end(),
                           [](DriveAction a, DriveAction b) { return precedence(a) < precedence(b); });
}

inline std::size_t count_active(const SourceActions& actions)
{
  return static_cast<std::size_t>(
      std::count_if(actions.begin(), actions.end(), [](DriveAction action) { return action != DriveAction::NONE; }));
}

/// Whether a request is done with a joint it names: its own action was written there, or a higher-ranked request
/// took the joint this cycle. While the joint still holds an earlier, different action, the request waits.
inline bool done_with_joint(DriveAction requested, DriveAction written, bool held, bool refused)
{
  return requested == written ? !refused : !held;
}

}  // namespace synapticon_controllers::drive_controller_detail
