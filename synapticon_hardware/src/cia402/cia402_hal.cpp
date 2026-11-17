// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Cia402Hal: powering drives up and down, per joint drive actions, and when setpoints reach a drive.

#include "synapticon_hardware/cia402/cia402_hal.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "synapticon_hardware/cia402/drive_action_walk.hpp"

namespace synapticon_hardware
{

namespace
{
using Clock = std::chrono::steady_clock;

// Time limit for enable(), disable() and each drive action to get a drive to its target state.
constexpr std::chrono::seconds WALK_TIMEOUT{ 5 };

// Standstill: the speed (rad/s, position shaft) stays below REST_SPEED for REST_TIME. A single slow sample is not
// enough, because the speed passes zero when a drive overshoots. On a ramped stop the power goes off at standstill, or
// after RAMP_WAIT_LIMIT in Quick Stop Active at the latest.
constexpr double REST_SPEED = 0.1;
constexpr std::chrono::milliseconds REST_TIME{ 50 };
constexpr std::chrono::seconds RAMP_WAIT_LIMIT{ 3 };
}  // namespace

Cia402Hal::Cia402Hal(std::string logger_name) : logger_name_(std::move(logger_name))
{
}

/// Order of the work:
/// 1. Refuse while any drive with energize is faulted, before anything changes, so that a running reset_fault goes
///    on. enabled_ is no help here, since a drive can fault after an earlier enable() without clearing it.
/// 2. Cancel every action other than enable. A stop left running would take the drives down again right after.
/// 3. Let setpoints through again for the joints that get power.
/// 4. Step every drive with energize up to Operation Enabled, one state per cycle. Nobody else writes setpoints
///    while this blocks, so each drive below Operation Enabled is told to stay where it is. A drive in Operation
///    Enabled keeps the target it has, and its controlword asks for Enable Operation again, which replaces a stop
///    that a drive action wrote. With every such drive in Operation Enabled this takes one cycle. Monitored drives
///    are kept without power.
/// 5. On a failure or timeout, take every drive down again, also those that already got power.
bool Cia402Hal::enable()
{
  if (!streaming_)
  {
    RCLCPP_ERROR(log(), "Cannot power the drives while no process data is exchanged. Call start() before enable().");
    return false;
  }
  if (!none_faulted())
  {
    return false;
  }
  cancel_walks([](DriveAction walking) { return walking != DriveAction::ENABLE; });

  const std::size_t energisable = energisable_count();
  if (energisable == 0)
  {
    RCLCPP_WARN(log(), "enable(): no drive gets power, since none of the %zu joint(s) sets energize.", drives_.size());
  }
  for (std::size_t j = 0; j < drives_.size(); ++j)
  {
    if (drives_[j].settings.energize)
    {
      refresh_feed(j, DriveAction::ENABLE, current_state(j));
    }
  }

  const Outcome result = walk_all("enable()", OnReadFailure::STOP, [this](std::size_t j) {
    const cia402::DeviceState state = current_state(j);
    if (state != cia402::DeviceState::OPERATION_ENABLED)
    {
      write_hold(j, transport_position(j));
    }
    if (!drives_[j].settings.energize)
    {
      write_step(j, state, cia402::DeviceState::SWITCH_ON_DISABLED, cia402::WalkPath::RAMPED);
      return Progress::DONE;
    }
    if (is_fault_state(state))
    {
      RCLCPP_ERROR(log(),
                   "Joint '%s' fell into %s while being powered, error code 0x%04X. Clear it with reset_fault, then "
                   "call enable() again.",
                   drives_[j].settings.name.c_str(), cia402::device_state_name(state), error_code(j));
      return Progress::FAILED;
    }
    write_step(j, state, cia402::DeviceState::OPERATION_ENABLED, cia402::WalkPath::RAMPED);
    return state == cia402::DeviceState::OPERATION_ENABLED ? Progress::DONE : Progress::PENDING;
  });

  switch (result)
  {
    case Outcome::DONE:
      enabled_ = true;
      RCLCPP_INFO(
          log(),
          "Power is on for %zu drive(s). Those that just got it hold their position, %zu drive(s) stay unpowered.",
          energisable, drives_.size() - energisable);
      return true;
    case Outcome::TIMED_OUT:
      report_enable_timeout();
      // A drive could still reach Operation Enabled later on the last controlword, so all of them go down.
      RCLCPP_ERROR(log(), "Powering the drives took longer than %ld s. All of them go back to Switch On Disabled.",
                   static_cast<long>(WALK_TIMEOUT.count()));
      disable();
      enabled_ = false;
      return false;
    case Outcome::FAILED:
      // Drives that got to Operation Enabled before the failure must not keep their power. If the bus failed, this
      // is a best effort.
      RCLCPP_ERROR(log(), "enable() failed. All drives go back to Switch On Disabled.");
      disable();
      enabled_ = false;
      break;
  }
  return false;
}

/// Per joint and cycle: take a changed drive_action, pass the setpoints on if the joint gets them, write the
/// controlword and keep a quick stop hold in place. An invalid setpoint is logged once per run of such cycles.
bool Cia402Hal::write(const std::vector<DriveCommand>& commands)
{
  if (!streaming_)
  {
    RCLCPP_ERROR(log(), "write() is not possible before start().");
    return false;
  }
  if (commands.size() != drives_.size())
  {
    RCLCPP_ERROR(log(), "write() needs one command per joint: %zu joint(s), %zu command(s).", drives_.size(),
                 commands.size());
    return false;
  }

  bool valid = true;
  for (std::size_t j = 0; j < drives_.size(); ++j)
  {
    if (has_invalid_setpoint(commands[j]))
    {
      if (valid && !invalid_reported_)
      {
        RCLCPP_ERROR(log(),
                     "write(): joint '%s' got an infinite setpoint, or a negative stiffness, damping or "
                     "max_joint_torque. Each such field keeps its last value.",
                     drives_[j].settings.name.c_str());
      }
      valid = false;
    }
    DriveAction accepted = DriveAction::NONE;
    const DriveAction requested = commands[j].drive_action;
    if (requested != drives_[j].seen_request)
    {
      drives_[j].seen_request = requested;
      accepted = accept_request(j, requested);
    }

    // Stopped or faulted drives get no new setpoints. They keep the last target they got.
    const cia402::DeviceState state = current_state(j);
    refresh_feed(j, accepted, state);
    if (drives_[j].feed.forwarding)
    {
      write_setpoints(j, commands[j]);
    }

    steer(j);
    freeze_in_quick_stop(j, state);
  }
  invalid_reported_ = !valid;
  return exchange_write() && valid;
}

/// Takes every joint down, also one that a drive action powered, because enabled_ only follows enable() and
/// disable(). Actions that need power are cancelled first, so they do not work against the way down. The way down
/// is ramped, except for a joint that is already on the direct way of a disable_voltage. Only joints with energize
/// are waited for: a monitored joint left in Fault must not block, and Fault has no power anyway. A failed read does
/// not stop the way down: the controlwords go out every cycle on the states received last, and disable() returns
/// false when the last read failed.
bool Cia402Hal::disable()
{
  if (!streaming_)
  {
    enabled_ = false;
    return true;  // no exchange, so no drive can have power
  }
  cancel_walks([](DriveAction walking) { return needs_energised_drive(walking); });
  for (std::size_t j = 0; j < drives_.size(); ++j)
  {
    refresh_feed(j, DriveAction::DISABLE, current_state(j));
  }

  const Outcome result = walk_all("disable()", OnReadFailure::CARRY_ON, [this](std::size_t j) {
    const cia402::DeviceState state = current_state(j);
    freeze_in_quick_stop(j, state);
    const bool direct = drives_[j].walking == DriveAction::DISABLE_VOLTAGE;
    write_step(j, state, cia402::DeviceState::SWITCH_ON_DISABLED,
               direct ? cia402::WalkPath::DIRECT : cia402::WalkPath::RAMPED);
    // Not Ready to Switch On has no power and moves on to Switch On Disabled by itself, so it counts as down.
    const bool down = cia402::reached(state, cia402::DeviceState::SWITCH_ON_DISABLED) ||
                      state == cia402::DeviceState::NOT_READY_TO_SWITCH_ON;
    return down || !drives_[j].settings.energize ? Progress::DONE : Progress::PENDING;
  });
  if (result == Outcome::FAILED)
  {
    return false;
  }
  enabled_ = false;
  if (result == Outcome::TIMED_OUT)
  {
    RCLCPP_WARN(log(), "Some drives are still above Switch On Disabled when disable() gives up. Their controlword "
                       "already asks for the power to go off.");
    return false;
  }
  for (std::size_t j = 0; j < drives_.size(); ++j)
  {
    if (current_state(j) == cia402::DeviceState::FAULT)
    {
      RCLCPP_WARN(log(), "Joint '%s' remains in Fault after disable(), error code 0x%04X. A reset_fault clears it.",
                  drives_[j].settings.name.c_str(), error_code(j));
    }
  }
  RCLCPP_INFO(log(), "disable() finished, the %zu drive(s) have no power now.", drives_.size());
  return true;
}

std::size_t Cia402Hal::joint_count() const
{
  return drives_.size();
}

bool Cia402Hal::is_streaming() const
{
  return streaming_;
}

bool Cia402Hal::is_enabled() const
{
  if (!enabled_)
  {
    return false;
  }
  for (std::size_t j = 0; j < drives_.size(); ++j)
  {
    if (drives_[j].settings.energize && current_state(j) != cia402::DeviceState::OPERATION_ENABLED)
    {
      return false;
    }
  }
  return true;
}

DriveAction Cia402Hal::active_drive_action(std::size_t joint) const
{
  return joint < drives_.size() ? drives_[joint].walking : DriveAction::NONE;
}

bool Cia402Hal::forwarding_setpoints(std::size_t joint) const
{
  return joint >= drives_.size() || drives_[joint].feed.forwarding;
}

void Cia402Hal::init_joints(std::vector<JointSettings> joints, double control_frequency)
{
  drives_.clear();
  drives_.reserve(joints.size());
  for (auto& settings : joints)
  {
    drives_.push_back(DriveSlot{ std::move(settings) });
  }
  period_ = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / control_frequency));
}

cia402::DeviceState Cia402Hal::current_state(std::size_t joint) const
{
  return cia402::device_state(statusword(joint));
}

/// Checks in order: reset_fault needs a drive in Fault or Fault Reaction Active, NONE starts nothing (a running action
/// goes on to its end), the HAL must support the action, enable needs a drive without fault, actions that power the
/// drive need energize, and quick_stop falls back to disable when the drive has no quick stop. Asking again for the
/// running action keeps its deadline. A reset_fault taken in Fault Reaction Active sends the reset once the drive is in
/// Fault.
DriveAction Cia402Hal::accept_request(std::size_t j, DriveAction action)
{
  const char* name = drives_[j].settings.name.c_str();

  if (action == DriveAction::RESET_FAULT && !is_fault_state(current_state(j)))
  {
    RCLCPP_WARN(log(), "Joint '%s': no fault to reset (statusword 0x%04X), reset_fault ignored.", name, statusword(j));
    return DriveAction::NONE;
  }

  if (action == DriveAction::NONE)
  {
    return DriveAction::NONE;
  }
  if (!is_walk_action(action))
  {
    RCLCPP_WARN(log(), "Joint '%s': this HAL cannot carry out drive_action '%s', ignored.", name,
                action_to_string(action));
    return DriveAction::NONE;
  }
  if (action == DriveAction::ENABLE && is_fault_state(current_state(j)))
  {
    RCLCPP_WARN(log(), "Joint '%s': no drive_action 'enable' while the drive is in %s. Send reset_fault first.", name,
                cia402::device_state_name(current_state(j)));
    return DriveAction::NONE;
  }
  if (needs_energised_drive(action) && !drives_[j].settings.energize)
  {
    RCLCPP_WARN(log(), "Joint '%s' has energize set to false, drive_action '%s' ignored.", name,
                action_to_string(action));
    return DriveAction::NONE;
  }
  if (action == DriveAction::QUICK_STOP && !quick_stop_available(j))
  {
    RCLCPP_ERROR(log(),
                 "Joint '%s' has no quick stop set up (0x605A is neither 6 nor 2). Its drive_action 'quick_stop' runs "
                 "as 'disable'.",
                 name);
    action = DriveAction::DISABLE;
  }
  auto& joint = drives_[j];
  if (joint.walking == action)
  {
    return action;
  }
  joint.walking = action;
  joint.walk_ends = Clock::now() + WALK_TIMEOUT;
  RCLCPP_INFO(log(), "Joint '%s': drive_action '%s' started, heading for %s.", name, action_to_string(action),
              cia402::device_state_name(drive_walk(action).target));
  return action;
}

/// The state goes first, as it reports what the drive did since the last cycle. The action comes second, as it is
/// the request made in this cycle.
void Cia402Hal::refresh_feed(std::size_t j, DriveAction action, cia402::DeviceState state)
{
  auto& feed = drives_[j].feed;
  const char* name = drives_[j].settings.name.c_str();
  if (feed.on_state(state))
  {
    RCLCPP_INFO(log(), "Joint '%s': setpoints held back, the drive is in %s.", name, cia402::device_state_name(state));
  }
  if (feed.on_action(action))
  {
    RCLCPP_INFO(log(), "Joint '%s': setpoints %s after drive_action '%s'.", name,
                feed.forwarding ? "passed on again" : "held back", action_to_string(action));
  }
}

bool Cia402Hal::none_faulted() const
{
  bool none = true;
  for (std::size_t j = 0; j < drives_.size(); ++j)
  {
    const cia402::DeviceState state = current_state(j);
    if (drives_[j].settings.energize && is_fault_state(state))
    {
      RCLCPP_ERROR(log(), "No enable() while joint '%s' is in %s, error code 0x%04X. Send reset_fault first.",
                   drives_[j].settings.name.c_str(), cia402::device_state_name(state), error_code(j));
      none = false;
    }
  }
  return none;
}

/// While an action is still going on, the controller's setpoints already pass (from the enable on). A drive
/// below Operation Enabled ignores them, and follows the setpoint of the cycle in which it gets there. So the action
/// needs no hold of its own, and whoever asked for the enable decides where the joint goes.
bool Cia402Hal::advance_walk(std::size_t j, uint16_t sw)
{
  auto& joint = drives_[j];
  const DriveAction action = joint.walking;
  const DriveWalk walk = drive_walk(action);
  const cia402::DeviceState target = walk.target;
  const cia402::DeviceState state = cia402::device_state(sw);
  const char* name = joint.settings.name.c_str();

  if (action == DriveAction::ENABLE && is_fault_state(state))
  {
    joint.walking = DriveAction::NONE;  // only reset_fault leads out of a fault
    RCLCPP_ERROR(log(),
                 "Joint '%s': drive_action 'enable' stopped, the drive went to %s (error code 0x%04X). Send "
                 "reset_fault first.",
                 name, cia402::device_state_name(state), error_code(j));
    return false;
  }
  if (cia402::reached(state, target, walk.fault_exit))
  {
    joint.walking = DriveAction::NONE;
    if (state == target)
    {
      RCLCPP_INFO(log(), "Joint '%s' reached %s.", name, cia402::device_state_name(target));
    }
    else
    {
      RCLCPP_INFO(log(), "Joint '%s': drive_action '%s' done, the drive has no power (%s).", name,
                  action_to_string(action), cia402::device_state_name(state));
    }
    return false;
  }
  if (Clock::now() >= joint.walk_ends)
  {
    joint.walking = DriveAction::NONE;
    RCLCPP_ERROR(log(),
                 "Joint '%s': drive_action '%s' did not get to %s within %ld s, the drive is in %s (statusword "
                 "0x%04X).",
                 name, action_to_string(action), cia402::device_state_name(target),
                 static_cast<long>(WALK_TIMEOUT.count()), cia402::device_state_name(state), sw);
    if (action == DriveAction::ENABLE)
    {
      // The drive might still follow the last controlword up to Operation Enabled later, so take it down.
      RCLCPP_ERROR(log(), "Joint '%s': taking it back down to %s.", name,
                   cia402::device_state_name(drive_walk(DriveAction::DISABLE).target));
      refresh_feed(j, DriveAction::DISABLE, state);
      joint.walking = DriveAction::DISABLE;
      joint.walk_ends = Clock::now() + WALK_TIMEOUT;
      return true;
    }
    return false;
  }
  return true;
}

void Cia402Hal::steer(std::size_t j)
{
  const uint16_t sw = statusword(j);
  // Without a running action the target is UNKNOWN, which keeps the controlword unchanged.
  DriveWalk walk{};
  if (drives_[j].walking != DriveAction::NONE && advance_walk(j, sw))
  {
    walk = drive_walk(drives_[j].walking);
  }
  write_step(j, cia402::device_state(sw), walk.target, walk.path, walk.fault_exit);
}

bool Cia402Hal::resting(std::size_t j)
{
  auto& slow_start = drives_[j].slow_start;
  if (!(speed(j) < REST_SPEED))  // a NaN speed counts as moving
  {
    slow_start = Clock::time_point{};
    return false;
  }
  const auto now = Clock::now();
  if (slow_start == Clock::time_point{})
  {
    slow_start = now;
  }
  return now - slow_start >= REST_TIME;
}

/// In position and velocity modes the drive brakes along its own ramp and then ignores the target, so nothing is
/// needed. Other modes keep following the target. For them, with option code 6, the target fixed at the start of
/// the quick stop is what brings the joint to a stop.
void Cia402Hal::freeze_in_quick_stop(std::size_t j, cia402::DeviceState state)
{
  auto& joint = drives_[j];
  if (!joint.settings.hold_in_quick_stop)
  {
    return;
  }
  if (state != cia402::DeviceState::QUICK_STOP_ACTIVE)
  {
    joint.frozen = false;
    return;
  }
  if (!joint.frozen)
  {
    joint.frozen_target = transport_position(j);
    joint.frozen = true;
  }
  write_hold(j, joint.frozen_target);
}

void Cia402Hal::write_step(std::size_t j, cia402::DeviceState current, cia402::DeviceState target,
                           cia402::WalkPath path, cia402::FaultExit fault_exit)
{
  auto& wait_start = drives_[j].ramp_wait_start;
  const bool at_rest = resting(j);  // called every cycle, see resting()
  cia402::DeviceState step = target;
  if (!cia402::awaits_ramp(current, target, path))
  {
    wait_start = Clock::time_point{};
  }
  else if (wait_start != Clock::time_point::max())
  {  // max marks a wait that already ran out
    const auto now = Clock::now();
    if (wait_start == Clock::time_point{})
    {
      wait_start = now;
    }
    if (!at_rest && now - wait_start < RAMP_WAIT_LIMIT)
    {
      step = cia402::DeviceState::QUICK_STOP_ACTIVE;  // the joint still brakes, keep the power on
    }
    else if (!at_rest)
    {
      wait_start = Clock::time_point::max();
      RCLCPP_WARN(log(),
                  "Joint '%s' still turns at %.3g rad/s after %ld s of braking in Quick Stop Active. Switching "
                  "the power off now.",
                  drives_[j].settings.name.c_str(), speed(j), static_cast<long>(RAMP_WAIT_LIMIT.count()));
    }
  }
  write_controlword(j, cia402::next_controlword(last_controlword(j), current, step, path, fault_exit));
}

template <typename StepFn>
Cia402Hal::Outcome Cia402Hal::walk_all(const char* what, OnReadFailure on_read_failure, StepFn step)
{
  const auto deadline = Clock::now() + WALK_TIMEOUT;
  bool received = true;
  bool reported = false;
  while (Clock::now() < deadline)
  {
    received = exchange_read();
    if (!received && on_read_failure == OnReadFailure::STOP)
    {
      RCLCPP_ERROR(log(), "%s stopped: no process data received.", what);
      return Outcome::FAILED;
    }
    if (!received && !reported)
    {
      RCLCPP_ERROR(log(), "%s: no process data received. The controlwords go on, based on the states received last.",
                   what);
      reported = true;
    }
    bool all_done = true;
    for (std::size_t j = 0; j < drives_.size(); ++j)
    {
      const Progress result = step(j);
      if (result == Progress::FAILED)
      {
        return Outcome::FAILED;
      }
      all_done = all_done && result == Progress::DONE;
    }
    if (!exchange_write())
    {
      RCLCPP_ERROR(log(), "%s stopped: the process data could not be sent.", what);
      return Outcome::FAILED;
    }
    if (all_done && received)
    {
      return Outcome::DONE;
    }
    if (all_done)
    {
      break;
    }
    std::this_thread::sleep_for(period_);
  }
  if (!received)
  {
    RCLCPP_ERROR(log(), "%s ends without process data, so the states of the drives are not confirmed.", what);
    return Outcome::FAILED;
  }
  return Outcome::TIMED_OUT;
}

std::size_t Cia402Hal::energisable_count() const
{
  return static_cast<std::size_t>(
      std::count_if(drives_.begin(), drives_.end(), [](const DriveSlot& slot) { return slot.settings.energize; }));
}

template <typename Predicate>
void Cia402Hal::cancel_walks(Predicate cancelled)
{
  for (DriveSlot& slot : drives_)
  {
    if (cancelled(slot.walking))
    {
      slot.walking = DriveAction::NONE;
    }
  }
}

void Cia402Hal::report_enable_timeout() const
{
  for (std::size_t j = 0; j < drives_.size(); ++j)
  {
    if (drives_[j].settings.energize)  // enable() does not wait for the others
    {
      const uint16_t sw = statusword(j);
      RCLCPP_ERROR(log(),
                   "Joint '%s' did not reach Operation Enabled, it stays in %s (statusword 0x%04X, error code "
                   "0x%04X).",
                   drives_[j].settings.name.c_str(), cia402::device_state_name(cia402::device_state(sw)), sw,
                   error_code(j));
    }
  }
}

}  // namespace synapticon_hardware
