// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Cia402Hal, the transport-independent half of a HAL for CiA 402 drives.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>

#include "synapticon_hardware/cia402/state_machine.hpp"
#include "synapticon_hardware/drive_data.hpp"
#include "synapticon_hardware/cia402/setpoint_forwarding.hpp"
#include "synapticon_hardware/synapticon_hal.hpp"

namespace synapticon_hardware
{

/// @brief Drive logic shared by every HAL whose drives speak CiA 402.
///
/// The split of work:
/// - Here: powering the drives up and down (enable(), disable()), the drive actions that write() starts per joint,
///   when controller setpoints reach a drive, holding an impedance joint in a quick stop, and keeping the torque on
///   during a ramped stop until the joint is still.
/// - In the derived HAL: moving the bytes. It implements the protected hooks (read and write half of one cycle,
///   the drive's values, the controlword and setpoint writes), calls init_joints() from connect(), and sets
///   `streaming_` in start() and stop().
class Cia402Hal : public SynapticonHalI
{
public:
  /// Per joint settings of the drive logic.
  struct JointSettings
  {
    std::string name;
    /// False: the drive only reports, it is kept without power and enable() and disable() do not wait for it.
    bool energize{ true };
    /// True for modes in which the drive does not brake by itself, such as impedance mode: in Quick Stop Active the
    /// target stays at the position where the stop started.
    bool hold_in_quick_stop{ false };
  };

  bool enable() override;
  bool write(const std::vector<DriveCommand>& commands) override;
  bool disable() override;
  std::size_t joint_count() const override;
  bool is_streaming() const override;
  /// True after a successful enable() and before disable(), while every joint with energize is in Operation Enabled.
  bool is_enabled() const override;

  /// The drive action that `joint` is still carrying out, NONE when idle.
  DriveAction active_drive_action(std::size_t joint) const;

  /// True while write() passes `joint`'s setpoints on. A stop action (quick_stop, disable, disable_voltage) or a
  /// Fault turns this off until the joint is enabled again. The drive then keeps the last target it got.
  bool forwarding_setpoints(std::size_t joint) const;

  /// True when `joint`'s drive is set up for a quick stop. Otherwise a quick_stop request runs as a disable.
  virtual bool quick_stop_available(std::size_t joint) const = 0;

protected:
  /// All log output of the drive logic goes to the logger `logger_name`.
  explicit Cia402Hal(std::string logger_name);

  /// Start the drive logic for `joints`, given in joint order, with no action running. Called from connect(),
  /// before the first cycle.
  void init_joints(std::vector<JointSettings> joints, double control_frequency);

  /// @name Transport hooks
  /// The derived HAL moves the data. One cycle is exchange_read(), then the writes, then exchange_write().
  /// @{

  /// Receive this cycle's data, so that the getters below return fresh values.
  virtual bool exchange_read() = 0;
  /// Send what the write hooks set for this cycle.
  virtual bool exchange_write() = 0;

  virtual uint16_t statusword(std::size_t joint) const = 0;
  virtual uint16_t error_code(std::size_t joint) const = 0;
  /// Controlword sent one cycle ago.
  virtual uint16_t last_controlword(std::size_t joint) const = 0;
  /// Absolute motor side speed in rad/s.
  virtual double speed(std::size_t joint) const = 0;
  /// Measured position, in the same unit as the position targets that the transport sends.
  virtual double transport_position(std::size_t joint) const = 0;

  virtual void write_controlword(std::size_t joint, uint16_t controlword) = 0;
  /// Set this cycle's setpoints from the controller fields of `command`.
  virtual void write_setpoints(std::size_t joint, const DriveCommand& command) = 0;
  /// Set the target to `position` (in transport_position() units), with velocity and torque offset zero.
  virtual void write_hold(std::size_t joint, double position) = 0;
  /// @}

  /// CiA 402 state of `joint`, from the statusword received last.
  cia402::DeviceState current_state(std::size_t joint) const;

  const char* logger_name() const
  {
    return logger_name_.c_str();
  }

  /// Logger of this HAL.
  rclcpp::Logger log() const
  {
    return rclcpp::get_logger(logger_name_);
  }

  bool streaming_{ false };
  bool enabled_{ false };  ///< Set by a successful enable(), cleared by disable() and stop().

private:
  /// What the drive logic keeps for one joint.
  struct DriveSlot
  {
    JointSettings settings;
    DriveAction seen_request{ DriveAction::NONE };  ///< last drive_action value seen, to spot a change
    /// Action in progress, NONE when idle. It advances by one state per cycle and ends at its target or its
    /// deadline. write() checks the deadline every cycle instead of waiting.
    DriveAction walking{ DriveAction::NONE };
    std::chrono::steady_clock::time_point walk_ends{};
    SetpointForwarding feed{};
    /// Position target kept in Quick Stop Active, valid while `frozen` is set.
    double frozen_target{ 0.0 };
    bool frozen{ false };
    /// Start of the wait in Quick Stop Active during a ramped stop. Zero when not waiting, max after the wait ran
    /// out.
    std::chrono::steady_clock::time_point ramp_wait_start{};
    /// First cycle below REST_SPEED. Zero while the joint is faster.
    std::chrono::steady_clock::time_point slow_start{};
  };

  /// Handle a changed drive_action of joint `j`. Returns the action now running, or NONE when the request is
  /// refused (the reason is logged).
  DriveAction accept_request(std::size_t j, DriveAction action);
  /// Decide whether `j`'s setpoints go to the drive, given the action just accepted and the drive's state.
  void refresh_feed(std::size_t j, DriveAction action, cia402::DeviceState state);
  /// False when a joint with energize is in Fault or Fault Reaction Active. Each such joint is logged.
  bool none_faulted() const;
  /// Check `j`'s running action for this cycle: done, out of time, or going on (returns true). An enable that runs
  /// out of time turns into a disable.
  bool advance_walk(std::size_t j, uint16_t statusword);
  /// Choose and write `j`'s controlword for this cycle.
  void steer(std::size_t j);
  /// True when `j` has been below REST_SPEED for REST_TIME. Must run every cycle for every joint, or the time
  /// is measured from a later sample.
  bool resting(std::size_t j);
  /// With hold_in_quick_stop set, keep `j`'s target where Quick Stop Active began. Runs once per joint and cycle.
  void freeze_in_quick_stop(std::size_t j, cia402::DeviceState state);
  /// Send the controlword for one state change of `j` from `current` toward `target`. On a ramped way down the drive
  /// stays in Quick Stop Active until the joint is still or RAMP_WAIT_LIMIT is over, so the power stays on while it
  /// brakes. Runs exactly once per joint and cycle.
  void write_step(std::size_t j, cia402::DeviceState current, cia402::DeviceState target, cia402::WalkPath path,
                  cia402::FaultExit fault_exit = cia402::FaultExit::HOLD);
  /// How walk_all() ended.
  enum class Outcome
  {
    DONE,       ///< all joints at their target
    TIMED_OUT,  ///< out of time
    FAILED,     ///< a joint or a bus exchange failed, the reason is logged
  };
  /// What the per joint step of walk_all() reports.
  enum class Progress
  {
    DONE,
    PENDING,
    FAILED,
  };
  /// Exchange cycles and call `step` for each joint in each of them, until all are done, one fails or
  /// WALK_TIMEOUT is over. `what` is the name logged when an exchange fails.
  template <typename StepFn>
  Outcome walk_all(const char* what, StepFn step);
  /// Count of joints with energize set.
  std::size_t energisable_count() const;
  /// Set every running action for which `cancelled(walking)` returns true back to NONE.
  template <typename Predicate>
  void cancel_walks(Predicate cancelled);
  /// Log the state of every joint with energize after enable() ran out of time.
  void report_enable_timeout() const;

  std::string logger_name_;
  std::vector<DriveSlot> drives_;
  std::chrono::nanoseconds period_{ 1000000 };
};

}  // namespace synapticon_hardware
