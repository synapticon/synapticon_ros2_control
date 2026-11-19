// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Counting and reporting of interface accesses the ros2_control handle layer refused.

#include <cstddef>
#include <cstdint>

#include <rclcpp/clock.hpp>

namespace synapticon_hardware
{

/// Running count of interface accesses the handle layer refused.
struct HandleAccessFailures
{
  uint64_t state_write{ 0 };         ///< State interface could not be written this cycle.
  uint64_t setpoint_read{ 0 };       ///< Setpoint interface could not be read this cycle.
  uint64_t action_read{ 0 };         ///< drive_action interface could not be read: request lost.
  uint64_t command_seed_write{ 0 };  ///< Command interface could not be seeded at activation.

  uint64_t total() const
  {
    return state_write + setpoint_read + action_read + command_seed_write;
  }
  bool any() const
  {
    return total() != 0;
  }
};

/// @brief Counts refused handle accesses and reports them to the log, throttled.
///
/// In the sequential read, update, write model nothing is ever refused. A refusal means the
/// component runs asynchronously and an access was dropped, so it is logged, but at most once per
/// log period from the real-time loop.
class HandleAccessReport
{
public:
  /// Count `refused` state writes of this cycle and report them.
  void state_writes(std::size_t refused);

  /// Count the setpoint and drive action reads refused this cycle and report them. A refused
  /// action read loses a request, so it is reported as an error.
  void command_reads(std::size_t setpoints_refused, std::size_t actions_refused);

  /// Count the command seeds that could not be written at activation and report them.
  void command_seeds(std::size_t refused);

  /// Log the totals of this activation, if there were any refusals.
  void summarise() const;

  /// Forget the counts of the previous activation.
  void reset();

  const HandleAccessFailures& failures() const
  {
    return failures_;
  }

private:
  HandleAccessFailures failures_{};
  rclcpp::Clock throttle_clock_{ RCL_STEADY_TIME };
};

}  // namespace synapticon_hardware
