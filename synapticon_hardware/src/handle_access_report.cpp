// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Throttled logging of refused ros2_control handle accesses.

#include "synapticon_hardware/handle_access_report.hpp"

#include <rclcpp/rclcpp.hpp>

namespace synapticon_hardware
{

namespace
{
constexpr const char* LOGGER_NAME = "SynapticonSystemInterface";

// Minimum gap between two reports of refused interface accesses.
constexpr int FAILURE_LOG_PERIOD_MS = 5000;
}  // namespace

void HandleAccessReport::state_writes(std::size_t refused)
{
  if (refused == 0)
  {
    return;
  }
  failures_.state_write += refused;
  RCLCPP_WARN_THROTTLE(rclcpp::get_logger(LOGGER_NAME), throttle_clock_, FAILURE_LOG_PERIOD_MS,
                       "%zu state interface write(s) refused this cycle (%lu since activation). Those interfaces "
                       "keep last cycle's value and catch up on the next read().",
                       refused, static_cast<unsigned long>(failures_.state_write));
}

void HandleAccessReport::command_reads(std::size_t setpoints_refused, std::size_t actions_refused)
{
  if (setpoints_refused != 0)
  {
    failures_.setpoint_read += setpoints_refused;
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger(LOGGER_NAME), throttle_clock_, FAILURE_LOG_PERIOD_MS,
                         "%zu setpoint interface read(s) refused this cycle (%lu since activation). Those joints "
                         "keep last cycle's setpoint and recover on the next write().",
                         setpoints_refused, static_cast<unsigned long>(failures_.setpoint_read));
  }
  if (actions_refused != 0)
  {
    // Reported as an error, unlike a refused setpoint: a drive_action is edge-triggered, so a
    // cycle that could not be read drops the request outright rather than delaying it.
    failures_.action_read += actions_refused;
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(LOGGER_NAME), throttle_clock_, FAILURE_LOG_PERIOD_MS,
                          "%zu drive_action interface read(s) refused this cycle (%lu since activation). A request "
                          "asserted on those interfaces this cycle is LOST, not delayed. Re-issue it.",
                          actions_refused, static_cast<unsigned long>(failures_.action_read));
  }
}

void HandleAccessReport::command_seeds(std::size_t refused)
{
  if (refused == 0)
  {
    return;
  }
  failures_.command_seed_write += refused;
  RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
               "%zu command interface(s) could not be seeded at activation. Those interfaces still hold "
               "their default, so a controller reading its command back sees a value this hardware never "
               "chose.",
               refused);
}

void HandleAccessReport::summarise() const
{
  if (!failures_.any())
  {
    return;
  }
  RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
              "Interface accesses refused during this activation: %lu state write(s), %lu setpoint "
              "read(s), %lu drive_action read(s), %lu command seed write(s). Expect these only when the "
              "component is run asynchronously.",
              static_cast<unsigned long>(failures_.state_write), static_cast<unsigned long>(failures_.setpoint_read),
              static_cast<unsigned long>(failures_.action_read),
              static_cast<unsigned long>(failures_.command_seed_write));
}

void HandleAccessReport::reset()
{
  failures_ = HandleAccessFailures{};
}

}  // namespace synapticon_hardware
