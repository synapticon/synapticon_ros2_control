// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief DriveController's view of the drives: their reported state, condition, diagnostics and drive status.

#include <cstdio>
#include <limits>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "drive_controller_detail.hpp"
#include "synapticon_controllers/drive_controller.hpp"

namespace synapticon_controllers
{
using namespace drive_controller_detail;

namespace
{
std::string hex4(uint16_t value)
{
  char text[8];
  std::snprintf(text, sizeof(text), "0x%04X", value);
  return text;
}
}  // namespace

ReportedState DriveController::reported_state(std::size_t joint) const
{
  // A slave that left the bus keeps its last PDO data, so the master's view decides first.
  const JointSlots& slot = slots_[joint];
  const bool online = state_interfaces_[slot.online].get_optional<bool>().value_or(false);
  const uint8_t al_state = state_interfaces_[slot.al_state].get_optional<uint8_t>().value_or(0);
  if (!online || al_state != AL_STATE_OP)
  {
    return ReportedState::DISCONNECTED;
  }
  return reported_from_device_state(state_interfaces_[slot.device_state].get_optional<uint8_t>().value_or(
      static_cast<uint8_t>(DeviceState::UNKNOWN)));
}

void DriveController::read_reported_states()
{
  for (std::size_t j = 0; j < joint_names_.size(); ++j)
  {
    current_states_[j] = reported_state(j);
  }
}

JointCondition DriveController::condition_of(std::size_t joint, ReportedState state) const
{
  switch (state)
  {
    case ReportedState::DISCONNECTED:
      return JointCondition::UNREACHABLE;
    case ReportedState::FAULT:
    case ReportedState::FAULT_REACTION_ACTIVE:
      return JointCondition::FAULTED;
    default:
      break;
  }
  // Statusword bit 13 is the following error in the position modes pp (1) and csp (8).
  const int8_t mode = state_interfaces_[slots_[joint].mode_of_operation].get_optional<int8_t>().value_or(0);
  const uint16_t status_word = state_interfaces_[slots_[joint].status_word].get_optional<uint16_t>().value_or(0);
  const bool following_error = (mode == 1 || mode == 8) && (status_word & (1u << 13)) != 0;
  return following_error ? JointCondition::FOLLOWING_ERROR : JointCondition::HEALTHY;
}

std::string DriveController::error_report_of(std::size_t joint) const
{
  return decode_error_report(state_interfaces_[slots_[joint].error_report].get_optional().value_or(0.0));
}

void DriveController::fill_status_entry(const DriveReport& report,
                                        synapticon_control_msgs::msg::DriveStatus& entry) const
{
  entry.online = report.state != ReportedState::DISCONNECTED;
  entry.al_state = report.al_state;
  entry.device_state = report.device_state;
  entry.device_state_name = state_label(report.state);
  entry.mode_of_operation = report.mode_of_operation;
  entry.error_code = report.error_code;
  entry.error_report = decode_error_report(report.error_report);
  entry.latched = report.latched;
}

void DriveController::fill_report(std::size_t joint, DriveReport& report) const
{
  const JointSlots& slot = slots_[joint];
  const auto read = [this](std::size_t index, auto fallback) {
    return state_interfaces_[index].get_optional<decltype(fallback)>().value_or(fallback);
  };
  report.state = current_states_[joint];
  report.online = read(slot.online, false);
  report.al_state = read(slot.al_state, uint8_t{ 0 });
  report.device_state = read(slot.device_state, static_cast<uint8_t>(DeviceState::UNKNOWN));
  report.mode_of_operation = read(slot.mode_of_operation, int8_t{ 0 });
  report.error_code = read(slot.error_code, uint16_t{ 0 });
  report.error_report = read(slot.error_report, 0.0);
  report.latched = reaction_enabled_ && reaction_ && reaction_->latched(joint);
  for (std::size_t field = 0; field < DIAGNOSTICS_FIELDS.size(); ++field)
  {
    report.diagnostics[field] = read(slot.diagnostics[field], std::numeric_limits<double>::quiet_NaN());
  }
}

void DriveController::describe_drive(const std::optional<DriveReport>& report, std::size_t joint,
                                     diagnostic_updater::DiagnosticStatusWrapper& status) const
{
  status.hardware_id = joint_names_[joint];
  if (!report)
  {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No reading of the drive yet");
    return;
  }
  status.summary(state_level(report->state), state_label(report->state));
  status.add("online", report->online ? "yes" : "no");
  status.add("al_state", name_or_code(AL_STATE_NAMES, report->al_state));
  status.add("mode_of_operation", name_or_code(MODE_NAMES, report->mode_of_operation));
  // Outside Fault, the error code and report are left over from an earlier fault, so they are not shown.
  if (report->state == ReportedState::FAULT)
  {
    status.add("error_code", hex4(report->error_code));
    status.add("error_report", decode_error_report(report->error_report));
  }
  if (reaction_enabled_)
  {
    status.add("fault_reaction", report->latched ? "holds this joint" : "idle");
  }
  for (std::size_t field = 0; field < DIAGNOSTICS_FIELDS.size(); ++field)
  {
    status.add(DIAGNOSTICS_FIELDS[field], std::to_string(report->diagnostics[field]));
  }
}

void DriveController::publish_drive_status_change(std::size_t joint, const rclcpp::Time& time, const DriveReport& report)
{
  std::lock_guard<std::mutex> guard(drive_status_mutex_);
  if (joint >= drive_status_msg_.drives.size())
  {
    return;
  }
  fill_status_entry(report, drive_status_msg_.drives[joint]);
  drive_status_msg_.header.stamp = time;
  drive_status_pub_->publish(drive_status_msg_);
}

void DriveController::publish_drive_status_heartbeat()
{
  if (!active_.load())
  {
    return;
  }
  // The queued changes first, so the heartbeat is never older than a change published after it.
  diagnostics_.publish_changes();
  const auto latest = diagnostics_.latest();
  std::lock_guard<std::mutex> guard(drive_status_mutex_);
  for (std::size_t j = 0; j < latest.size() && j < drive_status_msg_.drives.size(); ++j)
  {
    if (latest[j])
    {
      fill_status_entry(*latest[j], drive_status_msg_.drives[j]);
    }
  }
  drive_status_msg_.header.stamp = get_node()->now();
  drive_status_pub_->publish(drive_status_msg_);
}

}  // namespace synapticon_controllers
