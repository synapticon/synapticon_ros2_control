// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief MockHal: stores each command and reports it back as the next state.

#include "synapticon_hardware/mock_hal.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace synapticon_hardware
{

namespace
{
/// Statusword of a joint after its first write. Any value other than 0 tells the reader that data arrives.
constexpr uint16_t STREAMING_STATUS_WORD = 0x0001;

/// Logger of every MockHal instance.
const rclcpp::Logger& mock_logger()
{
  static const rclcpp::Logger instance = rclcpp::get_logger("MockHal");
  return instance;
}

/// Readable name of a stage, for log lines.
const char* level_name(int level)
{
  constexpr std::array<const char*, 4> names{ "disconnected", "connected", "streaming", "enabled" };
  return names.at(static_cast<std::size_t>(level));
}

/// Store `source` in `target`, unless `source` is NaN or infinite.
void echo(double source, double& target)
{
  target = std::isfinite(source) ? source : target;
}
}  // namespace

MockHal::MockHal() noexcept = default;

bool MockHal::setup(std::vector<std::string> joint_names)
{
  if (level_ != Level::DISCONNECTED)
  {
    RCLCPP_ERROR(mock_logger(), "setup() is not allowed while the mock is connected. Call disconnect() before.");
    return false;
  }
  names_ = std::move(joint_names);
  reported_ = std::vector<DriveState>(names_.size());
  configured_ = true;
  return true;
}

bool MockHal::setup(const HalConfig& config)
{
  std::vector<std::string> joint_names(config.joints.size());
  for (std::size_t index = 0; index < joint_names.size(); ++index)
  {
    joint_names[index] = config.joints[index].name;
  }
  return setup(std::move(joint_names));
}

bool MockHal::connect()
{
  if (configured_)
  {
    return climb("connect()", Level::CONNECTED);
  }
  RCLCPP_ERROR(mock_logger(), "connect() is not allowed before setup() gave the mock its joints.");
  return false;
}

bool MockHal::start()
{
  return climb("start()", Level::STREAMING);
}

bool MockHal::enable()
{
  return climb("enable()", Level::ENABLED);
}

bool MockHal::read(std::vector<DriveState>& states)
{
  if (!is_streaming())
  {
    RCLCPP_ERROR(mock_logger(), "read() is not allowed: the mock is %s, so it has no data.",
                 level_name(static_cast<int>(level_)));
    return false;
  }
  states = reported_;
  return true;
}

bool MockHal::write(const std::vector<DriveCommand>& commands)
{
  if (!is_streaming())
  {
    RCLCPP_ERROR(mock_logger(), "write() is not allowed: the mock is %s, so it takes no commands.",
                 level_name(static_cast<int>(level_)));
    return false;
  }
  if (commands.size() != reported_.size())
  {
    RCLCPP_ERROR(mock_logger(), "write() is not allowed with %zu commands: the mock has %zu joints.", commands.size(),
                 reported_.size());
    return false;
  }
  bool valid = true;
  for (std::size_t index = 0; index < commands.size(); ++index)
  {
    if (has_invalid_setpoint(commands[index]))
    {
      RCLCPP_ERROR(mock_logger(), "write(): invalid setpoint for joint '%s', number %zu. That field keeps its value.",
                   names_[index].c_str(), index);
      valid = false;
    }
    DriveState& state = reported_[index];
    echo(commands[index].position, state.position);
    echo(commands[index].velocity, state.velocity);
    echo(commands[index].effort, state.effort);
    state.status_word = STREAMING_STATUS_WORD;
  }
  return valid;
}

bool MockHal::disable()
{
  if (level_ == Level::ENABLED)
  {
    level_ = Level::STREAMING;
  }
  return true;
}

bool MockHal::stop()
{
  if (level_ >= Level::STREAMING)
  {
    level_ = Level::CONNECTED;  // switches the power off too
  }
  return true;
}

bool MockHal::disconnect()
{
  level_ = Level::DISCONNECTED;
  return true;
}

std::size_t MockHal::joint_count() const
{
  return names_.size();
}

bool MockHal::is_connected() const
{
  return level_ != Level::DISCONNECTED;
}

bool MockHal::is_streaming() const
{
  return level_ >= Level::STREAMING;
}

bool MockHal::is_enabled() const
{
  return level_ == Level::ENABLED;
}

bool MockHal::set_states(const std::vector<DriveState>& states)
{
  if (states.size() == names_.size())
  {
    reported_ = states;
    return true;
  }
  RCLCPP_ERROR(mock_logger(), "set_states() is not allowed with %zu states: the mock has %zu joints.", states.size(),
               names_.size());
  return false;
}

bool MockHal::climb(const char* call, Level next)
{
  if (level_ >= next)
  {
    return true;
  }
  const auto below = static_cast<Level>(static_cast<int>(next) - 1);
  if (level_ == below)
  {
    level_ = next;
    return true;
  }
  RCLCPP_ERROR(mock_logger(), "%s is not allowed: the mock is %s, but the call needs it %s.", call,
               level_name(static_cast<int>(level_)), level_name(static_cast<int>(below)));
  return false;
}

}  // namespace synapticon_hardware

PLUGINLIB_EXPORT_CLASS(synapticon_hardware::MockHal, synapticon_hardware::SynapticonHalI)
