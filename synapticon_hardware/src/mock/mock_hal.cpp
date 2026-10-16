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
#include <optional>
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

/// Position of the first command that has an infinite setpoint, if any. NaN does not count.
std::optional<std::size_t> first_infinite(const std::vector<DriveCommand>& commands)
{
  for (std::size_t index = 0; index < commands.size(); ++index)
  {
    const DriveCommand& c = commands[index];
    for (const double value : { c.position, c.velocity, c.effort, c.stiffness, c.damping, c.max_joint_torque })
    {
      if (std::isinf(value))
      {
        return index;
      }
    }
  }
  return std::nullopt;
}

/// Store `source` in `target`, unless `source` is NaN.
void echo(double source, double& target)
{
  target = std::isnan(source) ? target : source;
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
    return step("connect()", Level::DISCONNECTED, Level::CONNECTED);
  }
  RCLCPP_ERROR(mock_logger(), "connect() is not allowed before setup() gave the mock its joints.");
  return false;
}

bool MockHal::start()
{
  return step("start()", Level::CONNECTED, Level::STREAMING);
}

bool MockHal::enable()
{
  return step("enable()", Level::STREAMING, Level::ENABLED);
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
  if (const auto index = first_infinite(commands))
  {
    RCLCPP_ERROR(mock_logger(), "write() rejected: infinite setpoint for joint '%s', number %zu of the command.",
                 names_[*index].c_str(), *index);
    return false;
  }
  for (std::size_t index = 0; index < commands.size(); ++index)
  {
    DriveState& state = reported_[index];
    echo(commands[index].position, state.position);
    echo(commands[index].velocity, state.velocity);
    echo(commands[index].effort, state.effort);
    state.status_word = STREAMING_STATUS_WORD;
  }
  return true;
}

bool MockHal::disable()
{
  // Streaming without power is where disable() leads, so there is nothing to do.
  return level_ == Level::STREAMING || step("disable()", Level::ENABLED, Level::STREAMING);
}

bool MockHal::stop()
{
  if (level_ == Level::ENABLED)
  {
    RCLCPP_ERROR(mock_logger(), "stop() needs disabled drives. Use disable() to switch them off.");
    return false;
  }
  return step("stop()", Level::STREAMING, Level::CONNECTED);
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

bool MockHal::step(const char* call, Level expected, Level next)
{
  if (level_ == expected)
  {
    level_ = next;
    return true;
  }
  RCLCPP_ERROR(mock_logger(), "%s is not allowed: the mock is %s, but the call needs it %s.", call,
               level_name(static_cast<int>(level_)), level_name(static_cast<int>(expected)));
  return false;
}

}  // namespace synapticon_hardware

PLUGINLIB_EXPORT_CLASS(synapticon_hardware::MockHal, synapticon_hardware::SynapticonHalI)
