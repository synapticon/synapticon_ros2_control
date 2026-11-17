// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief MockCia402Hal: Cia402Hal over simulated drives.

#include "synapticon_hardware/mock_cia402_hal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "synapticon_hardware/hal_config.hpp"
#include "simulated_drive.hpp"

namespace synapticon_hardware
{

namespace
{
constexpr const char* LOGGER_NAME = "MockCia402Hal";
constexpr int IMPEDANCE_MODE = -6;
// The longest time one read advances the simulation, so a paused caller does not see a jump.
constexpr double MAX_STEP = 0.05;

FollowingErrorMode parse_following_error(const std::string& value)
{
  if (value == "off")
  {
    return FollowingErrorMode::OFF;
  }
  if (value == "report")
  {
    return FollowingErrorMode::REPORT;
  }
  if (value == "fault")
  {
    return FollowingErrorMode::FAULT;
  }
  throw std::invalid_argument("following_error must be off, report or fault, got '" + value + "'");
}

int parse_mode(const std::string& value)
{
  const long long mode = read_whole_number("mode_of_operation", value);
  if (mode != IMPEDANCE_MODE && mode != 8 && mode != 9)
  {
    throw std::invalid_argument("mode_of_operation must be -6, 8 or 9, got '" + value + "'");
  }
  return static_cast<int>(mode);
}
}  // namespace

struct MockCia402Hal::JointConfig
{
  std::string name;
  bool energize{ true };
  SimulatedDriveSettings drive;
};

MockCia402Hal::MockCia402Hal() : Cia402Hal(LOGGER_NAME)
{
}

MockCia402Hal::~MockCia402Hal() = default;

bool MockCia402Hal::setup(const HalConfig& config)
{
  if (connected_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "setup() refused: the HAL is brought up, disconnect() first.");
    return false;
  }
  configured_ = false;
  joints_.clear();
  double control_frequency = 0.0;
  bool quick_stop_hold = true;
  std::vector<JointConfig> joints;
  try
  {
    const auto& hw = config.hardware_parameters;
    control_frequency = read_non_negative("control_frequency", "Hz", param_or(hw, "control_frequency", "500"));
    quick_stop_hold = read_flag(param_or(hw, "quick_stop_hold", "true"));
    const std::string default_mode = param_or(hw, "mode_of_operation", "8");
    for (const auto& spec : config.joints)
    {
      const auto& p = spec.parameters;
      JointConfig joint;
      joint.name = spec.name;
      joint.energize = read_flag(param_or(p, "energize", "true"));
      joint.drive.mode_of_operation = parse_mode(param_or(p, "mode_of_operation", default_mode));
      joint.drive.quick_stop_option_code =
          static_cast<int>(read_whole_number("quick_stop_option_code", param_or(p, "quick_stop_option_code", "6")));
      joint.drive.quick_stop_deceleration =
          read_non_negative("quick_stop_deceleration", "rad/s^2", param_or(p, "quick_stop_deceleration", "20.0"));
      if (std::isnan(joint.drive.quick_stop_deceleration))
      {
        throw std::invalid_argument("quick_stop_deceleration is empty");
      }
      joint.drive.following_error = parse_following_error(param_or(p, "following_error", "off"));
      joints.push_back(std::move(joint));
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "setup(): invalid parameter: %s", e.what());
    return false;
  }
  if (joints.empty() || !(control_frequency > 0.0))
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "setup(): needs at least one joint and a positive frequency.");
    return false;
  }
  control_frequency_ = control_frequency;
  quick_stop_hold_ = quick_stop_hold;
  joints_ = std::move(joints);
  configured_ = true;
  return true;
}

bool MockCia402Hal::connect()
{
  if (connected_)
  {
    return true;
  }
  if (!configured_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "connect() needs setup().");
    return false;
  }
  std::vector<JointSettings> settings;
  drives_.clear();
  for (const auto& joint : joints_)
  {
    settings.push_back(
        { joint.name, joint.energize, quick_stop_hold_ && joint.drive.mode_of_operation == IMPEDANCE_MODE });
    drives_.emplace_back(joint.drive);
  }
  init_joints(std::move(settings), control_frequency_);
  states_.assign(joints_.size(), DriveState{});
  commands_.assign(joints_.size(), DriveCommand{});
  controlwords_.assign(joints_.size(), 0);
  connected_ = true;
  return true;
}

bool MockCia402Hal::start()
{
  if (streaming_)
  {
    return true;
  }
  if (!connected_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "start() needs connect().");
    return false;
  }
  streaming_ = true;
  last_step_ = std::chrono::steady_clock::now();
  exchange_read();
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Simulation running: %zu drive(s) exchange valid data, all unpowered.",
              drives_.size());
  return true;
}

bool MockCia402Hal::read(std::vector<DriveState>& states)
{
  if (!streaming_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "read() before start().");
    return false;
  }
  exchange_read();
  states = states_;
  return true;
}

bool MockCia402Hal::stop()
{
  // A drive can have power without enable(), from a drive_action, so the drives always go down first.
  if (streaming_ && !disable())
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "stop() refused: the drives did not all lose their power.");
    return false;
  }
  streaming_ = false;
  enabled_ = false;
  return true;
}

bool MockCia402Hal::disconnect()
{
  stop();
  streaming_ = false;
  enabled_ = false;
  connected_ = false;
  drives_.clear();
  return true;
}

std::size_t MockCia402Hal::joint_count() const
{
  return joints_.size();
}

bool MockCia402Hal::is_connected() const
{
  return connected_;
}

bool MockCia402Hal::quick_stop_available(std::size_t joint) const
{
  const int code = joint < joints_.size() ? joints_[joint].drive.quick_stop_option_code : 0;
  return code == 6 || code == 2;
}

bool MockCia402Hal::exchange_read()
{
  const auto now = std::chrono::steady_clock::now();
  // One step per read, like one bus cycle: a CSP target arrives once per cycle.
  const double dt = std::min(MAX_STEP, std::chrono::duration<double>(now - last_step_).count());
  last_step_ = now;
  if (dt > 0.0)
  {
    for (auto& drive : drives_)
    {
      drive.step(dt);
    }
  }
  for (std::size_t j = 0; j < drives_.size(); ++j)
  {
    states_[j] = drives_[j].state();
  }
  return true;
}

bool MockCia402Hal::exchange_write()
{
  for (std::size_t j = 0; j < drives_.size(); ++j)
  {
    drives_[j].command(controlwords_[j], commands_[j]);
  }
  return true;
}

uint16_t MockCia402Hal::statusword(std::size_t joint) const
{
  return states_[joint].status_word;
}

uint16_t MockCia402Hal::error_code(std::size_t joint) const
{
  return states_[joint].error_code;
}

uint16_t MockCia402Hal::last_controlword(std::size_t joint) const
{
  return controlwords_[joint];
}

double MockCia402Hal::speed(std::size_t joint) const
{
  return std::abs(states_[joint].velocity);
}

double MockCia402Hal::transport_position(std::size_t joint) const
{
  return states_[joint].position;
}

void MockCia402Hal::write_controlword(std::size_t joint, uint16_t controlword)
{
  controlwords_[joint] = controlword;
}

void MockCia402Hal::write_setpoints(std::size_t joint, const DriveCommand& command)
{
  // A value that is not sendable keeps the one sent last.
  auto& out = commands_[joint];
  for (const auto field : { &DriveCommand::position, &DriveCommand::velocity, &DriveCommand::effort,
                            &DriveCommand::stiffness, &DriveCommand::damping, &DriveCommand::max_joint_torque })
  {
    if (is_sendable_setpoint(field, command.*field))
    {
      out.*field = command.*field;
    }
  }
}

void MockCia402Hal::write_hold(std::size_t joint, double position)
{
  auto& out = commands_[joint];
  out.position = position;
  out.velocity = 0.0;
  out.effort = 0.0;
}

}  // namespace synapticon_hardware

PLUGINLIB_EXPORT_CLASS(synapticon_hardware::MockCia402Hal, synapticon_hardware::SynapticonHalI)
