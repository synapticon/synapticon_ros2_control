// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief EthercatPdoHal: the process data arrays of the EtherCAT HALs, and the steps that work on them.

#include "synapticon_hardware/ethercat/ethercat_pdo_hal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "synapticon_hardware/ethercat/coe_objects.hpp"

namespace synapticon_hardware::ethercat
{

namespace
{
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

/// `value_si` divided by the factor of its unit. With a factor of 0, the value is passed unchanged.
double to_raw(double value_si, double si_per_raw)
{
  return si_per_raw != 0.0 ? value_si / si_per_raw : value_si;
}

/// The array to a drive as it starts. Targets are NaN: they are not valid values, so no motion can come from them.
/// Every slot that the drive gets each cycle without a NaN check starts at 0 (controlword, stiffness, damping,
/// torque ceiling). The CiA 402 logic and the bring-up checks replace these later.
std::vector<double> initial_outputs()
{
  std::vector<double> out(layout::OUTPUT_SIZE, NaN);
  for (const std::size_t zeroed : { slot::CONTROL_WORD, slot::STIFFNESS, slot::DAMPING, slot::TORQUE_LIMIT })
  {
    out[zeroed] = 0.0;
  }
  return out;
}
}  // namespace

EthercatPdoHal::EthercatPdoHal(std::string logger_name) : Cia402Hal(std::move(logger_name))
{
}

// Quick stop values from bring-up.

bool EthercatPdoHal::quick_stop_available(std::size_t joint) const
{
  return is_supported_quick_stop(quick_stop_option_code(joint));
}

int EthercatPdoHal::quick_stop_option_code(std::size_t joint) const
{
  return joint < option_codes_.size() ? option_codes_[joint] : -1;
}

double EthercatPdoHal::quick_stop_deceleration(std::size_t joint) const
{
  return joint < quick_stop_decelerations_.size() ? quick_stop_decelerations_[joint] : NaN;
}

// Steps of connect().

bool EthercatPdoHal::set_up_drives(const std::vector<JointConfig>& joints, bool quick_stop_hold,
                                   double control_frequency)
{
  slave_config_units_.clear();
  for (const JointConfig& joint : joints)
  {
    try
    {
      slave_config_units_.push_back(read_slave_config_units(joint.slave_config_file));
    }
    catch (const std::invalid_argument& e)
    {
      RCLCPP_ERROR(log(), "Joint '%s': %s", joint.name.c_str(), e.what());
      return false;
    }
  }
  outputs_.assign(joints.size(), initial_outputs());
  inputs_.assign(joints.size(), std::vector<double>(layout::INPUT_SIZE, 0.0));
  si_factors_.assign(joints.size(), UNREAD_SI_FACTORS);
  option_codes_.assign(joints.size(), -1);
  quick_stop_decelerations_.assign(joints.size(), NaN);

  std::vector<JointSettings> settings(joints.size());
  std::transform(joints.begin(), joints.end(), settings.begin(), [quick_stop_hold](const JointConfig& joint) {
    // The drive does not brake in impedance mode, so on a quick stop the HAL keeps the target where it was.
    return JointSettings{ joint.name, joint.energize, quick_stop_hold && joint.mode_of_operation == mode::IMPEDANCE };
  });
  init_joints(std::move(settings), control_frequency);
  return true;
}

bool EthercatPdoHal::check_drives(const std::vector<JointConfig>& joints, const SdoRetry& retry)
{
  std::vector<DriveReadback> readback;
  const RetryingSdo sdo(*this, *this, retry, log());
  const bool usable =
      synapticon_hardware::ethercat::check_drives(sdo, sdo, joints, slave_config_units_, log(), readback);
  for (std::size_t j = 0; j < readback.size(); ++j)
  {
    apply_readback(j, readback[j]);
  }
  return usable;
}

// Step of start(), and the conversion of the states.

bool EthercatPdoHal::wait_for_streaming(const std::vector<JointConfig>& joints, double control_frequency,
                                        double timeout_s)
{
  using Clock = std::chrono::steady_clock;
  const auto give_up =
      Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_s));
  const auto cycle = std::chrono::duration<double>(1.0 / control_frequency);
  for (;;)
  {
    if (!exchange_read() || !exchange_write())
    {
      RCLCPP_ERROR(log(), "start(): the process data exchange failed while waiting for the first valid data.");
      return false;
    }
    if (all_streaming(joints))
    {
      return true;
    }
    if (Clock::now() >= give_up)
    {
      break;
    }
    std::this_thread::sleep_for(cycle);
  }
  RCLCPP_ERROR(log(),
               "start(): no valid data from every drive within %.1f s of OP (statusword still 0). Check the PDO "
               "mapping of the slave_config, or raise readiness_timeout_s for a slow bus.",
               timeout_s);
  return false;
}

void EthercatPdoHal::unpack_states(std::vector<DriveState>& states) const
{
  states.resize(inputs_.size());
  for (std::size_t j = 0; j < inputs_.size(); ++j)
  {
    read_channels(STATE_WORDS, layout::WORDS, inputs_[j], si_factors_[j], states[j]);
    read_channels(STATE_BYTES, layout::BYTES, inputs_[j], si_factors_[j], states[j]);
    read_channels(STATE_REALS, layout::REALS, inputs_[j], si_factors_[j], states[j]);
  }
}

// Cia402Hal hooks.

uint16_t EthercatPdoHal::statusword(std::size_t joint) const
{
  return register_at(inputs_[joint], slot::STATUS_WORD);
}

uint16_t EthercatPdoHal::error_code(std::size_t joint) const
{
  return register_at(inputs_[joint], slot::ERROR_CODE);
}

uint16_t EthercatPdoHal::last_controlword(std::size_t joint) const
{
  return register_at(outputs_[joint], slot::CONTROL_WORD);
}

double EthercatPdoHal::speed(std::size_t joint) const
{
  return std::abs(inputs_[joint][slot::ACTUAL_VELOCITY] * si_factor(si_factors_[joint], Unit::ANGULAR_VELOCITY));
}

double EthercatPdoHal::transport_position(std::size_t joint) const
{
  return inputs_[joint][slot::ACTUAL_POSITION];  // raw counts
}

void EthercatPdoHal::write_controlword(std::size_t joint, uint16_t controlword)
{
  outputs_[joint][slot::CONTROL_WORD] = controlword;
}

void EthercatPdoHal::write_setpoints(std::size_t joint, const DriveCommand& command)
{
  const SiFactors& factors = si_factors_[joint];
  for (std::size_t c = 0; c < std::size(COMMAND_CHANNELS); ++c)
  {
    const auto& channel = COMMAND_CHANNELS[c];
    const double value_si = command.*(channel.field);
    if (std::isfinite(value_si) || !holds_last_value(channel.extra))
    {
      outputs_[joint][layout::COMMANDS + c] = to_raw(value_si, si_factor(factors, channel.extra));
    }
  }
}

void EthercatPdoHal::write_hold(std::size_t joint, double position)
{
  // Target the given position with zero velocity and zero torque offset, so the joint stays put.
  auto& out = outputs_[joint];
  out[slot::TORQUE_OFFSET] = 0.0;
  out[slot::TARGET_VELOCITY] = 0.0;
  out[slot::TARGET_POSITION] = position;
}

std::optional<uint32_t> EthercatPdoHal::sdo_u32(uint16_t position, CoeAddress object) const
{
  return read_u32(*this, log(), position, object);
}

// Helpers.

void EthercatPdoHal::apply_readback(std::size_t joint, const DriveReadback& readback)
{
  si_factors_[joint] = readback.si_factors;
  option_codes_[joint] = readback.quick_stop_option_code;
  quick_stop_decelerations_[joint] = readback.quick_stop_deceleration;
  const std::pair<std::size_t, std::optional<double>> seeds[] = {
    { slot::STIFFNESS, readback.stiffness },
    { slot::DAMPING, readback.damping },
    { slot::TORQUE_LIMIT, readback.max_torque },
  };
  for (const auto& [index, value] : seeds)
  {
    if (value)
    {
      outputs_[joint][index] = *value;
    }
  }
}

bool EthercatPdoHal::all_streaming(const std::vector<JointConfig>& joints) const
{
  // Statusword 0 means the drive has no valid process data yet. Joints that are never powered do not count.
  for (std::size_t j = 0; j < joints.size(); ++j)
  {
    if (joints[j].energize && statusword(j) == 0)
    {
      return false;
    }
  }
  return true;
}

}  // namespace synapticon_hardware::ethercat
