// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The link between the channels of a slave_config and the fields of DriveCommand and DriveState.
///
/// An EtherCAT HAL keeps two arrays of doubles per joint, one going to the drive and one coming back. The tables in
/// this file give every channel a fixed place (its slot) in those arrays, a field and a unit. The HAL tells its
/// master the slot of each channel by name. The cyclic code then works with the slot numbers only.
/// Channels of the slave_config that are not listed here are sent with their default_value.

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <type_traits>
#include <vector>

#include "synapticon_hardware/drive_data.hpp"
#include "synapticon_hardware/interface_binding.hpp"

namespace synapticon_hardware::ethercat
{

/// @brief What one raw step of a channel is worth. Multiplying by the unit's factor gives the SI value.
///
/// Three units depend on the drive and get their factor at bring-up: ANGLE (one encoder count), ANGULAR_VELOCITY
/// (one step of 0x60A9) and MOTOR_TORQUE (a thousandth of the rated torque). The others are fixed milli units:
/// OUTPUT_TORQUE is mNm at the output shaft (0x2052, 0x2053), STIFFNESS mNm/rad, DAMPING mNm s/rad, TORQUE_LIMIT mNm,
/// VOLTAGE mV and CURRENT mA. TEMPERATURE is taken as a thousandth of a degree C, since the ESI gives no unit.
/// RAW values pass unchanged. COUNT is the number of units, used as an array size.
enum class Unit : std::size_t
{
  RAW,
  ANGLE,
  ANGULAR_VELOCITY,
  MOTOR_TORQUE,
  OUTPUT_TORQUE,
  STIFFNESS,
  DAMPING,
  TORQUE_LIMIT,
  VOLTAGE,
  CURRENT,
  TEMPERATURE,
  COUNT
};

/// One factor per Unit, for one joint. All are on the motor side, except OUTPUT_TORQUE.
using SiFactors = std::array<double, static_cast<std::size_t>(Unit::COUNT)>;

/// The factor that `factors` holds for `unit`.
inline double si_factor(const SiFactors& factors, Unit unit)
{
  return factors[static_cast<std::size_t>(unit)];
}

/// The factor that `factors` holds for `unit`, to change it.
inline double& si_factor(SiFactors& factors, Unit unit)
{
  return factors[static_cast<std::size_t>(unit)];
}

/// Factors to start with: 0.001 for the fixed milli units, and 1 for RAW and the units that bring-up still has to
/// read from the drive.
constexpr SiFactors unread_si_factors()
{
  SiFactors factors{};
  factors.fill(1.0e-3);
  for (const Unit read_from_drive : { Unit::RAW, Unit::ANGLE, Unit::ANGULAR_VELOCITY, Unit::MOTOR_TORQUE })
  {
    factors[static_cast<std::size_t>(read_from_drive)] = 1.0;
  }
  return factors;
}
inline constexpr SiFactors UNREAD_SI_FACTORS = unread_si_factors();

/// One row of a channel table: the name in the slave_config, the field it fills or comes from, and its Unit.
template <typename FieldT, typename OwnerT>
using Channel = NamedField<FieldT, OwnerT, Unit>;

/// The commands, sent to the drive every cycle in this order after the controlword.
constexpr Channel<double, DriveCommand> COMMAND_CHANNELS[] = {
  { "position", &DriveCommand::position, Unit::ANGLE },
  { "velocity", &DriveCommand::velocity, Unit::ANGULAR_VELOCITY },
  { "joint_torque_offset", &DriveCommand::effort, Unit::OUTPUT_TORQUE },
  { "max_joint_torque", &DriveCommand::max_joint_torque, Unit::TORQUE_LIMIT },
  { "stiffness", &DriveCommand::stiffness, Unit::STIFFNESS },
  { "damping", &DriveCommand::damping, Unit::DAMPING },
};

/// The states that come back from the drive, in three tables by field type. The 16-bit and 8-bit registers keep
/// their raw value.
constexpr Channel<uint16_t, DriveState> STATE_WORDS[] = {
  { "status_word", &DriveState::status_word },
  { "error_code", &DriveState::error_code },
};
constexpr Channel<int8_t, DriveState> STATE_BYTES[] = {
  { "mode_of_operation", &DriveState::mode_of_operation },
};
/// The states in SI units. error_report is the exception: its double holds eight characters, which must not be scaled.
constexpr Channel<double, DriveState> STATE_REALS[] = {
  { "position", &DriveState::position, Unit::ANGLE },
  { "velocity", &DriveState::velocity, Unit::ANGULAR_VELOCITY },
  { "effort", &DriveState::effort, Unit::MOTOR_TORQUE },
  { "joint_torque_demand", &DriveState::joint_torque_demand, Unit::OUTPUT_TORQUE },
  { "motor_current", &DriveState::motor_current, Unit::CURRENT },
  { "dc_link_voltage", &DriveState::dc_link_voltage, Unit::VOLTAGE },
  { "temperature_core", &DriveState::temperature_core, Unit::TEMPERATURE },
  { "temperature_inverter", &DriveState::temperature_inverter, Unit::TEMPERATURE },
  { "error_report", &DriveState::error_report, Unit::RAW },
};

/// The channel name of the controlword. It is set by the CiA 402 logic of the HAL, never by a DriveCommand.
inline constexpr std::string_view CONTROL_WORD_CHANNEL = "control_word";

/// First slot of each table, and the array sizes. Going to the drive: the controlword at slot 0, then the commands.
/// Coming back: the 16-bit registers, the 8-bit register, then the SI values.
namespace layout
{
inline constexpr std::size_t CONTROL_WORD = 0;
inline constexpr std::size_t COMMANDS = 1;
inline constexpr std::size_t OUTPUT_SIZE = COMMANDS + std::size(COMMAND_CHANNELS);
inline constexpr std::size_t WORDS = 0;
inline constexpr std::size_t BYTES = WORDS + std::size(STATE_WORDS);
inline constexpr std::size_t REALS = BYTES + std::size(STATE_BYTES);
inline constexpr std::size_t INPUT_SIZE = REALS + std::size(STATE_REALS);
}  // namespace layout

namespace detail
{
/// The slot of `name` in `table`, which starts at `first`, or nothing.
template <typename Table>
constexpr std::optional<std::size_t> find_in(const Table& table, std::size_t first, std::string_view name)
{
  for (std::size_t i = 0; i < std::size(table); ++i)
  {
    if (name == table[i].name)
    {
      return first + i;
    }
  }
  return std::nullopt;
}
}  // namespace detail

/// The slot of channel `name` in the array to the drive, or nothing when no table lists it.
constexpr std::optional<std::size_t> output_slot(std::string_view name)
{
  if (name == CONTROL_WORD_CHANNEL)
  {
    return layout::CONTROL_WORD;
  }
  return detail::find_in(COMMAND_CHANNELS, layout::COMMANDS, name);
}

/// The slot of channel `name` in the array from the drive, or nothing when no table lists it.
constexpr std::optional<std::size_t> input_slot(std::string_view name)
{
  if (const auto word = detail::find_in(STATE_WORDS, layout::WORDS, name))
  {
    return word;
  }
  if (const auto byte = detail::find_in(STATE_BYTES, layout::BYTES, name))
  {
    return byte;
  }
  return detail::find_in(STATE_REALS, layout::REALS, name);
}

/// Call `visit(name, slot)` once per channel to the drive, starting with the controlword.
template <typename Visit>
void for_each_output_slot(Visit&& visit)
{
  visit(CONTROL_WORD_CHANNEL, layout::CONTROL_WORD);
  for (std::size_t i = 0; i < std::size(COMMAND_CHANNELS); ++i)
  {
    visit(std::string_view(COMMAND_CHANNELS[i].name), layout::COMMANDS + i);
  }
}

/// Call `visit(name, slot)` once per channel from the drive.
template <typename Visit>
void for_each_input_slot(Visit&& visit)
{
  const auto table = [&visit](const auto& channels, std::size_t first) {
    for (std::size_t i = 0; i < std::size(channels); ++i)
    {
      visit(std::string_view(channels[i].name), first + i);
    }
  };
  table(STATE_WORDS, layout::WORDS);
  table(STATE_BYTES, layout::BYTES);
  table(STATE_REALS, layout::REALS);
}

/// Named slots, for code that reads or writes one channel directly.
namespace slot
{
inline constexpr std::size_t CONTROL_WORD = layout::CONTROL_WORD;
inline constexpr std::size_t TARGET_POSITION = *output_slot("position");
inline constexpr std::size_t TARGET_VELOCITY = *output_slot("velocity");
inline constexpr std::size_t TORQUE_OFFSET = *output_slot("joint_torque_offset");
inline constexpr std::size_t TORQUE_LIMIT = *output_slot("max_joint_torque");
inline constexpr std::size_t STIFFNESS = *output_slot("stiffness");
inline constexpr std::size_t DAMPING = *output_slot("damping");
inline constexpr std::size_t STATUS_WORD = *input_slot("status_word");
inline constexpr std::size_t ERROR_CODE = *input_slot("error_code");
inline constexpr std::size_t MODE_DISPLAY = *input_slot("mode_of_operation");
inline constexpr std::size_t ACTUAL_POSITION = *input_slot("position");
inline constexpr std::size_t ACTUAL_VELOCITY = *input_slot("velocity");
inline constexpr std::size_t ACTUAL_TORQUE = *input_slot("effort");
inline constexpr std::size_t ERROR_REPORT = *input_slot("error_report");
}  // namespace slot

/// `value` as a FieldT. For an integer field the value is cut to a whole number via int64_t (no rounding) and then
/// wraps to the field's width.
template <typename FieldT>
FieldT to_field(double value)
{
  if constexpr (std::is_integral_v<FieldT>)
  {
    return static_cast<FieldT>(static_cast<int64_t>(value));
  }
  else
  {
    return static_cast<FieldT>(value);
  }
}

/// Fill the fields of `state` named in `table` from the array `input`, where the table starts at slot `first`.
/// Every value is multiplied by its unit's factor.
template <typename FieldT, std::size_t N>
void read_channels(const Channel<FieldT, DriveState> (&table)[N], std::size_t first, const std::vector<double>& input,
                   const SiFactors& factors, DriveState& state)
{
  for (std::size_t i = 0; i < N; ++i)
  {
    const auto& channel = table[i];
    state.*(channel.field) = to_field<FieldT>(input[first + i] * si_factor(factors, channel.extra));
  }
}

/// The value at `index` of `buffer` as a 16-bit register.
inline uint16_t register_at(const std::vector<double>& buffer, std::size_t index)
{
  return to_field<uint16_t>(buffer[index]);
}

}  // namespace synapticon_hardware::ethercat
