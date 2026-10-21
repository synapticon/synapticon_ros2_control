// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The units a drive is commissioned in, and the slave_config entries whose meaning depends on them.
///
/// The stack never commissions a drive's units. It reads them at bring-up, understands them completely or refuses the
/// drive, and converts the values it writes from SI units into the drive's own.

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "synapticon_hardware/ethercat/coe_objects.hpp"
#include "synapticon_hardware/ethercat/pdo_channels.hpp"

namespace synapticon_hardware::ethercat
{

/// The velocity of one raw unit in rad/s for the velocity unit `velocity_unit` (the content of 0x60A9), or nothing
/// when it is not one this driver understands. Understood: a power of ten from 10^-3 to 10^12, revolutions (0xB4),
/// per minute (0x47) or per second (0x03), and a last byte of 0.
std::optional<double> velocity_step(uint32_t velocity_unit);

/// The value of 0x6092:1 that disables the feed constant, so that positions are encoder counts and velocities
/// revolutions.
inline constexpr uint32_t FEED_CONSTANT_DISABLED = 0xFFFFFFFFu;

/// What the value of a unit-dependent object is counted in. Each is on the motor side, like the drive's objects.
enum class SiQuantity
{
  VELOCITY,      ///< rad/s, one step of the velocity unit
  ACCELERATION,  ///< rad/s^2, one step of the velocity unit per second
  POSITION,      ///< rad, one count of the position encoder
  TORQUE,        ///< Nm, a thousandth of the rated torque
};

/// The SI unit of `quantity`, for messages.
const char* si_unit_name(SiQuantity quantity);

/// What `object` is counted in, when the stack knows it to depend on the drive's units.
std::optional<SiQuantity> unit_dependent_quantity(CoeAddress object);

/// True for the objects that need no unit: option codes, gains in fixed units and times.
bool unit_free(CoeAddress object);

/// True for the objects that set the units themselves. Changing them is commissioning, not bring-up.
bool defines_units(CoeAddress object);

/// One `sdo_si:` entry of a slave_config: a value in SI units, converted at bring-up and written after the units were
/// read.
struct SiSdo
{
  CoeAddress object{ 0, 0 };
  std::string type;     ///< as in the slave_config, an integer type
  double value{ 0.0 };  ///< in the SI unit of `quantity`, the slave_config key `si_value`
  SiQuantity quantity{ SiQuantity::VELOCITY };
};

/// The `units:` section of a slave_config: the drive's units, for a drive that cannot report them. Each is used only
/// where the drive cannot be read or understood, and must agree with what the drive reports otherwise.
struct UnitOverrides
{
  std::optional<uint32_t> velocity_unit;          ///< as in 0x60A9
  std::optional<uint32_t> counts_per_revolution;  ///< of the position encoder
  std::optional<double> rated_torque;             ///< Nm
  bool feed_constant_disabled{ false };           ///< `feed_constant: disabled`
};

/// What the stack takes from a slave_config beyond the PDO mapping: the SI entries and the unit overrides.
struct SlaveConfigUnits
{
  std::vector<SiSdo> si_sdo;
  UnitOverrides overrides;
};

/// @brief Read and check the `sdo:`, `sdo_si:` and `units:` sections of the slave_config at `path`.
///
/// An `sdo:` entry must be unit-free, or carry `raw: true`. An `sdo_si:` entry must give `si_value` for an
/// integer-typed object whose unit the stack knows. Neither may set the units themselves.
/// @throws std::invalid_argument naming the file and the entry when a rule is broken or a value is malformed.
SlaveConfigUnits read_slave_config_units(const std::string& path);

/// @brief `entry.value` as the raw value of the drive with the unit factors `factors`, rounded to the nearest unit.
///
/// Empty, with the reason in `error`, when the value does not fit the type, a value other than 0 rounds to 0, or the
/// rounding changes it by more than one percent.
std::optional<int64_t> si_to_raw(const SiSdo& entry, const SiFactors& factors, std::string& error);

/// The little-endian bytes of `raw` for the integer `type` of a slave_config (`int8` to `uint32`).
std::vector<uint8_t> raw_bytes(int64_t raw, const std::string& type);

}  // namespace synapticon_hardware::ethercat
