// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief A drive's slave_config YAML as the SOEM HAL reads it: identity, startup SDOs and the PDO mapping.
///
/// The format is the one of the slave_config files the EtherCAT HALs share. Only the keys the SOEM HAL needs are read.

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace synapticon_ethercat_soem
{

/// The kind of value a PDO entry or SDO carries.
enum class ValueKind : uint8_t
{
  SIGNED,
  UNSIGNED,
  REAL,  ///< IEEE floating point, copied bit for bit
  BITS,  ///< `bool` and `bitN`, read and written under the channel's mask
};

/// A slave_config type: its kind and its size in bits.
struct ValueType
{
  ValueKind kind{ ValueKind::UNSIGNED };
  uint8_t bits{ 0 };
};

/// The type named `name` in a slave_config (`int8` .. `uint64`, `real32`/`float`, `real64`/`double`, `bool`, `bitN`).
std::optional<ValueType> value_type(const std::string& name);

/// One entry of a PDO, and how it maps to the HAL's buffers.
struct PdoChannel
{
  uint16_t index{ 0 };
  uint8_t sub_index{ 0 };
  ValueType type;
  std::string command_interface;  ///< empty when the channel carries no command
  std::string state_interface;    ///< empty when the channel feeds no state
  double factor{ 1.0 };           ///< buffer value = factor * raw value + offset, and back
  double offset{ 0.0 };
  /// Written as it is while the command is NaN. Only taken for a channel with a command interface.
  double default_value{ std::numeric_limits<double>::quiet_NaN() };
  uint8_t mask{ 255 };  ///< the bits of a `bool` or `bitN` channel
  bool skip{ false };   ///< mapped, but neither read nor written
};

/// One PDO (0x16xx or 0x1Axx) and its entries, in mapping order.
struct Pdo
{
  uint16_t index{ 0 };
  std::vector<PdoChannel> channels;
};

/// One startup SDO download.
struct StartupSdo
{
  uint16_t index{ 0 };
  uint8_t sub_index{ 0 };
  ValueType type;
  std::string value;  ///< as written in the YAML, converted on download
};

/// A drive's slave_config.
struct SlaveConfig
{
  uint32_t vendor_id{ 0 };
  uint32_t product_id{ 0 };
  uint32_t assign_activate{ 0 };  ///< the DC activation, 0 for no distributed clock
  std::vector<StartupSdo> sdo;
  std::vector<Pdo> rpdo;  ///< master to drive, on sync manager 2
  std::vector<Pdo> tpdo;  ///< drive to master, on sync manager 3
};

/// @brief Read the slave_config at `path`.
/// @return nothing, with `error` set, when the file cannot be read, a required key or a type is missing or wrong, or
/// a PDO channel does not fit the byte access of the process image.
std::optional<SlaveConfig> load_slave_config(const std::string& path, std::string& error);

}  // namespace synapticon_ethercat_soem
