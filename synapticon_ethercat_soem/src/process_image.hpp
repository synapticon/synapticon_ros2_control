// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief One drive's process image: where each slave_config channel sits in its PDO bytes, and the conversion
/// between those bytes and the HAL's buffers.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "slave_config.hpp"

namespace synapticon_ethercat_soem
{

/// A channel, its place in the drive's output or input bytes, and its slot in the HAL's buffer.
struct MappedChannel
{
  PdoChannel channel;
  std::size_t bit_offset{ 0 };      ///< from the start of the drive's output or input bytes
  std::optional<std::size_t> slot;  ///< in the HAL's buffer, nothing for a channel the HAL has no slot for
};

/// @brief One drive's process image layout and its conversion to and from the HAL's buffers.
///
/// The PDOs of the slave_config follow each other in mapping order, each entry right after the one before. A channel
/// reads as factor * raw + offset into its buffer slot. A command channel writes factor * command + offset while its
/// command is a number, else its raw default, else leaves the bytes as they are. The mode of operation (0x6060)
/// carries the joint's mode when that is a standard one, and the target position (0x607A) carries the actual position
/// unless the drive runs CSP or impedance mode and the command is a number.
class ProcessImage
{
public:
  /// The layout of `config` for a joint in `mode_of_operation`, with its channels assigned to the buffer slots of the
  /// PDO channel tables by their interface names.
  ProcessImage(const SlaveConfig& config, int mode_of_operation);

  /// The size of the drive's output (master to drive) and input (drive to master) bytes.
  std::size_t output_bytes() const;
  std::size_t input_bytes() const;

  /// Fill the HAL's input buffer from the drive's input bytes, and note the actual position and mode for the next
  /// write_outputs().
  void read_inputs(const uint8_t* bytes, std::vector<double>& inputs);

  /// Fill the drive's output bytes from the HAL's output buffer.
  void write_outputs(const std::vector<double>& outputs, uint8_t* bytes) const;

private:
  /// The value of `channel` for the drive's bytes, or nothing to leave them unchanged.
  std::optional<double> output_value(const MappedChannel& channel, const std::vector<double>& outputs) const;

  std::vector<MappedChannel> outputs_;
  std::vector<MappedChannel> inputs_;
  std::size_t output_bytes_{ 0 };
  std::size_t input_bytes_{ 0 };
  int mode_of_operation_{ 0 };
  double last_position_{ 0.0 };  ///< 0x6064 as last read, scaled
  int last_mode_display_{ 0 };   ///< 0x6061 as last read, 0 before the drive reports a mode
};

/// The value at `bit_offset` in `bytes`, as `type` describes it. Integers are little-endian, reals copied bit for bit.
double read_value(const uint8_t* bytes, std::size_t bit_offset, const ValueType& type, uint8_t mask);

/// Write `value` at `bit_offset` in `bytes`, as `type` describes it. Integers are truncated toward zero and saturated
/// to the field's range, except that a negative value of an unsigned field wraps as two's complement.
void write_value(uint8_t* bytes, std::size_t bit_offset, const ValueType& type, uint8_t mask, double value);

}  // namespace synapticon_ethercat_soem
