// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief One drive's process image for the EtherCAT HALs that move the bytes themselves.

#include "synapticon_hardware/ethercat/process_image.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>

#include "synapticon_hardware/ethercat/coe_objects.hpp"
#include "synapticon_hardware/ethercat/pdo_channels.hpp"

namespace synapticon_hardware::ethercat
{

namespace
{
namespace ec = synapticon_hardware::ethercat;

constexpr uint16_t OBJ_MODE_OF_OPERATION = 0x6060;
constexpr uint16_t OBJ_MODE_OF_OPERATION_DISPLAY = 0x6061;
constexpr uint16_t OBJ_POSITION_ACTUAL = 0x6064;
constexpr uint16_t OBJ_TARGET_POSITION = 0x607A;

/// The channels of `pdos` one after the other, from bit 0, with their slots. Returns the size in bytes.
template <typename SlotOf>
std::size_t lay_out(const std::vector<Pdo>& pdos, SlotOf slot_of, std::vector<MappedChannel>& mapped)
{
  std::size_t bits = 0;
  for (const auto& pdo : pdos)
  {
    for (const auto& channel : pdo.channels)
    {
      const std::string& name = channel.command_interface.empty() ? channel.state_interface : channel.command_interface;
      mapped.push_back({ channel, bits, name.empty() ? std::nullopt : slot_of(name) });
      bits += channel.type.bits;
    }
  }
  return (bits + 7) / 8;
}

/// The channels of `mapped` at the bit offsets `locate` gives, in `moved`. False, with `error` set, at the first
/// channel that has no place or would change its position within a byte.
bool place(const std::vector<MappedChannel>& mapped, const ChannelLocator& locate, const char* section,
           std::vector<MappedChannel>& moved, std::string& error)
{
  moved = mapped;
  for (auto& entry : moved)
  {
    char object[16];
    std::snprintf(object, sizeof(object), "0x%04X:%u", entry.channel.index, entry.channel.sub_index);
    const auto offset = locate(entry.channel);
    if (!offset)
    {
      error = std::string(section) + " channel " + object + " is not in the master's process data";
      return false;
    }
    if (*offset % 8 != entry.bit_offset % 8)
    {
      error = std::string(section) + " channel " + object + " is at bit " + std::to_string(*offset % 8) +
              " of a byte in the master's process data, but at bit " + std::to_string(entry.bit_offset % 8) +
              " in the slave_config";
      return false;
    }
    entry.bit_offset = *offset;
  }
  return true;
}

/// The little-endian integer of `bits` bits at byte `at` of `bytes`.
uint64_t read_unsigned(const uint8_t* bytes, std::size_t at, uint8_t bits)
{
  uint64_t value = 0;
  for (std::size_t i = bits / 8; i-- > 0;)
  {
    value = (value << 8) | bytes[at + i];
  }
  return value;
}

/// `value` as the raw bits of an integer field of `type`: saturated to the field's range, except that a negative value
/// of an unsigned field wraps as two's complement. NaN is 0. A BITS field counts as one unsigned byte.
uint64_t integer_bits(double value, const ValueType& type)
{
  if (std::isnan(value))
  {
    return 0;
  }
  const bool is_signed = type.kind == ValueKind::SIGNED;
  const int bits = type.kind == ValueKind::BITS ? 8 : type.bits;
  const int magnitude = is_signed ? bits - 1 : bits;
  if (value >= std::ldexp(1.0, magnitude))
  {
    return magnitude == 64 ? UINT64_MAX : (uint64_t{ 1 } << magnitude) - 1;
  }
  const double lowest = is_signed ? -std::ldexp(1.0, magnitude) : -std::ldexp(1.0, 63);
  if (value <= lowest)
  {
    return static_cast<uint64_t>(static_cast<int64_t>(lowest));
  }
  return value >= std::ldexp(1.0, 63) ? static_cast<uint64_t>(value) :
                                        static_cast<uint64_t>(static_cast<int64_t>(value));
}
}  // namespace

double read_value(const uint8_t* bytes, std::size_t bit_offset, const ValueType& type, uint8_t mask)
{
  const std::size_t at = bit_offset / 8;
  switch (type.kind)
  {
    case ValueKind::BITS:
      return type.bits == 1 ? ((bytes[at] & mask) != 0 ? 1.0 : 0.0) : static_cast<double>(bytes[at] & mask);
    case ValueKind::REAL:
      if (type.bits == 32)
      {
        float value;
        std::memcpy(&value, bytes + at, sizeof(value));
        return value;
      }
      else
      {
        double value;
        std::memcpy(&value, bytes + at, sizeof(value));
        return value;
      }
    case ValueKind::SIGNED:
    {
      const uint64_t raw = read_unsigned(bytes, at, type.bits);
      const uint64_t sign = uint64_t{ 1 } << (type.bits - 1);
      return static_cast<double>(static_cast<int64_t>((raw ^ sign) - sign));
    }
    case ValueKind::UNSIGNED:
      return static_cast<double>(read_unsigned(bytes, at, type.bits));
  }
  return 0.0;
}

void write_value(uint8_t* bytes, std::size_t bit_offset, const ValueType& type, uint8_t mask, double value)
{
  const std::size_t at = bit_offset / 8;
  switch (type.kind)
  {
    case ValueKind::BITS:
      if (type.bits == 1)
      {
        bytes[at] = value != 0.0 ? static_cast<uint8_t>(bytes[at] | mask) : static_cast<uint8_t>(bytes[at] & ~mask);
      }
      else
      {
        const auto bits = static_cast<uint8_t>(integer_bits(value, type));
        bytes[at] = static_cast<uint8_t>((bytes[at] & ~mask) | (bits & mask));
      }
      return;
    case ValueKind::REAL:
      if (type.bits == 32)
      {
        const auto real = static_cast<float>(value);
        std::memcpy(bytes + at, &real, sizeof(real));
      }
      else
      {
        std::memcpy(bytes + at, &value, sizeof(value));
      }
      return;
    case ValueKind::SIGNED:
    case ValueKind::UNSIGNED:
    {
      auto raw = integer_bits(value, type);
      for (std::size_t i = 0; i < type.bits / 8; ++i)
      {
        bytes[at + i] = static_cast<uint8_t>(raw & 0xFF);
        raw >>= 8;
      }
      return;
    }
  }
}

ProcessImage::ProcessImage(const SlaveConfig& config, int mode_of_operation) : mode_of_operation_(mode_of_operation)
{
  output_bytes_ = lay_out(config.rpdo, ec::output_slot, outputs_);
  input_bytes_ = lay_out(config.tpdo, ec::input_slot, inputs_);
}

bool ProcessImage::relocate(const ChannelLocator& locate_output, const ChannelLocator& locate_input, std::string& error)
{
  std::vector<MappedChannel> outputs;
  std::vector<MappedChannel> inputs;
  if (!place(outputs_, locate_output, "rpdo", outputs, error) || !place(inputs_, locate_input, "tpdo", inputs, error))
  {
    return false;
  }
  outputs_ = std::move(outputs);
  inputs_ = std::move(inputs);
  return true;
}

std::size_t ProcessImage::output_bytes() const
{
  return output_bytes_;
}

std::size_t ProcessImage::input_bytes() const
{
  return input_bytes_;
}

void ProcessImage::read_inputs(const uint8_t* bytes, std::vector<double>& inputs)
{
  for (const auto& mapped : inputs_)
  {
    const PdoChannel& channel = mapped.channel;
    if (channel.skip)
    {
      continue;
    }
    const double value =
        channel.factor * read_value(bytes, mapped.bit_offset, channel.type, channel.mask) + channel.offset;
    if (mapped.slot)
    {
      inputs[*mapped.slot] = value;
    }
    if (channel.index == OBJ_POSITION_ACTUAL)
    {
      last_position_ = value;
    }
    else if (channel.index == OBJ_MODE_OF_OPERATION_DISPLAY)
    {
      last_mode_display_ = static_cast<int>(value);
    }
  }
}

std::optional<double> ProcessImage::output_value(const MappedChannel& mapped, const std::vector<double>& outputs) const
{
  const PdoChannel& channel = mapped.channel;
  if (channel.command_interface.empty())
  {
    return std::nullopt;  // no command, so nothing is written, whatever the default
  }
  double fallback = channel.default_value;
  bool use_command = true;
  if (channel.index == OBJ_MODE_OF_OPERATION && mode_of_operation_ >= 0 && mode_of_operation_ <= 10)
  {
    fallback = mode_of_operation_;  // a standard mode from the joint, a vendor mode from the slave_config
  }
  else if (channel.index == OBJ_TARGET_POSITION && last_mode_display_ != 0)
  {
    // The actual position, unless the drive follows the commanded target: in CSP and in impedance mode.
    fallback = channel.factor * last_position_ + channel.offset;
    use_command = last_mode_display_ == ec::mode::CSP || last_mode_display_ == ec::mode::IMPEDANCE;
  }
  else if (channel.index == OBJ_TARGET_POSITION)
  {
    use_command = false;  // before the drive reports a mode, only a default target reaches it
  }
  if (use_command && mapped.slot && !std::isnan(outputs[*mapped.slot]))
  {
    return channel.factor * outputs[*mapped.slot] + channel.offset;
  }
  if (!std::isnan(fallback))
  {
    return fallback;
  }
  return std::nullopt;
}

void ProcessImage::write_outputs(const std::vector<double>& outputs, uint8_t* bytes) const
{
  for (const auto& mapped : outputs_)
  {
    if (mapped.channel.skip)
    {
      continue;
    }
    if (const auto value = output_value(mapped, outputs))
    {
      write_value(bytes, mapped.bit_offset, mapped.channel.type, mapped.channel.mask, *value);
    }
  }
}

}  // namespace synapticon_hardware::ethercat
