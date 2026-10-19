// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Startup SDO values and PDO mapping entries as the bytes a drive takes.

#include "coe_encoding.hpp"

#include <cstring>
#include <stdexcept>
#include <string>

namespace synapticon_ethercat_soem
{

namespace
{
/// The lowest `count` bytes of `raw`, little-endian.
std::vector<uint8_t> little_endian(uint64_t raw, std::size_t count)
{
  std::vector<uint8_t> bytes(count, 0);
  for (auto& byte : bytes)
  {
    byte = static_cast<uint8_t>(raw & 0xFF);
    raw >>= 8;
  }
  return bytes;
}

/// `text` as an integer of `type`, as raw bits. A negative value of an unsigned type is taken as two's complement, so
/// its range is that of the signed type of the same width. Throws for trailing characters or a value that does not fit.
uint64_t parse_integer(const std::string& text, const ValueType& type)
{
  const int bits = type.bits;
  const int64_t lowest = bits == 64 ? INT64_MIN : -(int64_t{ 1 } << (bits - 1));
  const uint64_t highest = type.kind == ValueKind::UNSIGNED ? (bits == 64 ? UINT64_MAX : (uint64_t{ 1 } << bits) - 1) :
                                                              (uint64_t{ 1 } << (bits - 1)) - 1;
  std::size_t used = 0;
  uint64_t raw = 0;
  bool fits = false;
  if (!text.empty() && text.front() == '-')
  {
    const int64_t value = std::stoll(text, &used, 0);
    raw = static_cast<uint64_t>(value);
    fits = value >= lowest;
  }
  else
  {
    raw = std::stoull(text, &used, 0);
    fits = raw <= highest;
  }
  if (used != text.size())
  {
    throw std::invalid_argument("trailing characters in '" + text + "'");
  }
  if (!fits)
  {
    throw std::out_of_range("'" + text + "' does not fit " + std::to_string(bits) + " bits");
  }
  return raw;
}
}  // namespace

std::vector<uint8_t> sdo_bytes(const StartupSdo& sdo)
{
  if (sdo.type.kind == ValueKind::REAL)
  {
    std::vector<uint8_t> bytes(sdo.type.bits / 8, 0);
    if (sdo.type.bits == 32)
    {
      const float value = std::stof(sdo.value);
      std::memcpy(bytes.data(), &value, sizeof(value));
    }
    else
    {
      const double value = std::stod(sdo.value);
      std::memcpy(bytes.data(), &value, sizeof(value));
    }
    return bytes;
  }
  return little_endian(parse_integer(sdo.value, sdo.type), sdo.type.bits / 8);
}

std::vector<uint8_t> pdo_entry_bytes(const PdoChannel& channel)
{
  const uint32_t entry = (uint32_t{ channel.index } << 16) | (uint32_t{ channel.sub_index } << 8) | channel.type.bits;
  return little_endian(entry, 4);
}

}  // namespace synapticon_ethercat_soem
