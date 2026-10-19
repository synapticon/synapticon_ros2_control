// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The bytes the HAL writes to a drive's object dictionary: startup SDO values and PDO mapping entries.

#include <cstdint>
#include <vector>

#include "slave_config.hpp"

namespace synapticon_ethercat_soem
{

/// The little-endian bytes of a startup SDO's value, as its type gives them. A negative value of an unsigned object
/// wraps as two's complement, so -1 sets every bit. Throws std::invalid_argument or std::out_of_range when the value
/// is not a number of that type: trailing characters, or an integer too wide for the object.
std::vector<uint8_t> sdo_bytes(const StartupSdo& sdo);

/// The four bytes of the PDO mapping entry for `channel`: its index, sub-index and bit length, little-endian.
std::vector<uint8_t> pdo_entry_bytes(const PdoChannel& channel);

}  // namespace synapticon_ethercat_soem
