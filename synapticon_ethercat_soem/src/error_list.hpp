// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Reading SOEM's error list: the abort code of one SDO transfer, and a log line for everything else.

#include <cstdint>
#include <optional>

extern "C" {
#include "ethercat.h"
}

namespace synapticon_ethercat_soem
{

/// The object of one SDO transfer, with the SOEM slave number (counted from 1).
struct SdoAddress
{
  uint16_t slave{ 0 };
  uint16_t index{ 0 };
  uint8_t sub_index{ 0 };
};

/// Empty SOEM's error list. It collects SDO aborts, emergencies, mailbox and packet errors of every slave, oldest
/// first. Returns the abort code of the SDO abort for `transfer`, or 0 when the list holds none. Every other entry is
/// logged as a warning.
uint32_t drain_error_list(ecx_contextt& context, const std::optional<SdoAddress>& transfer = std::nullopt);

}  // namespace synapticon_ethercat_soem
