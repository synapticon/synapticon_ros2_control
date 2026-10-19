// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Resetting SOEM's process data mapping, so that a later start() maps it anew.

extern "C" {
#include "ethercat.h"
}

namespace synapticon_ethercat_soem
{

/// Forget the process data mapping of every slave in `context`, as ecx_config_init() leaves it. Mapping again then
/// starts from the first FMMU and points the slaves into the new buffer. Without this, SOEM appends the new mapping
/// to the FMMUs used before, which run out after a few mappings, and keeps each slave's old buffer pointers.
void forget_process_data_mapping(ecx_contextt& context);

}  // namespace synapticon_ethercat_soem
