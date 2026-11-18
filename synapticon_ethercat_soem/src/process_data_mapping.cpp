// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Resetting SOEM's process data mapping.

#include "process_data_mapping.hpp"

#include <cstring>

namespace synapticon_ethercat_soem
{

void forget_process_data_mapping(ecx_contextt& context)
{
  for (int s = 1; s <= *context.slavecount; ++s)
  {
    ec_slavet& slave = context.slavelist[s];
    slave.FMMUunused = 0;
    std::memset(slave.FMMU, 0, sizeof(slave.FMMU));
    slave.outputs = nullptr;
    slave.inputs = nullptr;
  }
  // Mapping adds these up over the slaves.
  for (int g = 0; g < context.maxgroup; ++g)
  {
    context.grouplist[g].blockLRW = 0;
    context.grouplist[g].Ebuscurrent = 0;
  }
}

}  // namespace synapticon_ethercat_soem
