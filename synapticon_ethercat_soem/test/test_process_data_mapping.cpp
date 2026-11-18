// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Resetting SOEM's process data mapping, on a context filled as a mapping leaves it.

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "process_data_mapping.hpp"

namespace
{
using synapticon_ethercat_soem::forget_process_data_mapping;

/// A context with two slaves, each using two FMMUs and pointing into `io_map`, as after one mapping.
struct MappedContext
{
  MappedContext()
  {
    context.slavelist = slaves.data();
    context.slavecount = &slave_count;
    context.maxslave = static_cast<int>(slaves.size());
    context.grouplist = groups.data();
    context.maxgroup = static_cast<int>(groups.size());
    for (int s = 1; s <= slave_count; ++s)
    {
      ec_slavet& slave = slaves[s];
      slave.configadr = static_cast<uint16>(0x1000 + s);
      slave.FMMUunused = 2;
      slave.FMMU[0].LogStart = 16;
      slave.FMMU[1].LogStart = 32;
      slave.outputs = io_map.data();
      slave.inputs = io_map.data() + 8;
    }
    groups[0].blockLRW = 1;
    groups[0].Ebuscurrent = 120;
  }

  std::vector<uint8_t> io_map = std::vector<uint8_t>(64, 0);
  std::vector<ec_slavet> slaves = std::vector<ec_slavet>(4);
  std::vector<ec_groupt> groups = std::vector<ec_groupt>(2);
  int slave_count{ 2 };
  ecx_contextt context{};
};

TEST(ForgetProcessDataMapping, FreesEveryFmmuAndBufferPointerOfEachSlave)
{
  MappedContext mapped;
  forget_process_data_mapping(mapped.context);
  for (int s = 1; s <= mapped.slave_count; ++s)
  {
    const ec_slavet& slave = mapped.slaves[s];
    EXPECT_EQ(slave.FMMUunused, 0);
    for (const ec_fmmut& fmmu : slave.FMMU)
    {
      EXPECT_EQ(fmmu.LogStart, 0u);
    }
    EXPECT_EQ(slave.outputs, nullptr);
    EXPECT_EQ(slave.inputs, nullptr);
  }
}

TEST(ForgetProcessDataMapping, KeepsWhatTheBusScanFound)
{
  MappedContext mapped;
  forget_process_data_mapping(mapped.context);
  EXPECT_EQ(mapped.slaves[1].configadr, 0x1001);
  EXPECT_EQ(mapped.slaves[2].configadr, 0x1002);
}

TEST(ForgetProcessDataMapping, ResetsTheGroupTotalsOfTheMapping)
{
  MappedContext mapped;
  forget_process_data_mapping(mapped.context);
  EXPECT_EQ(mapped.groups[0].blockLRW, 0);
  EXPECT_EQ(mapped.groups[0].Ebuscurrent, 0);
}

}  // namespace
