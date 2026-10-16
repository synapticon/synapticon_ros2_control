// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The link health a HAL reports, and the healthy default.

#include <gtest/gtest.h>

#include "synapticon_hardware/link_health.hpp"
#include "synapticon_hardware/mock_hal.hpp"

namespace
{
namespace al_state = synapticon_hardware::al_state;
namespace wc_state = synapticon_hardware::wc_state;
using synapticon_hardware::healthy_link;
using synapticon_hardware::MockHal;

/// healthy_link() reports every drive online in OP, the link up and the domain complete.
TEST(LinkHealth, TheHealthyLinkHasEveryDriveOnlineInOp)
{
  const auto health = healthy_link(3);
  ASSERT_EQ(health.drives.size(), 3u);
  for (const auto& drive : health.drives)
  {
    EXPECT_EQ(drive.al_state, al_state::OP);
    EXPECT_TRUE(drive.online);
  }
  EXPECT_TRUE(health.bus.link_up);
  EXPECT_EQ(health.bus.slaves_responding, 3u);
  EXPECT_EQ(health.bus.al_states, al_state::OP);
  EXPECT_EQ(health.bus.domain_wc_state, wc_state::COMPLETE);
}

/// A HAL without a link layer reports the healthy default for its joints.
TEST(LinkHealth, AHalWithoutALinkLayerReportsEverythingHealthy)
{
  MockHal hal;
  ASSERT_TRUE(hal.setup({ "a", "b" }));
  synapticon_hardware::LinkHealth health;
  hal.link_health(health);
  ASSERT_EQ(health.drives.size(), 2u);
  EXPECT_TRUE(health.drives[1].online);
  EXPECT_TRUE(health.bus.link_up);
  EXPECT_EQ(health.bus.slaves_responding, 2u);
}

/// A drive counts as operational only when it answers and is in OP.
TEST(LinkHealth, OnlyAnAnsweringDriveInOpIsOperational)
{
  using synapticon_hardware::EthercatSlaveHealth;
  using synapticon_hardware::is_operational;
  EXPECT_TRUE(is_operational(EthercatSlaveHealth{ al_state::OP, true }));
  EXPECT_FALSE(is_operational(EthercatSlaveHealth{ al_state::OP, false }));
  EXPECT_FALSE(is_operational(EthercatSlaveHealth{ al_state::SAFEOP, true }));
  EXPECT_FALSE(is_operational(EthercatSlaveHealth{ 0, true }));
}

}  // namespace
