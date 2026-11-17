// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Counting the cycles in a row without complete process data, and only those whose frame was sent.

#include <gtest/gtest.h>

#include "bad_cycle_count.hpp"

namespace
{
using synapticon_ethercat_soem::BadCycleCount;
using Change = BadCycleCount::Change;

/// One exchange: a frame goes out and comes back `complete` or not.
Change cycle(BadCycleCount& count, bool complete)
{
  count.sent();
  return count.received(complete);
}

TEST(BadCycleCount, FailsAfterTheLimitInARowAndRecoversOnACompleteCycle)
{
  BadCycleCount count;
  count.start(3);
  EXPECT_EQ(cycle(count, false), Change::NONE);
  EXPECT_EQ(cycle(count, false), Change::NONE);
  EXPECT_TRUE(count.usable());
  EXPECT_EQ(cycle(count, false), Change::LIMIT_REACHED);
  EXPECT_FALSE(count.usable());
  EXPECT_EQ(cycle(count, false), Change::NONE);
  EXPECT_FALSE(count.usable());
  EXPECT_EQ(cycle(count, true), Change::COMPLETE_AGAIN);
  EXPECT_TRUE(count.usable());
}

TEST(BadCycleCount, ACompleteCycleEndsTheRun)
{
  BadCycleCount count;
  count.start(2);
  EXPECT_EQ(cycle(count, false), Change::NONE);
  EXPECT_EQ(cycle(count, true), Change::NONE);
  EXPECT_EQ(cycle(count, false), Change::NONE);
  EXPECT_TRUE(count.usable());
}

/// The frame in flight lasts from the send to the receive. A read without a frame sent has nothing to count.
TEST(BadCycleCount, TracksTheFrameInFlight)
{
  BadCycleCount count;
  count.start(1);
  EXPECT_FALSE(count.frame_in_flight());
  count.sent();
  EXPECT_TRUE(count.frame_in_flight());
  count.received(true);
  EXPECT_FALSE(count.frame_in_flight());
  EXPECT_TRUE(count.usable());
}

/// With a limit of 1, a single incomplete cycle makes the data unusable, and the next complete one clears it.
TEST(BadCycleCount, ALimitOfOneFailsOnTheFirstIncompleteCycle)
{
  BadCycleCount count;
  count.start(1);
  EXPECT_EQ(cycle(count, true), Change::NONE);
  EXPECT_EQ(cycle(count, false), Change::LIMIT_REACHED);
  EXPECT_FALSE(count.usable());
  EXPECT_EQ(cycle(count, true), Change::COMPLETE_AGAIN);
  EXPECT_TRUE(count.usable());
}

/// Before start() and after stop(), as on the way to OP, nothing is counted and the data stays usable.
TEST(BadCycleCount, CountsOnlyWhileStarted)
{
  BadCycleCount count;
  EXPECT_EQ(cycle(count, false), Change::NONE);
  EXPECT_TRUE(count.usable());
  EXPECT_FALSE(count.frame_in_flight());
  count.start(1);
  EXPECT_EQ(cycle(count, false), Change::LIMIT_REACHED);
  count.stop();
  EXPECT_TRUE(count.usable());
  count.start(1);
  EXPECT_TRUE(count.usable());
}

}  // namespace
