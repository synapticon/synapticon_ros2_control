// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Counting of refused handle accesses.

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "synapticon_hardware/handle_access_report.hpp"

namespace
{
using synapticon_hardware::HandleAccessFailures;
using synapticon_hardware::HandleAccessReport;

/// total() sums every kind of failure, and any() holds once one is counted.
TEST(HandleAccessFailures, TotalSumsEveryDirection)
{
  HandleAccessFailures failures;
  EXPECT_FALSE(failures.any());
  failures.state_write = 2;
  failures.setpoint_read = 3;
  failures.action_read = 5;
  failures.command_seed_write = 7;
  EXPECT_EQ(failures.total(), 17u);
  EXPECT_TRUE(failures.any());
}

/// State writes, setpoint reads, action reads and command seeds are counted apart.
TEST(HandleAccessReport, CountsEachKindSeparately)
{
  HandleAccessReport report;
  report.state_writes(2);
  report.state_writes(0);
  report.command_reads(3, 1);
  report.command_reads(0, 4);
  report.command_seeds(1);
  EXPECT_EQ(report.failures().state_write, 2u);
  EXPECT_EQ(report.failures().setpoint_read, 3u);
  EXPECT_EQ(report.failures().action_read, 5u);
  EXPECT_EQ(report.failures().command_seed_write, 1u);
  EXPECT_EQ(report.failures().total(), 11u);
}

/// reset() clears the counts, and summarising with nothing to report succeeds.
TEST(HandleAccessReport, ResetForgetsThePreviousActivation)
{
  HandleAccessReport report;
  report.state_writes(4);
  report.summarise();
  report.reset();
  EXPECT_FALSE(report.failures().any());
  report.summarise();
}

}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
