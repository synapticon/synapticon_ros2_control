// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The test gains that the hil_tests section of a robot definition gives the bench joints.

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "synapticon_hardware/testing/hal_bench.hpp"

namespace
{
using synapticon_hardware::testing::own_test_gains;
using synapticon_hardware::testing::test_gains;
using synapticon_hardware::testing::TestGains;

const char* const ROBOT = "joints:\n"
                          "  - { name: hip }\n"
                          "  - { name: knee }\n";

TEST(TestGains, AreTheDefaultsWithoutAHilTestsSection)
{
  const TestGains gains = test_gains(YAML::Load(ROBOT), 0);
  EXPECT_EQ(gains.stiffness, TestGains{}.stiffness);
  EXPECT_EQ(gains.damping, TestGains{}.damping);
  EXPECT_FALSE(own_test_gains(gains));
}

TEST(TestGains, TakeTheJointsEntryOverTheSectionsValues)
{
  const auto robot = YAML::Load(std::string(ROBOT) + "hil_tests:\n"
                                                     "  stiffness: 4.0\n"
                                                     "  joints:\n"
                                                     "    knee: { damping: 0.2 }\n");
  const TestGains hip = test_gains(robot, 0);
  const TestGains knee = test_gains(robot, 1);
  EXPECT_EQ(hip.stiffness, 4.0);
  EXPECT_EQ(hip.damping, TestGains{}.damping);
  EXPECT_EQ(knee.stiffness, 4.0);
  EXPECT_EQ(knee.damping, 0.2);
  EXPECT_TRUE(own_test_gains(hip));
  EXPECT_TRUE(own_test_gains(knee));
}

TEST(TestGains, CountAsOwnOnlyWhereTheyDifferFromTheDefaults)
{
  const auto robot = YAML::Load(std::string(ROBOT) + "hil_tests:\n"
                                                     "  joints:\n"
                                                     "    knee: { stiffness: 1.0 }\n");
  EXPECT_FALSE(own_test_gains(test_gains(robot, 0)));
  EXPECT_TRUE(own_test_gains(test_gains(robot, 1)));
}

TEST(TestGains, RefuseAJointTheRobotDoesNotHave)
{
  const auto robot = YAML::Load(std::string(ROBOT) + "hil_tests:\n"
                                                     "  joints:\n"
                                                     "    ankle: { stiffness: 1.0 }\n");
  EXPECT_THROW(test_gains(robot, 0), std::runtime_error);
}

}  // namespace
