// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Reading single values from a HalConfig: lookup with a fallback, and flags.

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "synapticon_hardware/hal_config.hpp"

namespace
{
using synapticon_hardware::HalConfig;
using synapticon_hardware::JointSpec;
using synapticon_hardware::param_or;
using synapticon_hardware::ParameterMap;
using synapticon_hardware::read_flag;

TEST(ReadFlag, RefusesWordsOutsideTheFourSpellings)
{
  for (const char* text : { "on", "off", "", " true", "true ", "10", "-1", "t" })
  {
    EXPECT_THROW(read_flag(text), std::invalid_argument) << "'" << text << "'";
  }
}

TEST(ReadFlag, IgnoresLetterCase)
{
  for (const char* text : { "true", "True", "tRuE", "1" })
  {
    EXPECT_TRUE(read_flag(text)) << text;
  }
  for (const char* text : { "false", "FALSE", "fAlse", "0" })
  {
    EXPECT_FALSE(read_flag(text)) << text;
  }
}

TEST(ReadFlag, NamesTheRefusedTextInTheError)
{
  try
  {
    read_flag("maybe");
    FAIL() << "no exception";
  }
  catch (const std::invalid_argument& e)
  {
    EXPECT_NE(std::string(e.what()).find("maybe"), std::string::npos) << e.what();
  }
}

TEST(ParamOr, KeepsAnEmptyValueThatIsSet)
{
  const ParameterMap map{ { "plugin", "" } };
  EXPECT_EQ(param_or(map, "plugin", "fallback"), "");
}

TEST(ParamOr, FallsBackOnlyForAMissingName)
{
  const ParameterMap map{ { "control_frequency", "500" } };
  EXPECT_EQ(param_or(map, "control_frequency", "1000"), "500");
  EXPECT_EQ(param_or(map, "Control_Frequency", "1000"), "1000") << "names are case sensitive";
  EXPECT_EQ(param_or(map, "quick_stop_hold", "true"), "true");
  EXPECT_TRUE(param_or(map, "quick_stop_hold").empty()) << "the fallback defaults to the empty string";
}

TEST(HalConfig, KeepsTheJointsInTheOrderGiven)
{
  HalConfig config;
  config.hardware_parameters["hal"] = "mock";
  for (const char* name : { "lift", "conveyor", "gripper" })
  {
    config.joints.push_back(JointSpec{ name, { { "position", std::to_string(config.joints.size()) } } });
  }
  ASSERT_EQ(config.joints.size(), 3u);
  EXPECT_EQ(config.joints[0].name, "lift");
  EXPECT_EQ(config.joints[2].name, "gripper");
  EXPECT_EQ(param_or(config.joints[1].parameters, "position"), "1");
  EXPECT_EQ(param_or(config.hardware_parameters, "hal"), "mock");
}

}  // namespace
