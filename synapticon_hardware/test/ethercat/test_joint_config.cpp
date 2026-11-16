// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Parsing the joint parameters of a drive on an EtherCAT bus.

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "synapticon_hardware/ethercat/joint_config.hpp"

namespace
{
using synapticon_hardware::ethercat::joint_from;
using synapticon_hardware::ethercat::JointConfig;
using synapticon_hardware::ethercat::shared_drive_error;
using Parameters = std::unordered_map<std::string, std::string>;

/// The required parameter and `extra` on top.
Parameters with(const Parameters& extra)
{
  Parameters params{ { "slave_config", "/drives/joint.yaml" } };
  params.insert(extra.begin(), extra.end());
  return params;
}

TEST(JointConfig, OnlyTheSlaveConfigIsRequired)
{
  const auto joint = joint_from("joint_1", with({}));
  EXPECT_EQ(joint.name, "joint_1");
  EXPECT_EQ(joint.slave_config_file, "/drives/joint.yaml");
  EXPECT_EQ(joint.alias, 0);
  EXPECT_EQ(joint.position, 0);
  EXPECT_EQ(joint.mode_of_operation, -6);
  EXPECT_TRUE(std::isnan(joint.max_joint_torque));
  EXPECT_TRUE(joint.energize);
  EXPECT_FALSE(joint.invert);
}

TEST(JointConfig, ReadsEveryParameter)
{
  const auto joint = joint_from("joint_1", with({ { "alias", "7" },
                                                  { "position", "65535" },
                                                  { "mode_of_operation", "9" },
                                                  { "max_joint_torque", "2.5" },
                                                  { "stiffness", "0.4" },
                                                  { "damping", "0.1" },
                                                  { "energize", "false" },
                                                  { "invert", "true" } }));
  EXPECT_EQ(joint.alias, 7);
  EXPECT_EQ(joint.position, 65535);
  EXPECT_EQ(joint.mode_of_operation, 9);
  EXPECT_DOUBLE_EQ(joint.max_joint_torque, 2.5);
  EXPECT_DOUBLE_EQ(joint.stiffness, 0.4);
  EXPECT_DOUBLE_EQ(joint.damping, 0.1);
  EXPECT_FALSE(joint.energize);
  EXPECT_TRUE(joint.invert);
}

TEST(JointConfig, AnEmptyTorqueLimitKeepsTheDrives)
{
  EXPECT_TRUE(std::isnan(joint_from("joint_1", with({ { "max_joint_torque", "" } })).max_joint_torque));
}

class InvalidJoint : public ::testing::TestWithParam<Parameters>
{
};

TEST_P(InvalidJoint, IsRefused)
{
  Parameters params = GetParam();
  EXPECT_THROW(joint_from("joint_1", params), std::invalid_argument);
}

INSTANTIATE_TEST_SUITE_P(
    Cases, InvalidJoint,
    ::testing::Values(Parameters{}, with({ { "position", "65536" } }), with({ { "alias", "first" } }),
                      with({ { "mode_of_operation", "10" } }), with({ { "max_joint_torque", "-1" } }),
                      with({ { "max_joint_torque", "inf" } }), with({ { "invert", "maybe" } }),
                      with({ { "alias", "-1" } }), with({ { "position", "+3" } }), with({ { "position", "" } }),
                      with({ { "max_joint_torque", "nan" } }), with({ { "stiffness", "-0.1" } }),
                      with({ { "damping", "soft" } }), with({ { "position", "0x03" } }), with({ { "position", "3x" } }),
                      with({ { "position", "1.5" } }), with({ { "alias", " 1" } }),
                      with({ { "position", "99999999999999999999" } }), with({ { "mode_of_operation", "8.5" } }),
                      with({ { "mode_of_operation", "8x" } }), with({ { "max_joint_torque", "2Nm" } })));

TEST(JointConfig, AnAddressErrorNamesTheParameter)
{
  try
  {
    joint_from("joint_1", with({ { "position", "70000" } }));
    FAIL() << "no exception";
  }
  catch (const std::invalid_argument& e)
  {
    EXPECT_NE(std::string(e.what()).find("position"), std::string::npos) << e.what();
  }
}

/// Joints `name` at `alias` and `position`.
JointConfig at(const std::string& name, uint16_t alias, uint16_t position)
{
  JointConfig joint;
  joint.name = name;
  joint.alias = alias;
  joint.position = position;
  return joint;
}

TEST(SharedDrive, IsFoundByTheSameAliasAndPosition)
{
  const auto error = shared_drive_error({ at("first", 0, 0), at("second", 0, 1), at("third", 0, 0) });
  ASSERT_TRUE(error);
  EXPECT_NE(error->find("'first' and 'third'"), std::string::npos) << *error;
}

TEST(SharedDrive, IsNoneForDistinctAddresses)
{
  EXPECT_FALSE(shared_drive_error({ at("first", 0, 0), at("second", 0, 1), at("third", 7, 0) }));
}

}  // namespace
