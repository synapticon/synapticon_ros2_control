// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The drive to joint map and the HAL configuration built from a description.

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <hardware_interface/hardware_info.hpp>

#include "synapticon_hardware/drive_joint_map.hpp"
#include "description_builders.hpp"

namespace
{
using synapticon_hardware::build_drive_joint_map;
using synapticon_hardware::build_hal_config;
using synapticon_hardware::test::joint_info;
using synapticon_hardware::test::simple_transmission;

/// A joint declaration with an optional position parameter.
hardware_interface::ComponentInfo joint(const std::string& name, const std::string& position = "")
{
  return position.empty() ? joint_info(name) : joint_info(name, {}, {}, { { "position", position } });
}

/// Two drives, then an output joint driven through a transmission by the second drive.
hardware_interface::HardwareInfo transmission_robot()
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["network_interface"] = "eth0";
  info.joints = { joint("joint_1", "0"), joint("motor", "1"), joint("output") };
  info.transmissions.push_back(simple_transmission("tx", "output", "motor"));
  return info;
}

/// Without transmissions the map is the identity.
TEST(DriveJointMap, WithoutTransmissionsEveryJointHasADrive)
{
  hardware_interface::HardwareInfo info;
  info.joints = { joint("a"), joint("b") };
  const auto map = build_drive_joint_map(info);
  EXPECT_EQ(map.drive_count(), 2u);
  EXPECT_EQ(map.drive_to_joint, (std::vector<std::size_t>{ 0, 1 }));
  EXPECT_EQ(map.joint_to_drive, (std::vector<int>{ 0, 1 }));
  EXPECT_TRUE(map.has_drive(1));
}

/// A joint served only by a transmission has no drive, its actuator keeps one.
TEST(DriveJointMap, ATransmissionOnlyJointHasNoDrive)
{
  const auto map = build_drive_joint_map(transmission_robot());
  EXPECT_EQ(map.drive_count(), 2u);
  EXPECT_EQ(map.drive_to_joint, (std::vector<std::size_t>{ 0, 1 }));
  EXPECT_EQ(map.joint_to_drive, (std::vector<int>{ 0, 1, -1 }));
  EXPECT_FALSE(map.has_drive(2));
  EXPECT_TRUE(map.has_drive(1)) << "the actuator keeps its drive";
}

/// The HAL configuration carries the hardware parameters and only the drive-backed joints, in drive order.
TEST(DriveJointMap, TheHalConfigListsOnlyTheDriveBackedJoints)
{
  const auto info = transmission_robot();
  const auto config = build_hal_config(info, build_drive_joint_map(info));
  EXPECT_EQ(config.hardware_parameters.at("network_interface"), "eth0");
  ASSERT_EQ(config.joints.size(), 2u);
  EXPECT_EQ(config.joints[0].name, "joint_1");
  EXPECT_EQ(config.joints[0].parameters.at("position"), "0");
  EXPECT_EQ(config.joints[1].name, "motor");
  EXPECT_EQ(config.joints[1].parameters.at("position"), "1");
}

}  // namespace
