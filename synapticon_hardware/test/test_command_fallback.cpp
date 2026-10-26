// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The values a command falls back to where it leaves out the stiffness, the damping or the torque limit.

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <gmock/gmock.h>

#include "synapticon_hardware/command_fallback.hpp"

namespace
{
using synapticon_hardware::CommandFallback;
using synapticon_hardware::DriveCommand;
using synapticon_hardware::FallbackConfig;
using synapticon_hardware::read_command_fallbacks;
using synapticon_hardware::UnspecifiedCommandFill;

constexpr double NOT_GIVEN = std::numeric_limits<double>::quiet_NaN();

/// A robot definition with command defaults, one joint with its own stiffness and one without a drive.
constexpr const char* ROBOT_YAML = R"(
command_defaults:
  stiffness: 0.4
  damping: 0.1
  max_joint_torque: 50.0
joints:
  - {name: joint_1, stiffness: 2.0}
  - {name: joint_2}
  - {name: output, drive: false, max_joint_torque: 3.0}
)";

/// Write `content` to a file in the temporary directory and return its path.
std::string write_file(const std::string& name, const std::string& content)
{
  const auto path = std::filesystem::temp_directory_path() / ("test_command_fallback_" + name);
  std::ofstream(path) << content;
  return path.string();
}

/// A description of joint_1, joint_2 and output, reading `robot_config` when it is set.
hardware_interface::HardwareInfo description(const std::string& robot_config)
{
  hardware_interface::HardwareInfo info;
  if (!robot_config.empty())
  {
    info.hardware_parameters["robot_config"] = robot_config;
  }
  for (const char* name : { "joint_1", "joint_2", "output" })
  {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    info.joints.push_back(joint);
  }
  return info;
}

/// One joint with stiffness 0.4, damping 0.1 and torque limit 50, another with 1, 2 and 3.
FallbackConfig two_joints(bool keep_last_given = false)
{
  return FallbackConfig{ { CommandFallback{ 0.4, 0.1, 50.0 }, CommandFallback{ 1.0, 2.0, 3.0 } }, keep_last_given };
}

/// A command that gives no stiffness, damping or torque limit.
DriveCommand without_values()
{
  DriveCommand command;
  command.stiffness = NOT_GIVEN;
  command.damping = NOT_GIVEN;
  command.max_joint_torque = NOT_GIVEN;
  return command;
}

/// A joint's own entry wins over command_defaults, which fills what the entry leaves out.
TEST(CommandFallbacks, ComeFromTheJointEntryElseFromTheSection)
{
  const FallbackConfig config = read_command_fallbacks(description(write_file("section.yaml", ROBOT_YAML)));
  ASSERT_EQ(config.joints.size(), 3u);
  EXPECT_DOUBLE_EQ(config.joints[0].stiffness, 2.0);
  EXPECT_DOUBLE_EQ(config.joints[0].damping, 0.1);
  EXPECT_DOUBLE_EQ(config.joints[1].stiffness, 0.4);
  EXPECT_DOUBLE_EQ(config.joints[1].max_joint_torque, 50.0);
  EXPECT_DOUBLE_EQ(config.joints[2].max_joint_torque, 3.0);
  EXPECT_FALSE(config.keep_last_given);
}

TEST(CommandFallbacks, ReadKeepLastGiven)
{
  const std::string with_switch = std::string("command_defaults:\n  keep_last_given: true\n") +
                                  "joints:\n  - {name: joint_1}\n  - {name: joint_2}\n  - {name: output}\n";
  EXPECT_TRUE(read_command_fallbacks(description(write_file("switch.yaml", with_switch))).keep_last_given);
}

/// Without a robot definition and joint parameters there is nothing to fill in.
TEST(CommandFallbacks, AreNotGivenWithoutConfiguration)
{
  UnspecifiedCommandFill fill;
  fill.configure(read_command_fallbacks(description("")));
  EXPECT_FALSE(fill.any_configured());
  std::vector<DriveCommand> commands(3, without_values());
  fill.fill_joints(commands);
  EXPECT_TRUE(std::isnan(commands[0].stiffness));
}

/// A negative value is refused, naming the joint and the key.
TEST(CommandFallbacks, RefuseANegativeValue)
{
  const std::string yaml = "joints:\n  - {name: joint_1, damping: -1}\n  - {name: joint_2}\n  - {name: output}\n";
  try
  {
    read_command_fallbacks(description(write_file("negative.yaml", yaml)));
    FAIL() << "no exception";
  }
  catch (const std::invalid_argument& error)
  {
    EXPECT_THAT(error.what(), ::testing::HasSubstr("joint_1: damping"));
  }
}

/// By default a missing value gets the joint's fallback, also right after a command that gave one.
TEST(UnspecifiedCommandFill, GivesAMissingValueTheFallback)
{
  UnspecifiedCommandFill fill;
  fill.configure(two_joints());
  std::vector<DriveCommand> commands(2, without_values());
  commands[0].stiffness = 7.0;
  fill.fill_joints(commands);
  EXPECT_DOUBLE_EQ(commands[0].stiffness, 7.0);
  EXPECT_DOUBLE_EQ(commands[0].damping, 0.1);
  EXPECT_DOUBLE_EQ(commands[1].max_joint_torque, 3.0);

  commands.assign(2, without_values());
  fill.fill_joints(commands);
  EXPECT_DOUBLE_EQ(commands[0].stiffness, 0.4);
}

/// With keep_last_given a missing value repeats the joint's last given one, until reset() starts over.
TEST(UnspecifiedCommandFill, RepeatsTheLastGivenValueUntilReset)
{
  UnspecifiedCommandFill fill;
  fill.configure(two_joints(true));
  std::vector<DriveCommand> commands(2, without_values());
  fill.fill_joints(commands);
  EXPECT_DOUBLE_EQ(commands[0].stiffness, 0.4);

  commands[0].stiffness = 7.0;
  fill.fill_joints(commands);
  commands.assign(2, without_values());
  fill.fill_joints(commands);
  EXPECT_DOUBLE_EQ(commands[0].stiffness, 7.0);
  EXPECT_DOUBLE_EQ(commands[1].stiffness, 1.0);

  fill.reset();
  commands.assign(2, without_values());
  fill.fill_joints(commands);
  EXPECT_DOUBLE_EQ(commands[0].stiffness, 0.4);
}

/// A drive command that the transmission left without a value gets the value of the drive's own joint.
TEST(UnspecifiedCommandFill, FillsTheDrivesFromTheirOwnJoints)
{
  UnspecifiedCommandFill fill;
  fill.configure(two_joints());
  std::vector<DriveCommand> drives(1, without_values());
  drives[0].damping = 9.0;
  fill.fill_drives(drives, { 1 });
  EXPECT_DOUBLE_EQ(drives[0].stiffness, 1.0);
  EXPECT_DOUBLE_EQ(drives[0].damping, 9.0);
  EXPECT_DOUBLE_EQ(drives[0].max_joint_torque, 3.0);
}

}  // namespace
