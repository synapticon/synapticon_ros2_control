// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Merging a robot-definition YAML into the configuration of an EtherCAT HAL.

#include <filesystem>
#include <fstream>
#include <string>

#include <gmock/gmock.h>

#include "synapticon_hardware/ethercat/robot_config.hpp"
#include "synapticon_hardware/hal_config.hpp"

namespace
{
using synapticon_hardware::HalConfig;
using synapticon_hardware::JointSpec;
using synapticon_hardware::ethercat::apply_robot_config;

/// A robot definition with bus settings, two drive joints in reverse order and a joint without a drive.
constexpr const char* ROBOT_YAML = R"(
bus:
  master_id: 1
  control_frequency: 500
joints:
  - name: joint_2
    alias: 0
    position: 1
    slave_config: ""
    max_joint_torque: ""
  - name: joint_1
    position: 0
    slave_config: /drives/joint_1.yaml
    invert: true
    mode_of_operation: 9
  - name: output
    drive: false
transmissions: []
)";

/// Write `content` to a file in the test's temporary directory and return its path.
std::string write_file(const std::string& name, const std::string& content)
{
  const auto path = std::filesystem::temp_directory_path() / ("test_robot_config_" + name);
  std::ofstream(path) << content;
  return path.string();
}

/// A HalConfig for the drive joints joint_1 and joint_2, reading `robot_config` when it is set.
HalConfig make_config(const std::string& robot_config)
{
  HalConfig config;
  if (!robot_config.empty())
  {
    config.hardware_parameters["robot_config"] = robot_config;
  }
  config.hardware_parameters["default_slave_config"] = "/drives/default.yaml";
  config.hardware_parameters["mode_of_operation"] = "-6";
  config.joints = { JointSpec{ "joint_1", {} }, JointSpec{ "joint_2", {} } };
  return config;
}

/// The bus entries become hardware parameters, the joint entries are matched to the joints by name.
TEST(RobotConfig, FillsTheBusAndEachJointByName)
{
  HalConfig config = make_config(write_file("by_name.yaml", ROBOT_YAML));
  std::string error;
  ASSERT_TRUE(apply_robot_config(config, error)) << error;
  EXPECT_EQ(config.hardware_parameters.at("master_id"), "1");
  EXPECT_EQ(config.hardware_parameters.at("control_frequency"), "500");
  EXPECT_EQ(config.joints[0].parameters.at("invert"), "true");
  EXPECT_EQ(config.joints[0].parameters.at("slave_config"), "/drives/joint_1.yaml");
  EXPECT_EQ(config.joints[1].parameters.at("position"), "1");
  EXPECT_EQ(config.joints[0].parameters.count("name"), 0u);
}

/// A joint without its own slave_config or mode takes the defaults, an empty value counts as absent.
TEST(RobotConfig, AppliesTheDefaultsToJointsWithoutTheirOwn)
{
  HalConfig config = make_config(write_file("defaults.yaml", ROBOT_YAML));
  std::string error;
  ASSERT_TRUE(apply_robot_config(config, error)) << error;
  EXPECT_EQ(config.joints[1].parameters.at("slave_config"), "/drives/default.yaml");
  EXPECT_EQ(config.joints[1].parameters.at("mode_of_operation"), "-6");
  EXPECT_EQ(config.joints[1].parameters.count("max_joint_torque"), 0u);
  EXPECT_EQ(config.joints[0].parameters.at("mode_of_operation"), "9");
}

/// command_defaults become hardware parameters and fill the joints without their own value.
TEST(RobotConfig, AppliesTheCommandDefaults)
{
  const std::string yaml = std::string(ROBOT_YAML) +
                           "command_defaults: {stiffness: 0.4, damping: 0.1, max_joint_torque: 50, "
                           "keep_last_given: true}\n";
  HalConfig config = make_config(write_file("command_defaults.yaml", yaml));
  config.joints[0].parameters["stiffness"] = "2.0";
  std::string error;
  ASSERT_TRUE(apply_robot_config(config, error)) << error;
  EXPECT_EQ(config.hardware_parameters.at("default_stiffness"), "0.4");
  EXPECT_EQ(config.hardware_parameters.at("keep_last_given"), "true");
  EXPECT_EQ(config.joints[0].parameters.at("stiffness"), "2.0");
  EXPECT_EQ(config.joints[0].parameters.at("damping"), "0.1");
  EXPECT_EQ(config.joints[1].parameters.at("stiffness"), "0.4");
  EXPECT_EQ(config.joints[1].parameters.at("max_joint_torque"), "50");
}

/// A parameter the description sets itself wins over the file.
TEST(RobotConfig, ADescriptionParameterWinsOverTheFile)
{
  HalConfig config = make_config(write_file("precedence.yaml", ROBOT_YAML));
  config.hardware_parameters["control_frequency"] = "1000";
  config.joints[0].parameters["invert"] = "false";
  std::string error;
  ASSERT_TRUE(apply_robot_config(config, error)) << error;
  EXPECT_EQ(config.hardware_parameters.at("control_frequency"), "1000");
  EXPECT_EQ(config.joints[0].parameters.at("invert"), "false");
}

/// Without a robot definition only the defaults apply.
TEST(RobotConfig, WithoutAFileOnlyTheDefaultsApply)
{
  HalConfig config = make_config("");
  config.joints[0].parameters["slave_config"] = "/drives/own.yaml";
  std::string error;
  ASSERT_TRUE(apply_robot_config(config, error)) << error;
  EXPECT_EQ(config.joints[0].parameters.at("slave_config"), "/drives/own.yaml");
  EXPECT_EQ(config.joints[1].parameters.at("slave_config"), "/drives/default.yaml");
}

/// A drive joint the file does not list is refused, so no drive runs on guessed addressing.
TEST(RobotConfig, RejectsADriveJointWithoutAnEntry)
{
  HalConfig config = make_config(write_file("missing.yaml", ROBOT_YAML));
  config.joints.push_back(JointSpec{ "joint_3", {} });
  std::string error;
  EXPECT_FALSE(apply_robot_config(config, error));
  EXPECT_THAT(error, ::testing::HasSubstr("joint_3"));
}

/// A missing file, a file that is not a map and one without a joints list are refused.
TEST(RobotConfig, RejectsWhatIsNotARobotDefinition)
{
  std::string error;
  HalConfig missing = make_config("/nonexistent/robot.yaml");
  EXPECT_FALSE(apply_robot_config(missing, error));
  HalConfig list = make_config(write_file("list.yaml", "- a\n- b\n"));
  EXPECT_FALSE(apply_robot_config(list, error));
  HalConfig no_joints = make_config(write_file("no_joints.yaml", "bus:\n  master_id: 0\n"));
  EXPECT_FALSE(apply_robot_config(no_joints, error));
  EXPECT_THAT(error, ::testing::HasSubstr("joints"));
}
}  // namespace
