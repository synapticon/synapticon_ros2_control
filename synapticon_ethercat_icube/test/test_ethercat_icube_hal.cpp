// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Parameter validation in EthercatIcubeHal::setup().
///
/// setup() performs no IO, so the validation of the string parameters runs without a bus.

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "synapticon_hardware/hal_config.hpp"
#include "synapticon_ethercat_icube/ethercat_icube_hal.hpp"

namespace
{

using synapticon_ethercat_icube::EthercatIcubeHal;
using synapticon_hardware::HalConfig;
using synapticon_hardware::JointSpec;

/// A single-joint HalConfig with `extra_params` on top of the required slave_config (a placeholder
/// path, since setup() does not open it).
HalConfig make_config(const std::unordered_map<std::string, std::string>& extra_params)
{
  JointSpec joint;
  joint.name = "joint_1";
  joint.parameters["slave_config"] = "placeholder.yaml";
  for (const auto& [key, value] : extra_params)
  {
    joint.parameters[key] = value;
  }
  HalConfig config;
  config.joints.push_back(std::move(joint));
  return config;
}

/// A valid single-joint HalConfig with the given bus-wide hardware parameters.
HalConfig make_bus_config(const std::unordered_map<std::string, std::string>& hardware_params)
{
  HalConfig config = make_config({});
  config.hardware_parameters = hardware_params;
  return config;
}

// --- Required structure ----------------------------------------------------

/// A joint with only its slave_config is a valid configuration.
TEST(EthercatIcubeHalSetup, MinimalConfigAccepted)
{
  EthercatIcubeHal client;
  EXPECT_TRUE(client.setup(make_config({})));
}

/// A configured HAL has no link yet: no stage of the readiness query holds before connect().
TEST(EthercatIcubeHalSetup, NoStageHoldsBeforeBringUp)
{
  EthercatIcubeHal client;
  ASSERT_TRUE(client.setup(make_config({})));
  EXPECT_FALSE(client.is_connected());
  EXPECT_FALSE(client.is_streaming());
  EXPECT_FALSE(client.is_enabled());
  EXPECT_FALSE(client.is_ready());
}

/// The joint settings come from the robot-definition YAML the description names, and a drive joint
/// the file does not list is refused.
TEST(EthercatIcubeHalSetup, ReadsTheJointsFromTheRobotConfig)
{
  const auto path = std::filesystem::temp_directory_path() / "test_ethercat_icube_hal_robot.yaml";
  std::ofstream(path) << "joints:\n  - name: joint_1\n    position: 3\n    slave_config: placeholder.yaml\n";
  HalConfig config;
  config.hardware_parameters["robot_config"] = path.string();
  config.joints.push_back(JointSpec{ "joint_1", {} });
  EthercatIcubeHal client;
  EXPECT_TRUE(client.setup(config));

  config.joints.push_back(JointSpec{ "joint_2", { { "slave_config", "placeholder.yaml" } } });
  EthercatIcubeHal unlisted;
  EXPECT_FALSE(unlisted.setup(config));
}

/// joint_count() gives the joints of setup() before connect() as well.
TEST(EthercatIcubeHalSetup, CountsTheJointsBeforeConnect)
{
  EthercatIcubeHal client;
  EXPECT_EQ(client.joint_count(), 0u);
  ASSERT_TRUE(client.setup(make_config({})));
  EXPECT_EQ(client.joint_count(), 1u);
}

/// A failed setup() keeps nothing, also not the configuration of a setup() that succeeded before, in both overloads.
TEST(EthercatIcubeHalSetup, AFailedSetupKeepsNoConfiguration)
{
  EthercatIcubeHal typed;
  synapticon_hardware::ethercat::JointConfig drive;
  drive.name = "joint_1";
  drive.slave_config_file = "placeholder.yaml";
  ASSERT_TRUE(typed.setup({}, { synapticon_ethercat_icube::EthercatJointConfig{ drive } }));
  EXPECT_FALSE(typed.setup({}, {}));
  EXPECT_EQ(typed.joint_count(), 0u);
  EXPECT_FALSE(typed.connect()) << "connect() must not bring the bus up with the earlier configuration";

  EthercatIcubeHal described;
  ASSERT_TRUE(described.setup(make_config({})));
  EXPECT_FALSE(described.setup(make_config({ { "mode_of_operation", "4" } })));
  EXPECT_EQ(described.joint_count(), 0u);
  EXPECT_FALSE(described.connect());
}

/// A configuration without joints is refused.
TEST(EthercatIcubeHalSetup, RejectsNoJoints)
{
  EthercatIcubeHal client;
  EXPECT_FALSE(client.setup(HalConfig{}));
}

/// A joint without a slave_config is refused.
TEST(EthercatIcubeHalSetup, RejectsMissingSlaveConfig)
{
  HalConfig config = make_config({});
  config.joints[0].parameters.erase("slave_config");
  EthercatIcubeHal client;
  EXPECT_FALSE(client.setup(config));
}

/// Two joints at the same alias and ring position are refused: both would command the one drive there.
TEST(EthercatIcubeHalSetup, RejectsTwoJointsOnTheSameDrive)
{
  HalConfig config = make_config({});
  config.joints.push_back(JointSpec{ "joint_2", { { "slave_config", "placeholder.yaml" } } });
  EthercatIcubeHal client;
  EXPECT_FALSE(client.setup(config));
  config.joints[1].parameters["position"] = "1";
  EthercatIcubeHal distinct;
  EXPECT_TRUE(distinct.setup(config));
}

// --- Bus parameters --------------------------------------------------------

/// Numeric master_id, control_frequency and readiness_timeout_s are accepted.
TEST(EthercatIcubeHalSetup, AcceptsBusParameters)
{
  EthercatIcubeHal client;
  EXPECT_TRUE(client.setup(
      make_bus_config({ { "master_id", "1" }, { "control_frequency", "500" }, { "readiness_timeout_s", "10.0" } })));
}

/// A non-numeric bus parameter is refused.
TEST(EthercatIcubeHalSetup, RejectsNonNumericBusParameters)
{
  for (const std::string key : { "master_id", "control_frequency", "readiness_timeout_s" })
  {
    EthercatIcubeHal client;
    EXPECT_FALSE(client.setup(make_bus_config({ { key, "abc" } }))) << "bad " << key << " accepted";
  }
}

/// master_id is a plain index: no sign, fraction or trailing text.
TEST(EthercatIcubeHalSetup, RejectsAMasterIdThatIsNoIndex)
{
  for (const std::string value : { "-1", "+1", "1.5", "1x", "", " 1" })
  {
    EthercatIcubeHal client;
    EXPECT_FALSE(client.setup(make_bus_config({ { "master_id", value } }))) << "master_id '" << value << "' accepted";
  }
}

/// control_frequency and readiness_timeout_s must be finite and positive.
TEST(EthercatIcubeHalSetup, RejectsNonPositiveBusParameters)
{
  for (const std::string key : { "control_frequency", "readiness_timeout_s" })
  {
    for (const std::string value : { "0", "-1", "nan", "inf" })
    {
      EthercatIcubeHal client;
      EXPECT_FALSE(client.setup(make_bus_config({ { key, value } }))) << key << "='" << value << "' accepted";
    }
  }
}

/// quick_stop_hold takes any boolean spelling and refuses anything else.
TEST(EthercatIcubeHalSetup, ParsesTheQuickStopHoldFlag)
{
  for (const std::string value : { "true", "false", "1", "0", "False" })
  {
    EthercatIcubeHal client;
    EXPECT_TRUE(client.setup(make_bus_config({ { "quick_stop_hold", value } }))) << "'" << value << "' rejected";
  }
  EthercatIcubeHal client;
  EXPECT_FALSE(client.setup(make_bus_config({ { "quick_stop_hold", "maybe" } })));
}

// --- Per-joint numeric parameters ------------------------------------------

/// Numeric position and mode_of_operation and a plugin name are accepted.
TEST(EthercatIcubeHalSetup, AcceptsJointPlacementAndMode)
{
  EthercatIcubeHal client;
  EXPECT_TRUE(client.setup(make_config(
      { { "alias", "0" }, { "position", "7" }, { "mode_of_operation", "8" }, { "plugin", "some_package/SomeDrive" } })));
}

/// A non-numeric joint parameter is refused.
TEST(EthercatIcubeHalSetup, RejectsNonNumericJointParameters)
{
  for (const std::string key : { "alias", "position", "mode_of_operation" })
  {
    EthercatIcubeHal client;
    EXPECT_FALSE(client.setup(make_config({ { key, "abc" } }))) << "bad " << key << " accepted";
  }
}

/// position is a 16 bit EtherCAT address: larger values are refused, not wrapped.
TEST(EthercatIcubeHalSetup, RejectsOutOfRangePosition)
{
  EthercatIcubeHal accepts;
  EXPECT_TRUE(accepts.setup(make_config({ { "position", "65535" } })));
  EthercatIcubeHal refuses;
  EXPECT_FALSE(refuses.setup(make_config({ { "position", "65536" } })));
}

/// Drives are addressed by ring position: an alias other than 0 is refused, also one out of range.
TEST(EthercatIcubeHalSetup, RejectsANonZeroAlias)
{
  for (const std::string value : { "1", "65535", "65536" })
  {
    EthercatIcubeHal client;
    EXPECT_FALSE(client.setup(make_config({ { "alias", value } }))) << "alias " << value << " accepted";
  }
  EthercatIcubeHal zero;
  EXPECT_TRUE(zero.setup(make_config({ { "alias", "0" } })));
}

/// Only the impedance (-6), CSP (8) and CSV (9) modes are supported.
TEST(EthercatIcubeHalSetup, AcceptsOnlyTheSupportedModes)
{
  for (const std::string value : { "-6", "8", "9" })
  {
    EthercatIcubeHal client;
    EXPECT_TRUE(client.setup(make_config({ { "mode_of_operation", value } }))) << "mode " << value << " refused";
  }
  for (const std::string value : { "10", "1", "0", "-5" })
  {
    EthercatIcubeHal client;
    EXPECT_FALSE(client.setup(make_config({ { "mode_of_operation", value } }))) << "mode " << value << " accepted";
  }
}

// --- max_joint_torque: optional, but finite and non-negative when present ----

/// A finite, non-negative max_joint_torque [Nm] is accepted.
TEST(EthercatIcubeHalSetup, AcceptsMaxJointTorque)
{
  for (const std::string value : { "0", "12.5" })
  {
    EthercatIcubeHal client;
    EXPECT_TRUE(client.setup(make_config({ { "max_joint_torque", value } })))
        << "max_joint_torque='" << value << "' should be accepted";
  }
}

/// A negative, non-finite or non-numeric max_joint_torque is refused.
TEST(EthercatIcubeHalSetup, RejectsNegativeOrNonFiniteMaxJointTorque)
{
  for (const std::string value : { "-1.0", "nan", "inf", "abc" })
  {
    EthercatIcubeHal client;
    EXPECT_FALSE(client.setup(make_config({ { "max_joint_torque", value } })))
        << "max_joint_torque='" << value << "' should be rejected";
  }
}

// --- Boolean flags ---------------------------------------------------------

/// The invert and energize flags are optional.
TEST(EthercatIcubeHalBooleanFlags, AbsentDefaults)
{
  EthercatIcubeHal client;
  EXPECT_TRUE(client.setup(make_config({})));
}

/// The flags take words and digits in any case.
TEST(EthercatIcubeHalBooleanFlags, AcceptsBooleanSpellings)
{
  for (const std::string key : { "invert", "energize" })
  {
    for (const std::string value : { "true", "false", "1", "0", "True", "FALSE" })
    {
      EthercatIcubeHal client;
      EXPECT_TRUE(client.setup(make_config({ { key, value } }))) << key << "='" << value << "' should be accepted";
    }
  }
}

/// A flag value that is not a boolean is refused.
TEST(EthercatIcubeHalBooleanFlags, RejectsNonBoolean)
{
  for (const std::string key : { "invert", "energize" })
  {
    EthercatIcubeHal client;
    EXPECT_FALSE(client.setup(make_config({ { key, "maybe" } }))) << "bad " << key << " accepted";
  }
}

}  // namespace
