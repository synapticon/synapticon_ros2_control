// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief EthercatSoemHal without a bus: its registration, its configuration and a bring-up that cannot reach a drive.

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_loader.hpp>

#include "synapticon_ethercat_soem/ethercat_soem_hal.hpp"
#include "synapticon_hardware/synapticon_hal.hpp"

namespace
{
using synapticon_ethercat_soem::EthercatSoemHal;
using synapticon_ethercat_soem::JointConfig;
using synapticon_ethercat_soem::SoemConfig;
using synapticon_hardware::HalConfig;
using synapticon_hardware::JointSpec;

std::string example_slave_config()
{
  return ament_index_cpp::get_package_share_directory("synapticon_hardware") +
         "/config/example_somanet_circulo_safe_motion.yaml";
}

/// One joint with the example slave_config, on `network_interface` (none when empty).
HalConfig description(const std::string& network_interface,
                      const std::unordered_map<std::string, std::string>& joint_params = {})
{
  HalConfig config;
  if (!network_interface.empty())
  {
    config.hardware_parameters["network_interface"] = network_interface;
  }
  JointSpec joint;
  joint.name = "joint_1";
  joint.parameters = { { "slave_config", example_slave_config() } };
  joint.parameters.insert(joint_params.begin(), joint_params.end());
  config.joints.push_back(joint);
  return config;
}

JointConfig joint(const std::string& slave_config)
{
  JointConfig config;
  config.name = "joint_1";
  config.slave_config_file = slave_config;
  return config;
}

SoemConfig bus(const std::string& network_interface)
{
  SoemConfig config;
  config.network_interface = network_interface;
  return config;
}

TEST(EthercatSoemHal, IsRegisteredAsEthercatSoem)
{
  pluginlib::ClassLoader<synapticon_hardware::SynapticonHalI> loader("synapticon_hardware",
                                                                     "synapticon_hardware::SynapticonHalI");
  std::shared_ptr<synapticon_hardware::SynapticonHalI> hal;
  ASSERT_NO_THROW(hal = loader.createSharedInstance("ethercat_soem"));
  EXPECT_TRUE(hal->setup(description("eth_test")));
}

// --- Configuration ------------------------------------------------------------------------------

TEST(EthercatSoemHalSetup, NeedsANetworkInterface)
{
  EthercatSoemHal hal;
  EXPECT_FALSE(hal.setup(description("")));
  EXPECT_FALSE(hal.connect());
}

TEST(EthercatSoemHalSetup, NeedsAJoint)
{
  EthercatSoemHal hal;
  EXPECT_FALSE(hal.setup(bus("eth_test"), {}));
}

TEST(EthercatSoemHalSetup, RefusesTwoJointsOnTheSameDrive)
{
  EthercatSoemHal hal;
  JointConfig second = joint(example_slave_config());
  second.name = "joint_2";
  EXPECT_FALSE(hal.setup(bus("eth_test"), { joint(example_slave_config()), second }));
}

TEST(EthercatSoemHalSetup, ChecksTheSharedParameters)
{
  EthercatSoemHal invalid_joint;
  EXPECT_FALSE(invalid_joint.setup(description("eth_test", { { "mode_of_operation", "4" } })));
  EthercatSoemHal invalid_bus;
  HalConfig config = description("eth_test");
  config.hardware_parameters["control_frequency"] = "0";
  EXPECT_FALSE(invalid_bus.setup(config));
  EthercatSoemHal invalid_timeout;
  config = description("eth_test");
  config.hardware_parameters["state_timeout_s"] = "never";
  EXPECT_FALSE(invalid_timeout.setup(config));
}

TEST(EthercatSoemHalSetup, TakesAWholeNumberOfAtLeastOneAsMaxBadCycles)
{
  for (const char* refused : { "0", "-3", "2.5", "many", "" })
  {
    EthercatSoemHal hal;
    HalConfig config = description("eth_test");
    config.hardware_parameters["max_bad_cycles"] = refused;
    EXPECT_FALSE(hal.setup(config)) << "max_bad_cycles '" << refused << "'";
  }
  EthercatSoemHal hal;
  HalConfig config = description("eth_test");
  config.hardware_parameters["max_bad_cycles"] = "1";
  EXPECT_TRUE(hal.setup(config));
}

// --- Bring-up without a drive --------------------------------------------------------------------

TEST(EthercatSoemHalBringUp, IsRefusedBeforeSetup)
{
  EthercatSoemHal hal;
  EXPECT_FALSE(hal.connect());
  EXPECT_FALSE(hal.start());
}

TEST(EthercatSoemHalBringUp, StartIsRefusedBeforeConnect)
{
  EthercatSoemHal hal;
  ASSERT_TRUE(hal.setup(bus("eth_test"), { joint(example_slave_config()) }));
  EXPECT_FALSE(hal.start());
  EXPECT_FALSE(hal.is_streaming());
}

TEST(EthercatSoemHalBringUp, StepsDownFromRungsItIsNotOn)
{
  EthercatSoemHal hal;
  EXPECT_TRUE(hal.stop());
  EXPECT_TRUE(hal.disable());
  EXPECT_TRUE(hal.disconnect());
  ASSERT_TRUE(hal.setup(bus("eth_test"), { joint(example_slave_config()) }));
  EXPECT_TRUE(hal.stop());
  EXPECT_TRUE(hal.disable());
  EXPECT_TRUE(hal.disconnect());
  EXPECT_FALSE(hal.is_connected());
  EXPECT_FALSE(hal.is_streaming());
  EXPECT_FALSE(hal.is_enabled());
}

TEST(EthercatSoemHalBringUp, FailsOnAMissingSlaveConfig)
{
  EthercatSoemHal hal;
  ASSERT_TRUE(hal.setup(bus("eth_test"), { joint("/nonexistent/slave_config.yaml") }));
  EXPECT_FALSE(hal.connect());
  EXPECT_FALSE(hal.is_connected());
}

/// Without the interface, or without the capability for a raw socket, bring-up fails cleanly and can be retried.
TEST(EthercatSoemHalBringUp, FailsCleanlyWhenTheNetworkInterfaceCannotBeOpened)
{
  EthercatSoemHal hal;
  ASSERT_TRUE(hal.setup(bus("no_such_nic0"), { joint(example_slave_config()) }));
  EXPECT_FALSE(hal.connect());
  EXPECT_FALSE(hal.is_connected());
  EXPECT_FALSE(hal.start());
  EXPECT_FALSE(hal.connect());
  EXPECT_TRUE(hal.disconnect());
}

/// On an interface without drives, bring-up fails: without CAP_NET_RAW at the socket, with it because no slave answers.
TEST(EthercatSoemHalBringUp, FailsOnAnInterfaceWithoutDrives)
{
  EthercatSoemHal hal;
  ASSERT_TRUE(hal.setup(bus("lo"), { joint(example_slave_config()) }));
  EXPECT_FALSE(hal.connect());
  EXPECT_FALSE(hal.is_connected());
}

TEST(EthercatSoemHalBringUp, ReadingStatesIsRefusedBeforeStart)
{
  EthercatSoemHal hal;
  std::vector<synapticon_hardware::DriveState> states;
  EXPECT_FALSE(hal.read(states));
}

TEST(EthercatSoemHalBringUp, ReportsNoHealthWithoutABus)
{
  EthercatSoemHal hal;
  ASSERT_TRUE(hal.setup(bus("eth_test"), { joint(example_slave_config()) }));
  synapticon_hardware::LinkHealth health;
  hal.link_health(health);
  ASSERT_EQ(health.drives.size(), 1u);
  EXPECT_FALSE(health.drives[0].online);
  EXPECT_FALSE(health.bus.link_up);
}

}  // namespace
