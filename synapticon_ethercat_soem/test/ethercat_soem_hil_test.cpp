// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Hardware-in-the-loop run of the drive action suite and the EtherCAT bring-up tests on EthercatSoemHal, plus
/// the bring-up tests that only this HAL passes.
///
/// NOT part of `colcon test`: it needs one SOMANET drive connected, and it ENERGISES AND MOVES
/// the motor. Run it on purpose, with the joint free to turn about one output revolution, and with no other
/// EtherCAT master on the network interface:
///
///   net_raw_shell.sh --interface <if> ros2 run synapticon_ethercat_soem ethercat_soem_hil_test [--two-drives]
///
/// Besides the variables of the shared EtherCAT HIL tests, it reads:
///   SYNAPTICON_HIL_NETWORK_INTERFACE  the network interface of the drives (net_raw_shell.sh --interface sets it),
///                                     else network_interface of the robot definition's bus section

#include <cstdlib>
#include <iostream>
#include <vector>

#include "synapticon_ethercat_soem/ethercat_soem_hal.hpp"
#include "synapticon_hardware/ethercat/bus_config.hpp"
#include "synapticon_hardware/ethercat/joint_config.hpp"
#include "synapticon_hardware/testing/hal_bench.hpp"

namespace synapticon_hardware::testing
{
using EthercatHalUnderTest = synapticon_ethercat_soem::EthercatSoemHal;
}  // namespace synapticon_hardware::testing

#include "synapticon_hardware/testing/ethercat_hal_tests.hpp"

namespace synapticon_hardware::testing
{
/// The network interface of the drives: `SYNAPTICON_HIL_NETWORK_INTERFACE`, else the robot definition's.
std::string network_interface()
{
  return env_or("SYNAPTICON_HIL_NETWORK_INTERFACE", robot_bus_parameter("network_interface", ""));
}

/// The bench's bus on network_interface().
bool set_up_hal(EthercatHalUnderTest& hal, const ethercat::BusConfig& bus,
                const std::vector<ethercat::JointConfig>& joints)
{
  synapticon_ethercat_soem::SoemConfig config;
  static_cast<ethercat::BusConfig&>(config) = bus;
  config.network_interface = network_interface();
  return hal.setup(config, joints);
}

/// A startup SDO that the drive refuses fails the bring-up. 0x1000 (device type) is read-only, so the write never
/// takes and changes nothing.
TEST_F(EthercatBringUp, ARefusedStartupSdoFailsTheBringUp)
{
  JointConfig joint = bench_joint();
  joint.slave_config_file = write_config({ { "0x605a", 0, "int16", "6" }, { "0x1000", 0, "uint32", "0" } }, test_name(),
                                         joint.slave_config_file);
  auto hal = std::make_shared<ProbedHal>();
  ASSERT_TRUE(set_up_hal(*hal, bench_bus(), { joint }));
  EXPECT_FALSE(hal->connect());
  release(hal);
}
}  // namespace synapticon_hardware::testing

int main(int argc, char** argv)
{
  if (synapticon_hardware::testing::network_interface().empty())
  {
    std::cerr << "Set SYNAPTICON_HIL_NETWORK_INTERFACE, or network_interface in the bus section of the robot "
                 "definition, to the network interface of the drives.\n";
    return 1;
  }
  return synapticon_hardware::testing::run_ethercat_hil_tests(argc, argv);
}
