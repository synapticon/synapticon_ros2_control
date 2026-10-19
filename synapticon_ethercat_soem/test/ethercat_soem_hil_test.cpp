// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Hardware-in-the-loop run of the drive action suite and the EtherCAT bring-up tests on EthercatSoemHal.
///
/// NOT part of `colcon test`: it needs one SOMANET drive connected, and it ENERGISES AND MOVES
/// the motor. Run it on purpose, with the joint free to turn about one output revolution, and with no other
/// EtherCAT master on the network interface:
///
///   net_raw_shell.sh --interface <if> ros2 run synapticon_ethercat_soem ethercat_soem_hil_test [--two-drives]
///
/// Besides the variables of the shared EtherCAT HIL tests, it reads:
///   SYNAPTICON_HIL_NETWORK_INTERFACE  the network interface of the drives (required, net_raw_shell.sh --interface
///                                     sets it)

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

/// The bench's bus on the network interface `SYNAPTICON_HIL_NETWORK_INTERFACE`.
bool set_up_hal(EthercatHalUnderTest& hal, const ethercat::BusConfig& bus,
                const std::vector<ethercat::JointConfig>& joints)
{
  synapticon_ethercat_soem::SoemConfig config;
  static_cast<ethercat::BusConfig&>(config) = bus;
  config.network_interface = env_or("SYNAPTICON_HIL_NETWORK_INTERFACE", "");
  return hal.setup(config, joints);
}
}  // namespace synapticon_hardware::testing

#include "synapticon_hardware/testing/ethercat_hal_tests.hpp"

int main(int argc, char** argv)
{
  if (std::getenv("SYNAPTICON_HIL_NETWORK_INTERFACE") == nullptr)
  {
    std::cerr << "Set SYNAPTICON_HIL_NETWORK_INTERFACE to the network interface of the drives.\n";
    return 1;
  }
  return synapticon_hardware::testing::run_ethercat_hil_tests(argc, argv);
}
