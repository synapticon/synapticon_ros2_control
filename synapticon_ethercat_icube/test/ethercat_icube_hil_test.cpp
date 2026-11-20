// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Hardware-in-the-loop run of the drive action suite and the EtherCAT bring-up tests on EthercatIcubeHal.
///
/// NOT part of `colcon test`: it needs one SOMANET drive connected, and it ENERGISES AND MOVES
/// the motor. Run it on purpose, with the joint free to turn about one output revolution:
///
///   ros2 run synapticon_ethercat_icube ethercat_icube_hil_test [--two-drives]
///
/// Besides the variables of the shared EtherCAT HIL tests, it reads (optional):
///   SYNAPTICON_HIL_MASTER        IgH master index (default: the robot definition's master_id, else 0)

#include <string>
#include <vector>

#include "synapticon_ethercat_icube/ethercat_icube_hal.hpp"
#include "synapticon_hardware/ethercat/bus_config.hpp"
#include "synapticon_hardware/ethercat/joint_config.hpp"
#include "synapticon_hardware/testing/hal_bench.hpp"

namespace synapticon_hardware::testing
{
using EthercatHalUnderTest = synapticon_ethercat_icube::EthercatIcubeHal;
}  // namespace synapticon_hardware::testing

#include "synapticon_hardware/testing/ethercat_hal_tests.hpp"

namespace synapticon_hardware::testing
{
/// The bench's bus on the IgH master `SYNAPTICON_HIL_MASTER`, else the robot definition's `master_id`, else 0.
bool set_up_hal(EthercatHalUnderTest& hal, const ethercat::BusConfig& bus,
                const std::vector<ethercat::JointConfig>& joints)
{
  synapticon_ethercat_icube::EthercatConfig config;
  static_cast<ethercat::BusConfig&>(config) = bus;
  config.master_id =
      static_cast<unsigned>(std::stoul(env_or("SYNAPTICON_HIL_MASTER", robot_bus_parameter("master_id", "0"))));
  std::vector<synapticon_ethercat_icube::EthercatJointConfig> icube_joints;
  for (const auto& joint : joints)
  {
    icube_joints.push_back(synapticon_ethercat_icube::EthercatJointConfig{ joint });
  }
  return hal.setup(config, icube_joints);
}
}  // namespace synapticon_hardware::testing

int main(int argc, char** argv)
{
  return synapticon_hardware::testing::run_ethercat_hil_tests(argc, argv);
}
