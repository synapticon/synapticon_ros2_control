// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The link health of each joint's drive, from the slave states the IgH master lists.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "ethercat_interface/ec_master.hpp"
#include "synapticon_hardware/ethercat_health.hpp"
#include "synapticon_ethercat_icube/ethercat_icube_hal.hpp"

namespace synapticon_ethercat_icube
{

/// @brief Fill `drives` (one entry per joint, already sized) with the state of each joint's slave among `slaves`, and
/// return the AL states of those slaves, ORed. A slave without a joint is never configured, so it stays in PRE-OP and
/// is left out. A joint whose slave is not listed keeps its entry and adds nothing.
inline uint8_t fill_joint_slave_health(const std::vector<ethercat_interface::EcSlaveStateInfo>& slaves,
                                       const std::vector<EthercatJointConfig>& joints,
                                       std::vector<EthercatSlaveHealth>& drives)
{
  uint8_t al_states = 0;
  for (const auto& slave : slaves)
  {
    for (std::size_t j = 0; j < joints.size() && j < drives.size(); ++j)
    {
      if (joints[j].position == slave.position && joints[j].alias == slave.alias)
      {
        drives[j] = EthercatSlaveHealth{ slave.al_state, slave.online };
        al_states |= slave.al_state;
      }
    }
  }
  return al_states;
}

}  // namespace synapticon_ethercat_icube
