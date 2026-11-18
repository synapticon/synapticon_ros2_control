// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Health of the link below the drives, as a HAL reports it to the hardware interface.

#include <cstddef>
#include <vector>

#include "synapticon_hardware/ethercat_health.hpp"

namespace synapticon_hardware
{

/// @brief The link layer health: one entry per drive, and one for the whole link.
///
/// The values are those of the EtherCAT health state interfaces. A HAL without a link layer to
/// report, such as the mock, reports everything healthy.
struct LinkHealth
{
  std::vector<EthercatSlaveHealth> drives;  ///< In drive order.
  EthercatBusHealth bus;
};

/// Fill `health` as everything healthy: `drive_count` drives online in OP, the link up, the
/// domain complete. Allocates nothing once `health.drives` has the size.
inline void fill_healthy_link(std::size_t drive_count, LinkHealth& health)
{
  health.drives.assign(drive_count, EthercatSlaveHealth{ al_state::OP, true });
  health.bus = EthercatBusHealth{};
  health.bus.link_up = true;
  health.bus.slaves_responding = static_cast<uint32_t>(drive_count);
  health.bus.al_states = al_state::OP;
  health.bus.domain_wc_state = wc_state::COMPLETE;
}

/// Everything healthy, see fill_healthy_link().
inline LinkHealth healthy_link(std::size_t drive_count)
{
  LinkHealth health;
  fill_healthy_link(drive_count, health);
  return health;
}

}  // namespace synapticon_hardware
