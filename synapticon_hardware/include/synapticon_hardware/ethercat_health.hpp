// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief EtherCAT status of the bus and of each drive, as the master knows it besides the process data.
///
/// Each field is published as the ros2_control state interface of the same name. A HAL without EtherCAT reports
/// a healthy bus, so these interfaces exist with every HAL.
///
/// The fields are those the IgH master reports in `ecrt.h`, the C API of its user space library, so they carry the
/// master's own meaning. The SOEM HAL fills them with the same meaning.

#include <cstdint>

namespace synapticon_hardware
{

/// The states of the EtherCAT state machine as bit flags, so that the states of several slaves can be ORed.
namespace al_state
{
inline constexpr uint8_t INIT = 0x01;
inline constexpr uint8_t PREOP = 0x02;
inline constexpr uint8_t SAFEOP = 0x04;
inline constexpr uint8_t OP = 0x08;
}  // namespace al_state

/// How many slaves answered the last process data frame, judged by its working counter. The values of IgH's
/// `ec_wc_state_t`.
namespace wc_state
{
inline constexpr uint8_t NONE = 0;      ///< none answered
inline constexpr uint8_t PARTIAL = 1;   ///< at least one is missing
inline constexpr uint8_t COMPLETE = 2;  ///< all answered
}  // namespace wc_state

/// @brief Status of one drive on the bus: `al_state` and `online` of IgH's `ec_slave_config_state_t`.
struct EthercatSlaveHealth
{
  uint8_t al_state{ 0 };  ///< Its state as one `al_state` flag. 0 before the first read.
  bool online{ false };   ///< Whether it answers the master.
};

/// @brief Status of the whole bus: `link_up`, `slaves_responding` and `al_states` of IgH's `ec_master_state_t`,
/// and the working counter and its state of `ec_domain_state_t`.
struct EthercatBusHealth
{
  bool link_up{ false };                 ///< Whether the network port has a carrier.
  uint32_t slaves_responding{ 0 };       ///< How many slaves the master finds.
  uint8_t al_states{ 0 };                ///< `al_state` flags of every slave the HAL runs, ORed.
  uint32_t domain_working_counter{ 0 };  ///< Working counter of the last frame.
  uint8_t domain_wc_state{ 0 };          ///< What that counter means, a `wc_state` value.
};

/// @brief True when `drive` answers and is in OP. Only then is its process data valid.
inline bool is_operational(const EthercatSlaveHealth& drive)
{
  return drive.online && drive.al_state == al_state::OP;
}

}  // namespace synapticon_hardware
