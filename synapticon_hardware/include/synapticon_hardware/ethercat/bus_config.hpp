// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Settings of an EtherCAT HAL that hold for the whole bus, whichever master runs it.

#include <string>

#include "synapticon_hardware/hal_config.hpp"

namespace synapticon_hardware::ethercat
{

/// @brief Bus-wide settings, read from the hardware parameters of the same names.
struct BusConfig
{
  /// Frames per second [Hz]. Also the rate of the distributed clock.
  double control_frequency{ 500.0 };
  /// How long [s] start() gives the drives, once the bus is in OP, to send a statusword other than 0 and to run the
  /// mode of operation of their joint.
  double readiness_timeout_s{ 20.0 };
  /// Keep an impedance joint's target where its quick stop started, for as long as the drive is in Quick Stop
  /// Active. The drive does not brake by itself in that mode. False leaves the drive alone, which helps diagnose it.
  bool quick_stop_hold{ true };
  /// How often bring-up tries an SDO read in all when the master gets no valid answer, and the wait [s] between the
  /// tries. The IgH master answers no read of a drive while it reads that drive's object dictionary after a rescan.
  int sdo_read_attempts{ 5 };
  double sdo_retry_wait_s{ 1.0 };
};

/// @brief Read `control_frequency`, `readiness_timeout_s`, `quick_stop_hold`, `sdo_read_attempts` and
/// `sdo_retry_wait_s` from `hardware`. Absent ones keep the defaults of BusConfig.
/// @throws std::invalid_argument for a number that is not above 0, a count below 1, or a flag that is not one.
BusConfig bus_from(const ParameterMap& hardware);

/// @brief The number in `text`, when it is finite and greater than 0.
/// @throws std::invalid_argument otherwise.
double positive(const std::string& text);

}  // namespace synapticon_hardware::ethercat
