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
  /// How long [s] start() gives the drives, once the bus is in OP, to send a statusword other than 0.
  double readiness_timeout_s{ 20.0 };
  /// Keep an impedance joint's target where its quick stop started, for as long as the drive is in Quick Stop
  /// Active. The drive does not brake by itself in that mode. False leaves the drive alone, which helps diagnose it.
  bool quick_stop_hold{ true };
};

/// @brief Read `control_frequency`, `readiness_timeout_s` and `quick_stop_hold` from `hardware`. Absent ones keep
/// the defaults of BusConfig.
/// @throws std::invalid_argument for a number that is not above 0, or a flag that is not one.
BusConfig bus_from(const ParameterMap& hardware);

/// @brief The number in `text`, when it is finite and greater than 0.
/// @throws std::invalid_argument otherwise.
double positive(const std::string& text);

}  // namespace synapticon_hardware::ethercat
