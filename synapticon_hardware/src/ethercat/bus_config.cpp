// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Reading the bus-wide settings of an EtherCAT HAL.

#include "synapticon_hardware/ethercat/bus_config.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace synapticon_hardware::ethercat
{

namespace
{
/// The whole number in `text`, when it is at least 1.
int at_least_one(const std::string& text)
{
  std::size_t used = 0;
  const int number = std::stoi(text, &used);
  if (used == text.size() && number >= 1)
  {
    return number;
  }
  throw std::invalid_argument(text + " is not a whole number of at least 1");
}
}  // namespace

double positive(const std::string& text)
{
  const double number = std::stod(text);
  if (std::isfinite(number) && number > 0.0)
  {
    return number;
  }
  throw std::invalid_argument(text + " is not a finite number greater than 0");
}

BusConfig bus_from(const ParameterMap& hardware)
{
  BusConfig bus;
  if (const auto it = hardware.find("control_frequency"); it != hardware.end())
  {
    bus.control_frequency = positive(it->second);
  }
  if (const auto it = hardware.find("readiness_timeout_s"); it != hardware.end())
  {
    bus.readiness_timeout_s = positive(it->second);
  }
  if (const auto it = hardware.find("quick_stop_hold"); it != hardware.end())
  {
    bus.quick_stop_hold = read_flag(it->second);
  }
  if (const auto it = hardware.find("sdo_read_attempts"); it != hardware.end())
  {
    bus.sdo_read_attempts = at_least_one(it->second);
  }
  if (const auto it = hardware.find("sdo_retry_wait_s"); it != hardware.end())
  {
    bus.sdo_retry_wait_s = positive(it->second);
  }
  return bus;
}

}  // namespace synapticon_hardware::ethercat
