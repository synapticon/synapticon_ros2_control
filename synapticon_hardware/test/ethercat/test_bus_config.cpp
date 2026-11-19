// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Parsing the bus-wide settings of an EtherCAT HAL.

#include <stdexcept>
#include <string>
#include <unordered_map>

#include <gtest/gtest.h>

#include "synapticon_hardware/ethercat/bus_config.hpp"

namespace
{
using synapticon_hardware::ethercat::bus_from;
using Parameters = std::unordered_map<std::string, std::string>;

TEST(BusConfig, EverySettingHasADefault)
{
  const auto bus = bus_from({});
  EXPECT_DOUBLE_EQ(bus.control_frequency, 500.0);
  EXPECT_DOUBLE_EQ(bus.readiness_timeout_s, 20.0);
  EXPECT_TRUE(bus.quick_stop_hold);
  EXPECT_EQ(bus.sdo_read_attempts, 5);
  EXPECT_DOUBLE_EQ(bus.sdo_retry_wait_s, 1.0);
  EXPECT_EQ(bus.max_bad_cycles, 10);
}

TEST(BusConfig, ReadsEverySetting)
{
  const auto bus = bus_from({ { "control_frequency", "250" },
                              { "readiness_timeout_s", "2.5" },
                              { "quick_stop_hold", "false" },
                              { "sdo_read_attempts", "3" },
                              { "sdo_retry_wait_s", "0.2" },
                              { "max_bad_cycles", "3" } });
  EXPECT_DOUBLE_EQ(bus.control_frequency, 250.0);
  EXPECT_DOUBLE_EQ(bus.readiness_timeout_s, 2.5);
  EXPECT_FALSE(bus.quick_stop_hold);
  EXPECT_EQ(bus.sdo_read_attempts, 3);
  EXPECT_DOUBLE_EQ(bus.sdo_retry_wait_s, 0.2);
  EXPECT_EQ(bus.max_bad_cycles, 3);
}

class InvalidBus : public ::testing::TestWithParam<Parameters>
{
};

TEST_P(InvalidBus, IsRefused)
{
  EXPECT_THROW(bus_from(GetParam()), std::invalid_argument);
}

INSTANTIATE_TEST_SUITE_P(
    Cases, InvalidBus,
    ::testing::Values(Parameters{ { "control_frequency", "0" } }, Parameters{ { "control_frequency", "fast" } },
                      Parameters{ { "control_frequency", "1000Hz" } }, Parameters{ { "readiness_timeout_s", "-1" } },
                      Parameters{ { "readiness_timeout_s", "nan" } }, Parameters{ { "quick_stop_hold", "sometimes" } },
                      Parameters{ { "sdo_read_attempts", "0" } }, Parameters{ { "sdo_read_attempts", "2.5" } },
                      Parameters{ { "sdo_read_attempts", "often" } }, Parameters{ { "sdo_retry_wait_s", "0" } },
                      Parameters{ { "max_bad_cycles", "0" } }, Parameters{ { "max_bad_cycles", "1.5" } }));

}  // namespace
