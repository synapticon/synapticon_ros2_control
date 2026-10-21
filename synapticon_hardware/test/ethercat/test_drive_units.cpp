// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The drive's units: decoding the velocity unit, the rules of a slave_config's unit sections, and the
/// conversion of SI values into raw ones.

#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "synapticon_hardware/ethercat/drive_units.hpp"

namespace
{
using namespace synapticon_hardware::ethercat;

constexpr double RAD_PER_RPM = 2.0 * M_PI / 60.0;

TEST(VelocityStep, DecodesPowerOfTenAndTimeUnit)
{
  EXPECT_DOUBLE_EQ(*velocity_step(0x00B44700), RAD_PER_RPM);         // rev/min
  EXPECT_DOUBLE_EQ(*velocity_step(0xFDB44700), 1e-3 * RAD_PER_RPM);  // 10^-3 rev/min
  EXPECT_DOUBLE_EQ(*velocity_step(0x03B44700), 1e3 * RAD_PER_RPM);   // 10^3 rev/min
  EXPECT_DOUBLE_EQ(*velocity_step(0x00B40300), 2.0 * M_PI);          // rev/s
  EXPECT_DOUBLE_EQ(*velocity_step(0xFFB40300), 0.1 * 2.0 * M_PI);    // 10^-1 rev/s
}

TEST(VelocityStep, UnderstandsNothingElse)
{
  for (const uint32_t unit : { 0x00B44701u, 0x00B54700u, 0x00B40400u, 0xFCB44700u, 0x0DB44700u, 0u })
  {
    EXPECT_FALSE(velocity_step(unit)) << std::hex << unit;
  }
}

TEST(UnitTables, SortTheKnownObjects)
{
  EXPECT_EQ(unit_dependent_quantity({ 0x6085, 0 }), SiQuantity::ACCELERATION);
  EXPECT_EQ(unit_dependent_quantity({ 0x6080, 0 }), SiQuantity::VELOCITY);
  EXPECT_EQ(unit_dependent_quantity({ 0x6065, 0 }), SiQuantity::POSITION);
  EXPECT_EQ(unit_dependent_quantity({ 0x6072, 0 }), SiQuantity::TORQUE);
  EXPECT_FALSE(unit_dependent_quantity({ 0x605A, 0 }));
  EXPECT_TRUE(unit_free({ 0x605A, 0 }));
  EXPECT_TRUE(unit_free({ 0x2150, 1 }));
  EXPECT_FALSE(unit_free({ 0x6085, 0 }));
  EXPECT_TRUE(defines_units({ 0x60A9, 0 }));
  EXPECT_TRUE(defines_units({ 0x6092, 1 }));
  EXPECT_FALSE(defines_units({ 0x6085, 0 }));
}

/// A slave_config with `content` after a minimal head, written to a temporary file. Returns its path.
std::string slave_config(const std::string& content)
{
  static int count = 0;
  const std::string path = ::testing::TempDir() + "drive_units_" + std::to_string(count++) + ".yaml";
  std::ofstream(path) << "vendor_id: 0x22d2\nproduct_id: 0x302\n" << content;
  return path;
}

TEST(ReadSlaveConfigUnits, ReadsSiEntriesAndOverrides)
{
  const auto units =
      read_slave_config_units(slave_config("sdo:\n"
                                           "  - { index: 0x605a, sub_index: 0, type: int16, value: 6 }\n"
                                           "  - { index: 0x1234, sub_index: 1, type: uint8, value: 3, raw: true }\n"
                                           "  - { index: 0x6085, sub_index: 0, type: uint32, value: 5000, raw: true }\n"
                                           "sdo_si:\n"
                                           "  - { index: 0x6085, sub_index: 0, type: uint32, si_value: 200.0 }\n"
                                           "units:\n"
                                           "  velocity_unit: 0x00B44700\n"
                                           "  counts_per_revolution: 262144\n"
                                           "  rated_torque: 3.2\n"
                                           "  feed_constant: disabled\n"));
  ASSERT_EQ(units.si_sdo.size(), 1u);
  EXPECT_EQ(units.si_sdo[0].object.index, 0x6085);
  EXPECT_EQ(units.si_sdo[0].quantity, SiQuantity::ACCELERATION);
  EXPECT_DOUBLE_EQ(units.si_sdo[0].value, 200.0);
  EXPECT_EQ(units.overrides.velocity_unit, 0x00B44700u);
  EXPECT_EQ(units.overrides.counts_per_revolution, 262144u);
  EXPECT_EQ(units.overrides.rated_torque, 3.2);
  EXPECT_TRUE(units.overrides.feed_constant_disabled);
}

TEST(ReadSlaveConfigUnits, AcceptsAConfigWithoutUnitSections)
{
  const auto units = read_slave_config_units(slave_config(""));
  EXPECT_TRUE(units.si_sdo.empty());
  EXPECT_FALSE(units.overrides.velocity_unit);
}

struct RefusedCase
{
  const char* name;
  const char* content;
};

class RefusedSlaveConfig : public ::testing::TestWithParam<RefusedCase>
{
};

TEST_P(RefusedSlaveConfig, NamesTheEntry)
{
  EXPECT_THROW(read_slave_config_units(slave_config(GetParam().content)), std::invalid_argument);
}

INSTANTIATE_TEST_SUITE_P(
    Cases, RefusedSlaveConfig,
    ::testing::Values(
        RefusedCase{ "RawValueOfAUnitDependentObject", "sdo:\n  - { index: 0x6085, sub_index: 0, type: uint32, "
                                                       "value: 5000 }\n" },
        RefusedCase{ "UnknownObjectWithoutRaw", "sdo:\n  - { index: 0x1234, sub_index: 1, type: uint8, value: 3 }\n" },
        RefusedCase{ "SdoChangingTheUnits", "sdo:\n  - { index: 0x60a9, sub_index: 0, type: uint32, value: 1, "
                                            "raw: true }\n" },
        RefusedCase{ "SiValueInSdo", "sdo:\n  - { index: 0x6085, sub_index: 0, type: uint32, si_value: 1.0 }\n" },
        RefusedCase{ "SdoSiChangingTheUnits", "sdo_si:\n  - { index: 0x6092, sub_index: 1, type: uint32, "
                                              "si_value: 1.0 }\n" },
        RefusedCase{ "SdoSiWithoutKnownUnit", "sdo_si:\n  - { index: 0x605a, sub_index: 0, type: int16, "
                                              "si_value: 6 }\n" },
        RefusedCase{ "SdoSiWithValue", "sdo_si:\n  - { index: 0x6085, sub_index: 0, type: uint32, value: 5 }\n" },
        RefusedCase{ "SdoSiNotInteger", "sdo_si:\n  - { index: 0x6085, sub_index: 0, type: real32, si_value: 1.0 }\n" },
        RefusedCase{ "UnknownVelocityUnit", "units:\n  velocity_unit: 0x00B44701\n" },
        RefusedCase{ "ZeroCounts", "units:\n  counts_per_revolution: 0\n" },
        RefusedCase{ "NegativeRatedTorque", "units:\n  rated_torque: -1.0\n" },
        RefusedCase{ "FeedConstantNotDisabled", "units:\n  feed_constant: enabled\n" },
        RefusedCase{ "UnknownUnitsKey", "units:\n  position_unit: 1\n" }),
    [](const auto& info) { return std::string(info.param.name); });

TEST(ReadSlaveConfigUnits, RefusesAMissingFile)
{
  EXPECT_THROW(read_slave_config_units(::testing::TempDir() + "no_such_slave_config.yaml"), std::invalid_argument);
}

/// Factors of a drive in rev/min with 4096 counts per turn and a rated torque of 1 Nm.
SiFactors factors()
{
  SiFactors f = UNREAD_SI_FACTORS;
  si_factor(f, Unit::ANGULAR_VELOCITY) = RAD_PER_RPM;
  si_factor(f, Unit::ANGLE) = 2.0 * M_PI / 4096;
  si_factor(f, Unit::MOTOR_TORQUE) = 1e-3;
  return f;
}

TEST(SiToRaw, RoundsToTheNearestUnit)
{
  std::string error;
  EXPECT_EQ(si_to_raw({ { 0x6080, 0 }, "uint32", 15.7, SiQuantity::VELOCITY }, factors(), error), 150);
  EXPECT_EQ(si_to_raw({ { 0x6085, 0 }, "uint32", 1000.0, SiQuantity::ACCELERATION }, factors(), error), 9549);
  EXPECT_EQ(si_to_raw({ { 0x607D, 1 }, "int32", -1.0, SiQuantity::POSITION }, factors(), error), -652);
  EXPECT_EQ(si_to_raw({ { 0x6072, 0 }, "uint16", 0.96, SiQuantity::TORQUE }, factors(), error), 960);
  EXPECT_EQ(si_to_raw({ { 0x6080, 0 }, "uint32", 0.0, SiQuantity::VELOCITY }, factors(), error), 0);
}

TEST(SiToRaw, UsesTheMagnitudeOfTheFactorsOfAnInvertedJoint)
{
  SiFactors f = factors();
  si_factor(f, Unit::ANGULAR_VELOCITY) = -RAD_PER_RPM;
  std::string error;
  EXPECT_EQ(si_to_raw({ { 0x6080, 0 }, "uint32", 15.7, SiQuantity::VELOCITY }, f, error), 150);
}

TEST(SiToRaw, RefusesWhatTheUnitCannotHold)
{
  std::string error;
  EXPECT_FALSE(si_to_raw({ { 0x6085, 0 }, "uint32", 0.01, SiQuantity::ACCELERATION }, factors(), error));
  EXPECT_NE(error.find("rounds to 0"), std::string::npos) << error;
  EXPECT_FALSE(si_to_raw({ { 0x6065, 0 }, "uint32", 0.006, SiQuantity::POSITION }, factors(), error));
  EXPECT_NE(error.find("1 %"), std::string::npos) << error;
  EXPECT_FALSE(si_to_raw({ { 0x6072, 0 }, "uint16", 100.0, SiQuantity::TORQUE }, factors(), error));
  EXPECT_FALSE(si_to_raw({ { 0x6072, 0 }, "int8", 0.2, SiQuantity::TORQUE }, factors(), error));
  EXPECT_FALSE(si_to_raw({ { 0x6080, 0 }, "uint32", -1.0, SiQuantity::VELOCITY }, factors(), error));
}

TEST(SiToRaw, RefusesAnUnknownUnit)
{
  SiFactors f = factors();
  si_factor(f, Unit::ANGULAR_VELOCITY) = std::nan("");
  std::string error;
  EXPECT_FALSE(si_to_raw({ { 0x6080, 0 }, "uint32", 15.7, SiQuantity::VELOCITY }, f, error));
}

TEST(RawBytes, AreLittleEndianOfTheTypeSize)
{
  EXPECT_EQ(raw_bytes(9549, "uint32"), (std::vector<uint8_t>{ 0x4D, 0x25, 0x00, 0x00 }));
  EXPECT_EQ(raw_bytes(-652, "int32"), (std::vector<uint8_t>{ 0x74, 0xFD, 0xFF, 0xFF }));
  EXPECT_EQ(raw_bytes(960, "uint16"), (std::vector<uint8_t>{ 0xC0, 0x03 }));
  EXPECT_THROW(raw_bytes(1, "real32"), std::invalid_argument);
}

}  // namespace
