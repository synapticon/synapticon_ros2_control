// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Parsing the drive firmware version and comparing it with the minimum supported one.

#include <gtest/gtest.h>

#include "synapticon_hardware/ethercat/firmware_version.hpp"

namespace
{
using synapticon_hardware::ethercat::FirmwareVersion;
using synapticon_hardware::ethercat::MINIMUM_FIRMWARE;
using synapticon_hardware::ethercat::parse_firmware_version;

/// The space padded string of 0x100A parses into major, minor and patch.
TEST(FirmwareVersion, ParsesTheStringTheDriveReports)
{
  const auto version = parse_firmware_version("v5.6.6                                          ");
  ASSERT_TRUE(version.has_value());
  EXPECT_EQ(version->major, 5);
  EXPECT_EQ(version->minor, 6);
  EXPECT_EQ(version->patch, 6);
}

/// A build suffix is ignored and the leading v is optional.
TEST(FirmwareVersion, IgnoresABuildSuffixAndAMissingV)
{
  const auto with_suffix = parse_firmware_version("v5.6.8+g77");
  ASSERT_TRUE(with_suffix.has_value());
  EXPECT_EQ(with_suffix->patch, 8);
  const auto without_v = parse_firmware_version("5.7.0");
  ASSERT_TRUE(without_v.has_value());
  EXPECT_EQ(without_v->minor, 7);
}

/// An empty, non-numeric or incomplete string parses to nothing.
TEST(FirmwareVersion, RejectsWhatIsNotAVersion)
{
  EXPECT_FALSE(parse_firmware_version("").has_value());
  EXPECT_FALSE(parse_firmware_version("unknown").has_value());
  EXPECT_FALSE(parse_firmware_version("v5.6").has_value());
}

/// Versions compare major first, then minor, then patch.
TEST(FirmwareVersion, ComparesPartByPart)
{
  EXPECT_TRUE((FirmwareVersion{ 5, 6, 10 }) < MINIMUM_FIRMWARE);
  EXPECT_FALSE((FirmwareVersion{ 5, 6, 11 }) < MINIMUM_FIRMWARE);
  EXPECT_FALSE(MINIMUM_FIRMWARE < (FirmwareVersion{ 5, 6, 11 }));
  EXPECT_FALSE((FirmwareVersion{ 5, 7, 0 }) < MINIMUM_FIRMWARE);
  EXPECT_FALSE((FirmwareVersion{ 6, 0, 0 }) < MINIMUM_FIRMWARE);
  EXPECT_TRUE((FirmwareVersion{ 5, 6, 7 }) < MINIMUM_FIRMWARE);
  EXPECT_TRUE((FirmwareVersion{ 4, 9, 9 }) < MINIMUM_FIRMWARE);
}

}  // namespace

/// The minimum is 5.6.11: a build suffix does not matter, and what is not a version counts as below it.
TEST(FirmwareVersion, TheMinimumIsFiveSixEleven)
{
  const auto with_suffix = parse_firmware_version("v5.6.11+g77");
  ASSERT_TRUE(with_suffix.has_value());
  EXPECT_FALSE(*with_suffix < MINIMUM_FIRMWARE);
  const auto older = parse_firmware_version("v5.6.10");
  ASSERT_TRUE(older.has_value());
  EXPECT_TRUE(*older < MINIMUM_FIRMWARE);
  EXPECT_FALSE(parse_firmware_version("unknown").has_value());
}
