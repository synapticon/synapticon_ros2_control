// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The SOMANET drive firmware version, as reported in the manufacturer software version
/// object (0x100A), and the minimum version this package supports.

#include <cstdio>
#include <optional>
#include <string>
#include <tuple>

namespace synapticon_hardware::ethercat
{

/// @brief A firmware version, major.minor.patch.
struct FirmwareVersion
{
  int major{ 0 };
  int minor{ 0 };
  int patch{ 0 };

  bool operator<(const FirmwareVersion& other) const
  {
    return std::tie(major, minor, patch) < std::tie(other.major, other.minor, other.patch);
  }
};

/// The oldest firmware this package supports. It covers the impedance gains, the sign of the joint
/// torque offset and the quick stop behaviour the package relies on.
constexpr FirmwareVersion MINIMUM_FIRMWARE{ 5, 6, 11 };

/// @brief Parse a version string such as "v5.6.6" or "v5.6.8+g77". Leading whitespace and the "v"
/// are optional, anything after the patch number is ignored. Nothing if there is no x.y.z.
inline std::optional<FirmwareVersion> parse_firmware_version(const std::string& text)
{
  FirmwareVersion version;
  if (std::sscanf(text.c_str(), " v%d.%d.%d", &version.major, &version.minor, &version.patch) == 3 ||
      std::sscanf(text.c_str(), " %d.%d.%d", &version.major, &version.minor, &version.patch) == 3)
  {
    return version;
  }
  return std::nullopt;
}

}  // namespace synapticon_hardware::ethercat
