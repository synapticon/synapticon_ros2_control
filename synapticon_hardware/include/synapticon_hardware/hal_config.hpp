// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Input of SynapticonHalI::setup(), and two helpers to read values from it.
///
/// The parameters of the robot description arrive here as plain text. Each HAL converts the values it uses, which
/// keeps the HALs free of ros2_control types.

#include <algorithm>
#include <array>
#include <cctype>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace synapticon_hardware
{

/// Parameters by name, with their values as text.
using ParameterMap = std::unordered_map<std::string, std::string>;

/// @brief A joint of the description and its own parameters.
///
/// The two parts of `hardware_interface::ComponentInfo` that a HAL needs, as plain text.
struct JointSpec
{
  std::string name;
  ParameterMap parameters;
};

/// @brief The argument of SynapticonHalI::setup().
///
/// The parameters and joints of `hardware_interface::HardwareInfo`, the robot description's `<ros2_control>` tag.
/// The order of `joints` is the joint order of every later HAL call.
struct HalConfig
{
  ParameterMap hardware_parameters;
  std::vector<JointSpec> joints;
};

/// @brief Look `name` up in `map`.
///
/// `fallback` is used only when the name is missing. A name given with an empty value returns that empty value.
inline std::string param_or(const ParameterMap& map, const std::string& name, const std::string& fallback = {})
{
  if (const auto entry = map.find(name); entry != map.end())
  {
    return entry->second;
  }
  return fallback;
}

/// @brief Parse a yes or no setting: true or 1 for yes, false or 0 for no. Letter case does not matter.
///
/// Other text throws std::invalid_argument, like std::stod does for text that is no number.
inline bool read_flag(std::string_view text)
{
  const auto spelled = [text](std::string_view word) {
    return std::ranges::equal(text, word, [](char a, char b) {
      return std::tolower(static_cast<unsigned char>(a)) == std::tolower(static_cast<unsigned char>(b));
    });
  };
  constexpr std::array<std::string_view, 2> set_words{ "true", "1" };
  constexpr std::array<std::string_view, 2> clear_words{ "false", "0" };
  if (std::ranges::any_of(set_words, spelled))
  {
    return true;
  }
  if (std::ranges::any_of(clear_words, spelled))
  {
    return false;
  }
  throw std::invalid_argument("expected true, false, 1 or 0, got '" + std::string(text) + "'");
}

}  // namespace synapticon_hardware
