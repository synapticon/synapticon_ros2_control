// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Input of SynapticonHalI::setup(), and helpers to read values from it.
///
/// The parameters of the robot description arrive here as plain text. Each HAL converts the values it uses, which
/// keeps the HALs free of ros2_control types.

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
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

/// @brief Parse parameter `name`, a whole decimal number with an optional minus sign.
///
/// Any other text, such as "0x03", "3x", "1.5" or a number out of range, throws std::invalid_argument naming the
/// parameter.
inline long long read_whole_number(const std::string& name, const std::string& text)
{
  const std::size_t first_digit = !text.empty() && text.front() == '-' ? 1 : 0;
  const bool digits_only = text.size() > first_digit && std::all_of(text.begin() + first_digit, text.end(), [](char c) {
                             return std::isdigit(static_cast<unsigned char>(c)) != 0;
                           });
  if (digits_only)
  {
    try
    {
      return std::stoll(text);
    }
    catch (const std::out_of_range&)
    {
    }
  }
  throw std::invalid_argument(name + " must be a whole decimal number, got '" + text + "'");
}

/// @brief Parse parameter `name`, a finite value of 0 or more in `unit`. An empty text gives NaN, "not set".
///
/// Anything else, also a number followed by other text, throws std::invalid_argument naming the parameter.
inline double read_non_negative(const std::string& name, const std::string& unit, const std::string& text)
{
  if (text.empty())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double value = std::numeric_limits<double>::quiet_NaN();
  try
  {
    std::size_t used = 0;
    value = std::stod(text, &used);
    value = used == text.size() ? value : std::numeric_limits<double>::quiet_NaN();
  }
  catch (const std::exception&)
  {
  }
  if (!std::isfinite(value) || value < 0.0)
  {
    throw std::invalid_argument(name + " must be a finite number of " + unit + ", 0 or more, got '" + text + "'");
  }
  return value;
}

}  // namespace synapticon_hardware
