// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The units a drive is commissioned in, and the slave_config entries whose meaning depends on them.

#include "synapticon_hardware/ethercat/drive_units.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace synapticon_hardware::ethercat
{

namespace
{
/// One table entry: an object and what its value is counted in.
struct QuantityOf
{
  CoeAddress object;
  SiQuantity quantity;
};

/// The objects whose unit the stack knows to depend on the drive's units: those of CiA 402 and the ones Synapticon's
/// documentation lists for the velocity unit (0x60A9) and the feed constant (0x6092).
constexpr QuantityOf UNIT_DEPENDENT[] = {
  // velocity
  { { 0x606B, 0 }, SiQuantity::VELOCITY },   // velocity demand value
  { { 0x606C, 0 }, SiQuantity::VELOCITY },   // velocity actual value
  { { 0x606D, 0 }, SiQuantity::VELOCITY },   // velocity window
  { { 0x606F, 0 }, SiQuantity::VELOCITY },   // velocity threshold
  { { 0x607F, 0 }, SiQuantity::VELOCITY },   // max profile velocity
  { { 0x6080, 0 }, SiQuantity::VELOCITY },   // max motor speed
  { { 0x6081, 0 }, SiQuantity::VELOCITY },   // profile velocity
  { { 0x6082, 0 }, SiQuantity::VELOCITY },   // end velocity
  { { 0x6099, 1 }, SiQuantity::VELOCITY },   // homing speed, search for switch
  { { 0x6099, 2 }, SiQuantity::VELOCITY },   // homing speed, search for zero
  { { 0x60B1, 0 }, SiQuantity::VELOCITY },   // velocity offset
  { { 0x60FF, 0 }, SiQuantity::VELOCITY },   // target velocity
  { { 0x2013, 2 }, SiQuantity::VELOCITY },   // gain scheduling, threshold velocity 0
  { { 0x2013, 11 }, SiQuantity::VELOCITY },  // gain scheduling, threshold velocity 1
  { { 0x230B, 0 }, SiQuantity::VELOCITY },   // secondary velocity value
  // acceleration
  { { 0x6083, 0 }, SiQuantity::ACCELERATION },  // profile acceleration
  { { 0x6084, 0 }, SiQuantity::ACCELERATION },  // profile deceleration
  { { 0x6085, 0 }, SiQuantity::ACCELERATION },  // quick stop deceleration
  { { 0x609A, 0 }, SiQuantity::ACCELERATION },  // homing acceleration
  { { 0x60C5, 0 }, SiQuantity::ACCELERATION },  // max acceleration
  { { 0x60C6, 0 }, SiQuantity::ACCELERATION },  // max deceleration
  // position
  { { 0x6062, 0 }, SiQuantity::POSITION },  // position demand value
  { { 0x6064, 0 }, SiQuantity::POSITION },  // position actual value
  { { 0x6065, 0 }, SiQuantity::POSITION },  // following error window
  { { 0x6067, 0 }, SiQuantity::POSITION },  // position window
  { { 0x607A, 0 }, SiQuantity::POSITION },  // target position
  { { 0x607C, 0 }, SiQuantity::POSITION },  // home offset
  { { 0x607D, 1 }, SiQuantity::POSITION },  // software position limit, minimum
  { { 0x607D, 2 }, SiQuantity::POSITION },  // software position limit, maximum
  { { 0x60F4, 0 }, SiQuantity::POSITION },  // following error actual value
  // torque, in thousandths of the rated torque
  { { 0x6071, 0 }, SiQuantity::TORQUE },  // target torque
  { { 0x6072, 0 }, SiQuantity::TORQUE },  // max torque
  { { 0x6074, 0 }, SiQuantity::TORQUE },  // torque demand
  { { 0x6077, 0 }, SiQuantity::TORQUE },  // torque actual value
  { { 0x60E0, 0 }, SiQuantity::TORQUE },  // positive torque limit
  { { 0x60E1, 0 }, SiQuantity::TORQUE },  // negative torque limit
};

/// Objects whose value has no unit or a fixed one.
constexpr CoeAddress UNIT_FREE[] = {
  { 0x605A, 0 },  // quick stop option code
  { 0x605B, 0 },  // shutdown option code
  { 0x605C, 0 },  // disable operation option code
  { 0x605D, 0 },  // halt option code
  { 0x605E, 0 },  // fault reaction option code
  { 0x6060, 0 },  // mode of operation
  { 0x6066, 0 },  // following error timeout, ms
  { 0x6068, 0 },  // position window time, ms
  { 0x2004, 4 },  // brake release strategy
  { 0x2004, 5 },  // controller disable delay, ms
  { 0x2017, 1 },  // following error reaction
  { 0x2054, 0 },  // max joint torque, mNm
  { 0x2060, 1 },  // impedance stiffness, mNm/rad
  { 0x2060, 2 },  // impedance damping
  { 0x2060, 3 },  // gain of the inner torque loop
  { 0x2150, 1 },  // torque sensor type
};

/// Objects that set the units. The stack reads them and never writes them.
constexpr CoeAddress DEFINES_UNITS[] = {
  { 0x6076, 0 },  // rated torque
  { 0x6091, 1 },  // gear ratio, motor revolutions
  { 0x6091, 2 },  // gear ratio, shaft revolutions
  { 0x6092, 1 },  // feed constant, feed
  { 0x6092, 2 },  // feed constant, shaft revolutions
  { 0x60A8, 0 },  // SI unit position
  { 0x60A9, 0 },  // SI unit velocity
  { 0x2012, 9 },  // encoder used for the position
  { 0x2110, 3 },  // encoder 1 resolution
  { 0x2112, 3 },  // encoder 2 resolution
};

bool same(CoeAddress a, CoeAddress b)
{
  return a.index == b.index && a.subindex == b.subindex;
}

/// The size in bytes and signedness of the integer `type`, or nothing for another type.
std::optional<std::pair<int, bool>> integer_type(const std::string& type)
{
  const std::pair<const char*, std::pair<int, bool>> types[] = {
    { "int8", { 1, true } },    { "uint8", { 1, false } }, { "int16", { 2, true } },
    { "uint16", { 2, false } }, { "int32", { 4, true } },  { "uint32", { 4, false } },
  };
  for (const auto& [name, layout] : types)
  {
    if (type == name)
    {
      return layout;
    }
  }
  return std::nullopt;
}

/// "0x6085:0", for messages.
std::string object_name(CoeAddress object)
{
  char text[16];
  std::snprintf(text, sizeof(text), "0x%04X:%u", object.index, object.subindex);
  return text;
}

/// The object an entry names, with `where` and the file in the message of a missing key.
CoeAddress entry_object(const YAML::Node& entry, const std::string& where)
{
  if (!entry["index"] || !entry["sub_index"])
  {
    throw std::invalid_argument(where + ": an entry needs index and sub_index");
  }
  return { entry["index"].as<uint16_t>(), static_cast<uint8_t>(entry["sub_index"].as<unsigned>()) };
}

/// Check one `sdo:` entry against the rules of read_slave_config_units().
void check_raw_entry(const YAML::Node& entry, const std::string& file)
{
  const CoeAddress object = entry_object(entry, file + ": sdo");
  const std::string where = file + ": sdo entry " + object_name(object);
  if (entry["si_value"])
  {
    throw std::invalid_argument(where + " gives si_value, which belongs in the sdo_si section");
  }
  if (defines_units(object))
  {
    throw std::invalid_argument(where + " would change the drive's units, which is commissioning, not bring-up");
  }
  if (entry["raw"] && entry["raw"].as<bool>())
  {
    return;
  }
  if (unit_dependent_quantity(object))
  {
    throw std::invalid_argument(where + " depends on the drive's units: give it in sdo_si with si_value, or mark it "
                                        "raw: true if the raw value is meant");
  }
  if (!unit_free(object))
  {
    throw std::invalid_argument(where + " is not one the stack knows: mark it raw: true if the value is meant as "
                                        "written");
  }
}

/// One `sdo_si:` entry, checked.
SiSdo si_entry(const YAML::Node& entry, const std::string& file)
{
  const CoeAddress object = entry_object(entry, file + ": sdo_si");
  const std::string where = file + ": sdo_si entry " + object_name(object);
  if (defines_units(object))
  {
    throw std::invalid_argument(where + " would change the drive's units, which is commissioning, not bring-up");
  }
  const auto quantity = unit_dependent_quantity(object);
  if (!quantity)
  {
    throw std::invalid_argument(where + " has no unit the stack knows, so it cannot be converted. Give it in sdo");
  }
  if (entry["value"] || !entry["si_value"])
  {
    throw std::invalid_argument(where + " needs si_value and no value");
  }
  const std::string type = entry["type"] ? entry["type"].as<std::string>() : "";
  const double value = entry["si_value"].as<double>();
  if (!integer_type(type) || !std::isfinite(value))
  {
    throw std::invalid_argument(where + " needs an integer type and a finite si_value");
  }
  return { object, type, value, *quantity };
}

/// The `units:` section, checked.
UnitOverrides overrides_from(const YAML::Node& units, const std::string& file)
{
  UnitOverrides overrides;
  for (const auto& item : units)
  {
    const std::string key = item.first.as<std::string>();
    const std::string where = file + ": units: " + key;
    if (key == "velocity_unit")
    {
      overrides.velocity_unit = item.second.as<uint32_t>();
      if (!velocity_step(*overrides.velocity_unit))
      {
        throw std::invalid_argument(where + " is not a velocity unit the stack understands");
      }
    }
    else if (key == "counts_per_revolution")
    {
      overrides.counts_per_revolution = item.second.as<uint32_t>();
      if (*overrides.counts_per_revolution == 0)
      {
        throw std::invalid_argument(where + " must be above 0");
      }
    }
    else if (key == "rated_torque")
    {
      overrides.rated_torque = item.second.as<double>();
      if (!std::isfinite(*overrides.rated_torque) || *overrides.rated_torque <= 0.0)
      {
        throw std::invalid_argument(where + " must be above 0 Nm");
      }
    }
    else if (key == "feed_constant")
    {
      if (item.second.as<std::string>() != "disabled")
      {
        throw std::invalid_argument(where + " can only be 'disabled': the stack does not support linear units");
      }
      overrides.feed_constant_disabled = true;
    }
    else
    {
      throw std::invalid_argument(where + " is not a known key");
    }
  }
  return overrides;
}

/// The unit factor `quantity` is counted in.
Unit factor_unit(SiQuantity quantity)
{
  switch (quantity)
  {
    case SiQuantity::VELOCITY:
    case SiQuantity::ACCELERATION:
      return Unit::ANGULAR_VELOCITY;
    case SiQuantity::POSITION:
      return Unit::ANGLE;
    case SiQuantity::TORQUE:
      return Unit::MOTOR_TORQUE;
  }
  return Unit::RAW;
}
}  // namespace

std::optional<double> velocity_step(uint32_t velocity_unit)
{
  const auto exponent = static_cast<int8_t>(velocity_unit >> 24);
  const uint8_t distance = (velocity_unit >> 16) & 0xFF;
  const uint8_t time = (velocity_unit >> 8) & 0xFF;
  const uint8_t reserved = velocity_unit & 0xFF;
  constexpr uint8_t REVOLUTIONS = 0xB4;
  constexpr uint8_t PER_MINUTE = 0x47;
  constexpr uint8_t PER_SECOND = 0x03;
  if (exponent < -3 || exponent > 12 || distance != REVOLUTIONS || reserved != 0 ||
      (time != PER_MINUTE && time != PER_SECOND))
  {
    return std::nullopt;
  }
  const double seconds = time == PER_MINUTE ? 60.0 : 1.0;
  return std::pow(10.0, exponent) * 2.0 * std::numbers::pi / seconds;
}

const char* si_unit_name(SiQuantity quantity)
{
  switch (quantity)
  {
    case SiQuantity::VELOCITY:
      return "rad/s";
    case SiQuantity::ACCELERATION:
      return "rad/s^2";
    case SiQuantity::POSITION:
      return "rad";
    case SiQuantity::TORQUE:
      return "Nm";
  }
  return "";
}

std::optional<SiQuantity> unit_dependent_quantity(CoeAddress object)
{
  for (const auto& entry : UNIT_DEPENDENT)
  {
    if (same(entry.object, object))
    {
      return entry.quantity;
    }
  }
  return std::nullopt;
}

bool unit_free(CoeAddress object)
{
  return std::any_of(std::begin(UNIT_FREE), std::end(UNIT_FREE), [object](CoeAddress o) { return same(o, object); });
}

bool defines_units(CoeAddress object)
{
  return std::any_of(std::begin(DEFINES_UNITS), std::end(DEFINES_UNITS),
                     [object](CoeAddress o) { return same(o, object); });
}

SlaveConfigUnits read_slave_config_units(const std::string& path)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(path);
  }
  catch (const YAML::Exception& e)
  {
    throw std::invalid_argument(path + ": " + e.what());
  }
  SlaveConfigUnits units;
  try
  {
    for (const auto& entry : root["sdo"])
    {
      check_raw_entry(entry, path);
    }
    for (const auto& entry : root["sdo_si"])
    {
      units.si_sdo.push_back(si_entry(entry, path));
    }
    if (root["units"])
    {
      units.overrides = overrides_from(root["units"], path);
    }
  }
  catch (const YAML::Exception& e)
  {
    throw std::invalid_argument(path + ": " + e.what());
  }
  return units;
}

std::optional<int64_t> si_to_raw(const SiSdo& entry, const SiFactors& factors, std::string& error)
{
  const double step = std::abs(si_factor(factors, factor_unit(entry.quantity)));
  const std::string what =
      object_name(entry.object) + " = " + std::to_string(entry.value) + " " + si_unit_name(entry.quantity);
  if (!std::isfinite(step) || step <= 0.0)
  {
    error = what + ": the drive's unit is not known";
    return std::nullopt;
  }
  const auto layout = integer_type(entry.type);
  if (!layout)
  {
    error = what + ": " + entry.type + " is not an integer type";
    return std::nullopt;
  }
  const double raw = std::round(entry.value / step);
  const double lowest = layout->second ? -std::ldexp(1.0, 8 * layout->first - 1) : 0.0;
  const double highest = std::ldexp(1.0, layout->second ? 8 * layout->first - 1 : 8 * layout->first) - 1.0;
  if (raw < lowest || raw > highest)
  {
    error = what + " does not fit " + entry.type + " (" + std::to_string(raw) + " raw units)";
    return std::nullopt;
  }
  if (entry.value != 0.0 && raw == 0.0)
  {
    error = what + " rounds to 0 in the drive's unit of " + std::to_string(step) + " " + si_unit_name(entry.quantity);
    return std::nullopt;
  }
  if (std::abs(raw * step - entry.value) > 0.01 * std::abs(entry.value))
  {
    error = what + " becomes " + std::to_string(raw * step) + " in the drive's unit, more than 1 % off";
    return std::nullopt;
  }
  return static_cast<int64_t>(raw);
}

std::vector<uint8_t> raw_bytes(int64_t raw, const std::string& type)
{
  const auto layout = integer_type(type);
  if (!layout)
  {
    throw std::invalid_argument("not an integer type: " + type);
  }
  std::vector<uint8_t> bytes(static_cast<std::size_t>(layout->first));
  const auto bits = static_cast<uint64_t>(raw);
  for (std::size_t i = 0; i < bytes.size(); ++i)
  {
    bytes[i] = static_cast<uint8_t>(bits >> (8 * i));
  }
  return bytes;
}

}  // namespace synapticon_hardware::ethercat
