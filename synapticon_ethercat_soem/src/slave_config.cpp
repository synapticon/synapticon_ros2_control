// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Reading a drive's slave_config YAML for the SOEM HAL.

#include "slave_config.hpp"

#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "coe_encoding.hpp"

namespace synapticon_ethercat_soem
{

namespace
{
/// The PDOs of `node` (a `rpdo` or `tpdo` list).
std::vector<Pdo> read_pdos(const YAML::Node& node)
{
  std::vector<Pdo> pdos;
  for (const auto& pdo_node : node)
  {
    Pdo pdo;
    pdo.index = pdo_node["index"].as<uint16_t>();
    for (const auto& c : pdo_node["channels"])
    {
      PdoChannel channel;
      channel.index = c["index"].as<uint16_t>();
      channel.sub_index = static_cast<uint8_t>(c["sub_index"].as<unsigned>());
      const std::string type = c["type"].as<std::string>();
      const auto parsed = value_type(type);
      if (!parsed)
      {
        throw std::runtime_error("unknown type '" + type + "'");
      }
      channel.type = *parsed;
      if (c["command_interface"])
      {
        channel.command_interface = c["command_interface"].as<std::string>();
        if (c["default"])
        {
          channel.default_value = c["default"].as<double>();
        }
      }
      if (c["state_interface"])
      {
        channel.state_interface = c["state_interface"].as<std::string>();
      }
      if (c["factor"])
      {
        channel.factor = c["factor"].as<double>();
      }
      if (c["offset"])
      {
        channel.offset = c["offset"].as<double>();
      }
      if (c["mask"])
      {
        channel.mask = static_cast<uint8_t>(c["mask"].as<unsigned>());
      }
      if (c["skip"])
      {
        channel.skip = c["skip"].as<bool>();
      }
      pdo.channels.push_back(channel);
    }
    pdos.push_back(pdo);
  }
  return pdos;
}
}  // namespace

std::optional<ValueType> value_type(const std::string& name)
{
  if (name == "bool")
  {
    return ValueType{ ValueKind::BITS, 1 };
  }
  if (name.rfind("bit", 0) == 0)
  {
    try
    {
      const int bits = std::stoi(name.substr(3));
      if (bits > 0 && bits <= 8)
      {
        return ValueType{ ValueKind::BITS, static_cast<uint8_t>(bits) };
      }
    }
    catch (const std::exception&)
    {
    }
    return std::nullopt;
  }
  struct Named
  {
    const char* name;
    ValueType type;
  };
  constexpr Named TYPES[] = {
    { "int8", { ValueKind::SIGNED, 8 } },   { "uint8", { ValueKind::UNSIGNED, 8 } },
    { "int16", { ValueKind::SIGNED, 16 } }, { "uint16", { ValueKind::UNSIGNED, 16 } },
    { "int32", { ValueKind::SIGNED, 32 } }, { "uint32", { ValueKind::UNSIGNED, 32 } },
    { "int64", { ValueKind::SIGNED, 64 } }, { "uint64", { ValueKind::UNSIGNED, 64 } },
    { "real32", { ValueKind::REAL, 32 } },  { "float", { ValueKind::REAL, 32 } },
    { "real64", { ValueKind::REAL, 64 } },  { "double", { ValueKind::REAL, 64 } },
  };
  for (const auto& entry : TYPES)
  {
    if (name == entry.name)
    {
      return entry.type;
    }
  }
  return std::nullopt;
}

std::optional<SlaveConfig> load_slave_config(const std::string& path, std::string& error)
{
  try
  {
    const YAML::Node root = YAML::LoadFile(path);
    if (!root["vendor_id"] || !root["product_id"])
    {
      error = "slave_config '" + path + "' has no vendor_id or product_id";
      return std::nullopt;
    }
    SlaveConfig config;
    config.vendor_id = root["vendor_id"].as<uint32_t>();
    config.product_id = root["product_id"].as<uint32_t>();
    if (root["assign_activate"])
    {
      config.assign_activate = root["assign_activate"].as<uint32_t>();
    }
    for (const auto& s : root["sdo"])
    {
      StartupSdo sdo;
      sdo.index = s["index"].as<uint16_t>();
      sdo.sub_index = static_cast<uint8_t>(s["sub_index"].as<unsigned>());
      char object[16];
      std::snprintf(object, sizeof(object), "0x%04X:%u", sdo.index, sdo.sub_index);
      const std::string type = s["type"].as<std::string>();
      const auto parsed = value_type(type);
      if (!parsed || parsed->kind == ValueKind::BITS)
      {
        throw std::runtime_error(std::string("SDO ") + object + " has the unsupported type '" + type + "'");
      }
      sdo.type = *parsed;
      sdo.value = s["value"].as<std::string>();
      try
      {
        sdo_bytes(sdo);
      }
      catch (const std::exception&)
      {
        throw std::runtime_error(std::string("SDO ") + object + " has the value '" + sdo.value + "', which is not a " +
                                 type);
      }
      config.sdo.push_back(sdo);
    }
    config.rpdo = read_pdos(root["rpdo"]);
    config.tpdo = read_pdos(root["tpdo"]);
    return config;
  }
  catch (const std::exception& e)
  {
    error = "slave_config '" + path + "': " + e.what();
    return std::nullopt;
  }
}

}  // namespace synapticon_ethercat_soem
