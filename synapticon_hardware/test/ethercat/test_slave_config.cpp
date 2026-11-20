// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Reading a drive's slave_config: the example one, and files with mistakes.

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "synapticon_hardware/ethercat/slave_config.hpp"

namespace
{
using namespace synapticon_hardware::ethercat;

std::string example_path()
{
  return ament_index_cpp::get_package_share_directory("synapticon_hardware") +
         "/config/example_somanet_circulo_safe_motion.yaml";
}

/// `text` written to a temporary file named `name`.
std::string write_file(const std::string& name, const std::string& text)
{
  const auto path = std::filesystem::temp_directory_path() / ("test_slave_config_" + name);
  std::ofstream(path) << text;
  return path.string();
}

/// The channel `index`:`sub_index` of `pdos`.
const PdoChannel& channel(const std::vector<Pdo>& pdos, uint16_t index, uint8_t sub_index = 0)
{
  for (const auto& pdo : pdos)
  {
    for (const auto& c : pdo.channels)
    {
      if (c.index == index && c.sub_index == sub_index)
      {
        return c;
      }
    }
  }
  throw std::runtime_error("no channel");
}

TEST(SlaveConfig, ReadsTheExample)
{
  std::string error;
  const auto config = load_slave_config(example_path(), error);
  ASSERT_TRUE(config) << error;
  EXPECT_EQ(config->vendor_id, 0x22d2u);
  EXPECT_EQ(config->product_id, 0x302u);
  EXPECT_EQ(config->assign_activate, 0x0300u);
  ASSERT_EQ(config->sdo.size(), 2u);
  EXPECT_EQ(config->sdo[0].index, 0x605A);
  EXPECT_EQ(config->sdo[0].value, "6");
  EXPECT_EQ(config->sdo[1].type.kind, ValueKind::REAL);
  ASSERT_EQ(config->rpdo.size(), 1u);
  EXPECT_EQ(config->rpdo[0].index, 0x1600);
  EXPECT_EQ(config->rpdo[0].channels.size(), 8u);
  ASSERT_EQ(config->tpdo.size(), 2u);
  EXPECT_EQ(config->tpdo[1].index, 0x1A02);
}

TEST(SlaveConfig, ReadsTheChannelSettings)
{
  std::string error;
  const auto config = load_slave_config(example_path(), error);
  ASSERT_TRUE(config) << error;
  const PdoChannel& control_word = channel(config->rpdo, 0x6040);
  EXPECT_EQ(control_word.command_interface, "control_word");
  EXPECT_EQ(control_word.type.bits, 16);
  EXPECT_EQ(control_word.default_value, 0.0);
  EXPECT_TRUE(std::isnan(channel(config->rpdo, 0x607A).default_value));
  EXPECT_EQ(channel(config->tpdo, 0x6061).type.kind, ValueKind::SIGNED);
  EXPECT_EQ(channel(config->tpdo, 0x6041).state_interface, "status_word");
  EXPECT_EQ(channel(config->tpdo, 0x203F, 1).type.kind, ValueKind::REAL);
  EXPECT_EQ(channel(config->tpdo, 0x203F, 1).type.bits, 64);
  EXPECT_EQ(channel(config->tpdo, 0x6064).factor, 1.0);
}

TEST(SlaveConfig, ReadsFactorOffsetMaskAndSkip)
{
  const auto path = write_file("settings.yaml", R"(
vendor_id: 1
product_id: 2
tpdo:
  - index: 0x1a00
    channels:
      - { index: 0x6041, sub_index: 0, type: bit4, mask: 12, factor: 2.5, offset: -1, skip: true }
)");
  std::string error;
  const auto config = load_slave_config(path, error);
  ASSERT_TRUE(config) << error;
  const PdoChannel& c = config->tpdo[0].channels[0];
  EXPECT_EQ(c.type.kind, ValueKind::BITS);
  EXPECT_EQ(c.type.bits, 4);
  EXPECT_EQ(c.mask, 12);
  EXPECT_DOUBLE_EQ(c.factor, 2.5);
  EXPECT_DOUBLE_EQ(c.offset, -1.0);
  EXPECT_TRUE(c.skip);
  EXPECT_EQ(config->assign_activate, 0u);
  EXPECT_TRUE(config->rpdo.empty());
}

TEST(SlaveConfig, TakesADefaultOnlyWithACommandInterface)
{
  const auto path = write_file("defaults.yaml", R"(
vendor_id: 1
product_id: 2
rpdo:
  - index: 0x1600
    channels:
      - { index: 0x60fe, sub_index: 1, type: uint32, default: 5 }
      - { index: 0x60ff, sub_index: 0, type: int32, command_interface: velocity, default: 5 }
)");
  std::string error;
  const auto config = load_slave_config(path, error);
  ASSERT_TRUE(config) << error;
  // A default without a command interface is never written, so it is not taken.
  EXPECT_TRUE(std::isnan(channel(config->rpdo, 0x60FE, 1).default_value));
  EXPECT_EQ(channel(config->rpdo, 0x60FF).default_value, 5.0);
}

/// A startup SDO is read back unless its entry says verify: false.
TEST(SlaveConfig, ReadsTheVerifyKeyOfAStartupSdo)
{
  const auto path = write_file("verify.yaml", R"(
vendor_id: 1
product_id: 2
sdo:
  - { index: 0x605a, sub_index: 0, type: int16, value: 6 }
  - { index: 0x1010, sub_index: 1, type: uint32, value: 0x65766173, verify: false }
  - { index: 0x2060, sub_index: 3, type: real32, value: 1.0, verify: true }
)");
  std::string error;
  const auto config = load_slave_config(path, error);
  ASSERT_TRUE(config) << error;
  ASSERT_EQ(config->sdo.size(), 3u);
  EXPECT_TRUE(config->sdo[0].verify);
  EXPECT_FALSE(config->sdo[1].verify);
  EXPECT_TRUE(config->sdo[2].verify);
}

struct BrokenCase
{
  const char* name;
  const char* yaml;
};

class BrokenSlaveConfig : public ::testing::TestWithParam<BrokenCase>
{
};

TEST_P(BrokenSlaveConfig, IsRefusedWithAReason)
{
  std::string error;
  EXPECT_FALSE(load_slave_config(write_file(std::string(GetParam().name) + ".yaml", GetParam().yaml), error));
  EXPECT_FALSE(error.empty());
}

INSTANTIATE_TEST_SUITE_P(
    Cases, BrokenSlaveConfig,
    ::testing::Values(BrokenCase{ "NoIdentity", "rpdo: []\n" },
                      BrokenCase{ "UnknownChannelType", "vendor_id: 1\nproduct_id: 2\nrpdo:\n  - index: 0x1600\n    "
                                                        "channels:\n"
                                                        "      - { index: 0x6040, sub_index: 0, type: int24 }\n" },
                      BrokenCase{ "BitSdo", "vendor_id: 1\nproduct_id: 2\nsdo:\n  - { index: "
                                            "0x605a, sub_index: 0, type: bool, "
                                            "value: 1 }\n" },
                      BrokenCase{ "SdoValueNoNumber", "vendor_id: 1\nproduct_id: 2\nsdo:\n  - { index: "
                                                      "0x6085, sub_index: 0, type: uint32, "
                                                      "value: fast }\n" },
                      BrokenCase{ "SdoVerifyNoBoolean", "vendor_id: 1\nproduct_id: 2\nsdo:\n  - { index: "
                                                        "0x1010, sub_index: 1, type: uint32, "
                                                        "value: 1, verify: 0 }\n" },
                      BrokenCase{ "NotYaml", "vendor_id: [1\n" },
                      BrokenCase{ "SdoSubIndexAbove255", "vendor_id: 1\nproduct_id: 2\nsdo:\n  - { index: "
                                                         "0x605a, sub_index: 256, type: int16, value: 6 }\n" },
                      BrokenCase{ "PdoSubIndexAbove255", "vendor_id: 1\nproduct_id: 2\nrpdo:\n  - index: 0x1600\n    "
                                                         "channels:\n"
                                                         "      - { index: 0x6040, sub_index: 256, type: uint16 }\n" },
                      BrokenCase{ "WordAfterABit", "vendor_id: 1\nproduct_id: 2\ntpdo:\n  - index: 0x1a00\n    "
                                                   "channels:\n"
                                                   "      - { index: 0x2000, sub_index: 1, type: bool }\n"
                                                   "      - { index: 0x6041, sub_index: 0, type: uint16 }\n" },
                      BrokenCase{ "BitsAcrossABytesEnd", "vendor_id: 1\nproduct_id: 2\nrpdo:\n  - index: 0x1600\n    "
                                                         "channels:\n"
                                                         "      - { index: 0x2000, sub_index: 1, type: bit6 }\n"
                                                         "      - { index: 0x2000, sub_index: 2, type: bit4 }\n" }),
    [](const auto& info) { return std::string(info.param.name); });

/// `bool` and `bitN` channels that fill whole bytes may sit between byte-sized ones, also across PDOs.
TEST(SlaveConfig, TakesBitsThatFillTheirBytes)
{
  const auto path = write_file("bits.yaml", R"(
vendor_id: 1
product_id: 2
tpdo:
  - index: 0x1a00
    channels:
      - { index: 0x2000, sub_index: 1, type: bool }
      - { index: 0x2000, sub_index: 2, type: bit7 }
      - { index: 0x6041, sub_index: 0, type: uint16 }
      - { index: 0x2000, sub_index: 3, type: bit4 }
  - index: 0x1a01
    channels:
      - { index: 0x2000, sub_index: 4, type: bit4 }
      - { index: 0x6064, sub_index: 0, type: int32 }
)");
  std::string error;
  EXPECT_TRUE(load_slave_config(path, error)) << error;
}

/// The reason names the channel and where it starts.
TEST(SlaveConfig, NamesAChannelOffItsByte)
{
  const auto path = write_file("off_byte.yaml", R"(
vendor_id: 1
product_id: 2
rpdo:
  - index: 0x1600
    channels:
      - { index: 0x2000, sub_index: 1, type: bit3 }
      - { index: 0x6040, sub_index: 0, type: uint16 }
)");
  std::string error;
  EXPECT_FALSE(load_slave_config(path, error));
  EXPECT_NE(error.find("rpdo channel 0x6040:0 starts at bit 3"), std::string::npos) << error;
}

TEST(SlaveConfig, AMissingFileIsRefused)
{
  std::string error;
  EXPECT_FALSE(load_slave_config("/nonexistent/slave_config.yaml", error));
  EXPECT_NE(error.find("/nonexistent/slave_config.yaml"), std::string::npos);
}

TEST(ValueType, KnowsEverySlaveConfigType)
{
  EXPECT_EQ(value_type("int8")->bits, 8);
  EXPECT_EQ(value_type("uint16")->kind, ValueKind::UNSIGNED);
  EXPECT_EQ(value_type("int64")->bits, 64);
  EXPECT_EQ(value_type("float")->kind, ValueKind::REAL);
  EXPECT_EQ(value_type("double")->bits, 64);
  EXPECT_EQ(value_type("bool")->bits, 1);
  EXPECT_EQ(value_type("bit3")->bits, 3);
  EXPECT_FALSE(value_type("bit9"));
  EXPECT_FALSE(value_type("bitx"));
  EXPECT_FALSE(value_type("string"));
}

}  // namespace
