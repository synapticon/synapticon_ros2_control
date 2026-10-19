// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The bytes of startup SDO values and PDO mapping entries.

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "coe_encoding.hpp"

namespace
{
using namespace synapticon_ethercat_soem;
using Bytes = std::vector<uint8_t>;

StartupSdo sdo(const char* type, const std::string& value)
{
  StartupSdo s;
  s.index = 0x6085;
  s.type = *value_type(type);
  s.value = value;
  return s;
}

TEST(SdoBytes, IntegersAreLittleEndianInTheirSize)
{
  EXPECT_EQ(sdo_bytes(sdo("uint8", "6")), (Bytes{ 0x06 }));
  EXPECT_EQ(sdo_bytes(sdo("int16", "-2")), (Bytes{ 0xFE, 0xFF }));
  EXPECT_EQ(sdo_bytes(sdo("uint32", "100000")), (Bytes{ 0xA0, 0x86, 0x01, 0x00 }));
  EXPECT_EQ(sdo_bytes(sdo("uint16", "0x1234")), (Bytes{ 0x34, 0x12 }));
}

TEST(SdoBytes, MinusOneSetsEveryBitOfAnUnsignedObject)
{
  EXPECT_EQ(sdo_bytes(sdo("uint32", "-1")), (Bytes{ 0xFF, 0xFF, 0xFF, 0xFF }));
}

TEST(SdoBytes, TheWholeUint64RangeIsTaken)
{
  EXPECT_EQ(sdo_bytes(sdo("uint64", "18446744073709551615")), Bytes(8, 0xFF));
  EXPECT_EQ(sdo_bytes(sdo("uint64", "9223372036854775808")), (Bytes{ 0, 0, 0, 0, 0, 0, 0, 0x80 }));
}

TEST(SdoBytes, RealsAreCopiedBitForBit)
{
  const float single = 1.5F;
  Bytes expected(4);
  std::memcpy(expected.data(), &single, sizeof(single));
  EXPECT_EQ(sdo_bytes(sdo("real32", "1.5")), expected);
  const double twice = -2.25;
  expected.resize(8);
  std::memcpy(expected.data(), &twice, sizeof(twice));
  EXPECT_EQ(sdo_bytes(sdo("real64", "-2.25")), expected);
}

TEST(SdoBytes, AValueThatIsNoNumberThrows)
{
  EXPECT_THROW(sdo_bytes(sdo("uint16", "fast")), std::invalid_argument);
  EXPECT_THROW(sdo_bytes(sdo("int64", "99999999999999999999")), std::out_of_range);
  EXPECT_THROW(sdo_bytes(sdo("uint32", "12abc")), std::invalid_argument);
}

TEST(SdoBytes, AnIntegerTooWideForTheObjectThrows)
{
  EXPECT_THROW(sdo_bytes(sdo("uint8", "300")), std::out_of_range);
  EXPECT_THROW(sdo_bytes(sdo("int8", "128")), std::out_of_range);
  EXPECT_THROW(sdo_bytes(sdo("int16", "-32769")), std::out_of_range);
  EXPECT_THROW(sdo_bytes(sdo("uint16", "-32769")), std::out_of_range);
  EXPECT_EQ(sdo_bytes(sdo("uint8", "255")), (Bytes{ 0xFF }));
  EXPECT_EQ(sdo_bytes(sdo("int8", "-128")), (Bytes{ 0x80 }));
}

TEST(PdoEntryBytes, HoldIndexSubIndexAndBitLength)
{
  PdoChannel channel;
  channel.index = 0x6040;
  channel.sub_index = 0;
  channel.type = *value_type("uint16");
  EXPECT_EQ(pdo_entry_bytes(channel), (Bytes{ 0x10, 0x00, 0x40, 0x60 }));
  channel.index = 0x203F;
  channel.sub_index = 1;
  channel.type = *value_type("real64");
  EXPECT_EQ(pdo_entry_bytes(channel), (Bytes{ 0x40, 0x01, 0x3F, 0x20 }));
}

}  // namespace
