// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Taking the abort code of one SDO transfer from SOEM's error list.

#include <cstdint>

#include <gtest/gtest.h>

#include "error_list.hpp"

namespace
{
using synapticon_ethercat_soem::drain_error_list;
using synapticon_ethercat_soem::SdoAddress;

/// A context with nothing but SOEM's error list.
struct ErrorContext
{
  ErrorContext()
  {
    context.elist = &errors;
    context.ecaterror = &error_flag;
  }

  void push_abort(uint16_t slave, uint16_t index, uint8_t sub_index, uint32_t abort_code)
  {
    ec_errort error{};
    error.Etype = EC_ERR_TYPE_SDO_ERROR;
    error.Slave = slave;
    error.Index = index;
    error.SubIdx = sub_index;
    error.AbortCode = static_cast<int32>(abort_code);
    ecx_pusherror(&context, &error);
  }

  void push_emergency(uint16_t slave, uint16_t error_code)
  {
    ec_errort error{};
    error.Etype = EC_ERR_TYPE_EMERGENCY;
    error.Slave = slave;
    error.ErrorCode = error_code;
    ecx_pusherror(&context, &error);
  }

  ec_eringt errors{};
  boolean error_flag{ FALSE };
  ecx_contextt context{};
};

constexpr SdoAddress TRANSFER{ 2, 0x6092, 1 };
constexpr uint32_t NO_OBJECT = 0x06020000;

TEST(DrainErrorList, ReturnsTheAbortCodeOfTheTransfer)
{
  ErrorContext list;
  list.push_abort(2, 0x6092, 1, NO_OBJECT);
  EXPECT_EQ(drain_error_list(list.context, TRANSFER), NO_OBJECT);
  EXPECT_FALSE(ecx_iserror(&list.context));
}

TEST(DrainErrorList, SkipsOlderEntriesOfOtherKindsSlavesAndObjects)
{
  ErrorContext list;
  list.push_emergency(2, 0x6092);  // the emergency's error code shares its bytes with an abort code
  list.push_abort(1, 0x6092, 1, 0x06090011);
  list.push_abort(2, 0x6092, 2, 0x06090011);
  list.push_abort(2, 0x6091, 1, 0x06090011);
  list.push_abort(2, 0x6092, 1, NO_OBJECT);
  EXPECT_EQ(drain_error_list(list.context, TRANSFER), NO_OBJECT);
  EXPECT_FALSE(ecx_iserror(&list.context));
}

TEST(DrainErrorList, GivesNoCodeWhenTheTransferHasNoAbort)
{
  ErrorContext list;
  list.push_emergency(2, 0x8611);
  list.push_abort(1, 0x6092, 1, NO_OBJECT);
  EXPECT_EQ(drain_error_list(list.context, TRANSFER), 0u);
  EXPECT_FALSE(ecx_iserror(&list.context));
}

TEST(DrainErrorList, EmptiesTheListWithoutATransfer)
{
  ErrorContext list;
  list.push_abort(2, 0x6092, 1, NO_OBJECT);
  list.push_emergency(2, 0x8611);
  EXPECT_EQ(drain_error_list(list.context), 0u);
  EXPECT_FALSE(ecx_iserror(&list.context));
  EXPECT_EQ(drain_error_list(list.context, TRANSFER), 0u);
}

}  // namespace
