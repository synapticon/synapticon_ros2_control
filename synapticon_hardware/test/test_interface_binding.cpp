// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The interface binding mechanism (NamedField and find_field), exercised on a throwaway struct.

#include <gmock/gmock.h>

#include "synapticon_hardware/interface_binding.hpp"

namespace
{
using synapticon_hardware::find_field;
using synapticon_hardware::NamedField;

struct Owner
{
  double position{ 0.0 };
  double velocity{ 0.0 };
};

enum class Tag
{
  NONE,
  SCALED
};

constexpr NamedField<double, Owner> PLAIN[] = {
  { "position", &Owner::position },
  { "velocity", &Owner::velocity },
};

// Only the second entry states its tag, the first relies on the value-initialized Extra.
constexpr NamedField<double, Owner, Tag> TAGGED[] = {
  { "position", &Owner::position },
  { "velocity", &Owner::velocity, Tag::SCALED },
};

/// find_field() returns the member pointer of the named field.
TEST(InterfaceBindingTest, FindsABoundFieldAndWritesThroughIt)
{
  double Owner::*const field = find_field(PLAIN, "velocity");
  ASSERT_NE(field, nullptr);
  Owner owner;
  owner.*field = 2.5;
  EXPECT_EQ(owner.velocity, 2.5);
  EXPECT_EQ(owner.position, 0.0);
}

/// An unknown or empty name finds nothing.
TEST(InterfaceBindingTest, ReturnsNullForAnUnknownName)
{
  EXPECT_EQ(find_field(PLAIN, "effort"), nullptr);
  EXPECT_EQ(find_field(PLAIN, ""), nullptr);
}

/// An omitted Extra payload is value-initialized.
TEST(InterfaceBindingTest, OmittedExtraIsValueInitialized)
{
  static_assert(TAGGED[0].extra == Tag{});
  EXPECT_EQ(TAGGED[0].extra, Tag::NONE);
  EXPECT_EQ(TAGGED[1].extra, Tag::SCALED);
}

/// The lookup works the same with an Extra payload.
TEST(InterfaceBindingTest, LookupIsIndependentOfTheExtraPayload)
{
  EXPECT_EQ(find_field(TAGGED, "position"), &Owner::position);
  EXPECT_EQ(find_field(TAGGED, "unknown"), nullptr);
}
}  // namespace
