// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Counting the process data cycles in a row that came back incomplete.

#include "bad_cycle_count.hpp"

namespace synapticon_ethercat_soem
{

void BadCycleCount::start(int limit)
{
  counting_ = true;
  limit_ = limit;
  bad_ = 0;
}

void BadCycleCount::stop()
{
  counting_ = false;
  bad_ = 0;
}

void BadCycleCount::sent()
{
  in_flight_ = true;
}

bool BadCycleCount::frame_in_flight() const
{
  return in_flight_;
}

BadCycleCount::Change BadCycleCount::received(bool complete)
{
  in_flight_ = false;
  if (!counting_)
  {
    return Change::NONE;
  }
  if (complete)
  {
    const bool was_unusable = bad_ >= limit_;
    bad_ = 0;
    return was_unusable ? Change::COMPLETE_AGAIN : Change::NONE;
  }
  if (bad_ < limit_ && ++bad_ == limit_)
  {
    return Change::LIMIT_REACHED;
  }
  return Change::NONE;
}

bool BadCycleCount::usable() const
{
  return !counting_ || bad_ < limit_;
}

int BadCycleCount::limit() const
{
  return limit_;
}

}  // namespace synapticon_ethercat_soem
