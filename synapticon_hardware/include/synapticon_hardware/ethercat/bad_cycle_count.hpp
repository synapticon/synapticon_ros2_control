// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Counting the process data cycles in a row that came back incomplete.

namespace synapticon_hardware::ethercat
{

/// @brief The frame in flight, and the cycles in a row without complete process data while the count runs.
///
/// Only a frame that went out is counted. A receive without one finds no frame, which says nothing about the bus.
class BadCycleCount
{
public:
  /// What one received cycle changed.
  enum class Change
  {
    NONE,
    LIMIT_REACHED,   ///< this cycle made `limit` incomplete ones in a row
    COMPLETE_AGAIN,  ///< a complete cycle after the limit was reached
  };

  /// Count from now on, with no bad cycle yet. `limit` incomplete cycles in a row (at least 1) make the data unusable.
  void start(int limit);
  /// Stop counting. The data is usable again, and received() only clears the frame in flight.
  void stop();

  /// A frame went out.
  void sent();
  /// True from sent() until received().
  bool frame_in_flight() const;
  /// The frame in flight came back `complete`, or incomplete or not at all. Counted while the count runs.
  Change received(bool complete);

  /// False while the count runs and the last `limit` cycles were all incomplete.
  bool usable() const;
  int limit() const;

private:
  bool counting_{ false };
  bool in_flight_{ false };
  int limit_{ 1 };
  int bad_{ 0 };
};

}  // namespace synapticon_hardware::ethercat
