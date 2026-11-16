// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief MockHal, a loopback HAL for tests that run without any drive.

#include <cstddef>
#include <string>
#include <vector>

#include "synapticon_hardware/hal_config.hpp"
#include "synapticon_hardware/drive_data.hpp"
#include "synapticon_hardware/synapticon_hal.hpp"

namespace synapticon_hardware
{

/// @brief A loopback HAL: no drive behind it, the commands it gets are the states it reports.
///
/// Like a cable plugged back into its own socket, write() stores the commanded position, velocity and effort, and
/// read() hands them back. set_states() replaces what read() returns, for states a loopback cannot produce.
///
/// It follows the call rules of SynapticonHalI: an up-call out of order is logged and refused (false), with no
/// effect. In write(), a NaN field keeps the stored value. An infinite field keeps it too, and write() returns false.
class MockHal : public SynapticonHalI
{
public:
  MockHal() noexcept;
  ~MockHal() override = default;

  /// Configure the mock with nothing but its joint names, in joint order.
  bool setup(std::vector<std::string> joint_names);
  /// Configure the mock with the names of the joints in `config`. All other parameters are ignored.
  bool setup(const HalConfig& config) override;

  bool connect() override;
  bool start() override;
  bool enable() override;
  bool read(std::vector<DriveState>& states) override;
  bool write(const std::vector<DriveCommand>& commands) override;
  bool disable() override;
  bool stop() override;
  bool disconnect() override;
  std::size_t joint_count() const override;
  bool is_connected() const override;
  bool is_streaming() const override;
  bool is_enabled() const override;

  /// Overwrite the states that read() returns, one per joint. False, logged, for any other count.
  bool set_states(const std::vector<DriveState>& states);

private:
  /// The stages of a HAL, from the bottom up. A higher stage implies every lower one.
  enum class Level
  {
    DISCONNECTED,
    CONNECTED,
    STREAMING,
    ENABLED
  };

  /// Climb to `next` on behalf of `call`, from the stage just below it. Nothing changes when the mock is at `next`
  /// or above already. Refused, with a log line, from any lower stage.
  bool climb(const char* call, Level next);

  std::vector<std::string> names_;
  std::vector<DriveState> reported_;
  Level level_{ Level::DISCONNECTED };
  bool configured_{ false };
};

}  // namespace synapticon_hardware
