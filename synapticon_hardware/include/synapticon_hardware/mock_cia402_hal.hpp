// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief A HAL with simulated CiA 402 drives, for testing without hardware.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "synapticon_hardware/cia402/cia402_hal.hpp"
#include "synapticon_hardware/drive_data.hpp"
#include "synapticon_hardware/hal_config.hpp"

namespace synapticon_hardware
{

class SimulatedDrive;

/// @brief A HAL with one simulated CiA 402 drive per joint, loaded as `hal` value `mock_cia402`.
///
/// Unlike the passthrough MockHal, the drives have a state machine and a joint that moves: a drive
/// must be enabled before it follows its setpoints, a quick stop ramps it down, and a following
/// error can fault it. The drive action logic is the one of every Cia402Hal, so tests written for
/// a real HAL run here unchanged. The simulation runs in real time, stepped at every read.
///
/// Parameters, all optional:
///   hardware: control_frequency [Hz, 500], quick_stop_hold [true], mode_of_operation [8]
///   joint:    mode_of_operation (-6, 8 or 9), energize [true], quick_stop_option_code [6],
///             quick_stop_deceleration [rad/s^2, 20], following_error [off | report | fault]
class MockCia402Hal : public Cia402Hal
{
public:
  MockCia402Hal();
  ~MockCia402Hal() override;

  bool setup(const HalConfig& config) override;
  bool connect() override;
  bool start() override;
  bool read(std::vector<DriveState>& states) override;
  bool stop() override;
  bool disconnect() override;
  /// The joints as setup() found them, also before connect().
  std::size_t joint_count() const override;
  bool is_connected() const override;
  bool quick_stop_available(std::size_t joint) const override;

protected:
  bool exchange_read() override;
  bool exchange_write() override;
  uint16_t statusword(std::size_t joint) const override;
  uint16_t error_code(std::size_t joint) const override;
  uint16_t last_controlword(std::size_t joint) const override;
  double speed(std::size_t joint) const override;
  double transport_position(std::size_t joint) const override;
  void write_controlword(std::size_t joint, uint16_t controlword) override;
  void write_setpoints(std::size_t joint, const DriveCommand& command) override;
  void write_hold(std::size_t joint, double position) override;

private:
  /// One joint's configuration from setup().
  struct JointConfig;

  double control_frequency_{ 500.0 };
  bool quick_stop_hold_{ true };
  std::vector<JointConfig> joints_;
  bool configured_{ false };
  bool connected_{ false };

  std::vector<SimulatedDrive> drives_;
  std::vector<DriveState> states_;      ///< what the last read saw
  std::vector<DriveCommand> commands_;  ///< what the next write sends
  std::vector<uint16_t> controlwords_;
  std::chrono::steady_clock::time_point last_step_{};
};

}  // namespace synapticon_hardware
