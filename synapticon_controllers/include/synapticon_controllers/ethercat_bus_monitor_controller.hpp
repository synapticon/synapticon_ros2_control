// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief A controller that reports the health of the EtherCAT bus.

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <tuple>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "synapticon_controllers/realtime_diagnostics.hpp"

namespace synapticon_controllers
{

/// The bus health that the hardware reports, read in one cycle.
struct BusReading
{
  bool link_up = false;
  uint32_t slaves_responding = 0;
  uint8_t al_states = 0;  ///< One bit per EtherCAT state that at least one slave is in.
  uint32_t working_counter = 0;
  uint8_t wc_state = 0;  ///< Whether the slaves answered the process data: 0 none, 1 some, 2 all.
};

/// @brief Reports the health of the EtherCAT bus on /diagnostics.
///
/// It reads the bus health from the <gpio> that the parameter `bus_name` names ("ethercat_bus" by default): the
/// link, the slaves that answer, their EtherCAT states and how completely they answer the process data. The report,
/// named after the <gpio>, goes out when any of these but the working counter changes, and at least once a second. It
/// is an error without link, and a warning while not every slave answers the process data or is in OP. The drives'
/// own health is the drive controller's report.
class EthercatBusMonitorController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  BusReading read_bus() const;
  /// The report of `bus`, or a stale one before the first reading.
  static void describe(const std::optional<BusReading>& bus, RealtimeDiagnostics<BusReading>::Status& status);

  std::string bus_name_;
  /// The place of each bus field in state_interfaces_, found at activation.
  std::array<std::size_t, 5> slots_{};
  /// The link, the slaves that answer, their states and the process data state of the last report.
  std::optional<std::tuple<bool, uint32_t, uint8_t, uint8_t>> last_reported_;
  RealtimeDiagnostics<BusReading> diagnostics_;
};

}  // namespace synapticon_controllers
