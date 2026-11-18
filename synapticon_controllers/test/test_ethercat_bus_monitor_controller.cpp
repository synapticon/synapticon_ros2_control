// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Tests for EthercatBusMonitorController: its setup and the diagnostics it reports for the bus health.

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>

#include "rclcpp/rclcpp.hpp"
#include "synapticon_controllers/ethercat_bus_monitor_controller.hpp"
#include "controller_test_utils.hpp"

namespace
{
namespace test = synapticon_controllers::test;
using diagnostic_msgs::msg::DiagnosticStatus;

constexpr const char* BUS = "ethercat_bus";
constexpr uint8_t WC_INCOMPLETE = 1;
constexpr uint8_t WC_COMPLETE = 2;

/// The bus monitor with its last report in reach.
class ProbedBusMonitor : public synapticon_controllers::EthercatBusMonitorController
{
public:
  /// The report on /diagnostics, as the updater would publish it now.
  DiagnosticStatus report()
  {
    diagnostic_updater::DiagnosticStatusWrapper status;
    diagnostics_.describe(0, status);
    return status;
  }

  std::size_t publish_changes()
  {
    return diagnostics_.publish_changes();
  }

  std::string value(const std::string& key)
  {
    for (const auto& entry : report().values)
    {
      if (entry.key == key)
      {
        return entry.value;
      }
    }
    return "missing";
  }
};

/// The bus-health interfaces of the gpio `bus`, without `left_out`.
test::InterfaceHandles bus_handles(const std::string& left_out = "")
{
  test::InterfaceHandles handles;
  const std::vector<std::pair<const char*, const char*>> fields = {
    { "link_up", "bool" },          { "slaves_responding", "uint32" },
    { "al_states", "uint8" },       { "domain_working_counter", "uint32" },
    { "domain_wc_state", "uint8" },
  };
  for (const auto& [name, type] : fields)
  {
    if (left_out != name)
    {
      handles.add_state(BUS, name, type);
    }
  }
  return handles;
}

// --- Setup ------------------------------------------------------------------------

TEST(BusMonitorSetup, ABusNameOfTheWrongTypeFailsTheInit)
{
  synapticon_controllers::EthercatBusMonitorController monitor;
  EXPECT_EQ(test::init(monitor, "test_bus_monitor", { rclcpp::Parameter("bus_name", 5) }),
            controller_interface::return_type::ERROR);
}

TEST(BusMonitorSetup, AnEmptyBusNameFailsTheConfiguration)
{
  synapticon_controllers::EthercatBusMonitorController monitor;
  ASSERT_EQ(test::init(monitor, "test_bus_monitor", { rclcpp::Parameter("bus_name", "") }),
            controller_interface::return_type::OK);
  EXPECT_EQ(monitor.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::FAILURE);
}

TEST(BusMonitorSetup, AZeroCheckPeriodFailsTheConfiguration)
{
  synapticon_controllers::EthercatBusMonitorController monitor;
  ASSERT_EQ(test::init(monitor, "test_bus_monitor", { rclcpp::Parameter("diagnostics_check_period", 0.0) }),
            controller_interface::return_type::OK);
  EXPECT_EQ(monitor.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::FAILURE);
}

TEST(BusMonitorSetup, AMissingBusInterfaceFailsTheActivation)
{
  synapticon_controllers::EthercatBusMonitorController monitor;
  ASSERT_EQ(test::init(monitor, "test_bus_monitor", {}), controller_interface::return_type::OK);
  ASSERT_EQ(monitor.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(test::activate(monitor, bus_handles("domain_wc_state")), controller_interface::CallbackReturn::ERROR);
  monitor.release_interfaces();
}

// --- Reports ----------------------------------------------------------------------

/// The monitor over a healthy bus with one slave in OP.
class BusMonitor : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test::bring_up(monitor_, "test_bus_monitor", {}, handles_);
    set_bus(true, 0x08, WC_COMPLETE);
  }

  void TearDown() override
  {
    test::shut_down(monitor_);
  }

  void set_bus(bool link_up, uint8_t al_states, uint8_t wc_state, uint32_t slaves_responding = 1)
  {
    ASSERT_TRUE(handles_.states[0]->set_value<bool>(link_up));
    ASSERT_TRUE(handles_.states[1]->set_value<uint32_t>(slaves_responding));
    ASSERT_TRUE(handles_.states[2]->set_value<uint8_t>(al_states));
    ASSERT_TRUE(handles_.states[3]->set_value<uint32_t>(3));
    ASSERT_TRUE(handles_.states[4]->set_value<uint8_t>(wc_state));
  }

  /// One update a second after the last, so it always reports.
  void update()
  {
    seconds_ += 1;
    ASSERT_EQ(monitor_.update(rclcpp::Time(seconds_, 0), rclcpp::Duration(0, 0)), controller_interface::return_type::OK);
  }

  test::InterfaceHandles handles_ = bus_handles();
  ProbedBusMonitor monitor_;
  int seconds_{ 0 };
};

TEST_F(BusMonitor, BeforeTheFirstUpdateTheReportIsStale)
{
  EXPECT_EQ(monitor_.report().level, DiagnosticStatus::STALE);
  EXPECT_EQ(monitor_.report().hardware_id, BUS);
}

TEST_F(BusMonitor, AHealthyBusIsOk)
{
  update();
  EXPECT_EQ(monitor_.report().level, DiagnosticStatus::OK);
  EXPECT_EQ(monitor_.report().message, "EtherCAT link up, process data COMPLETE");
  EXPECT_EQ(monitor_.value("al_states"), "OP");
  EXPECT_EQ(monitor_.value("slaves_responding"), "1");
  EXPECT_EQ(monitor_.report().hardware_id, BUS);
}

TEST_F(BusMonitor, ChangesOfLinkSlavesStatesOrDomainAreQueued)
{
  update();  // the first reading
  update();
  set_bus(true, 0x08, WC_INCOMPLETE);
  update();
  set_bus(true, 0x0A, WC_INCOMPLETE);  // a slave left OP
  update();
  set_bus(true, 0x0A, WC_INCOMPLETE, 2);  // one more slave answers
  update();
  update();
  EXPECT_EQ(monitor_.publish_changes(), 4u);
}

TEST_F(BusMonitor, TheWorkingCounterAloneIsNoChange)
{
  update();
  ASSERT_TRUE(handles_.states[3]->set_value<uint32_t>(4));
  update();
  EXPECT_EQ(monitor_.publish_changes(), 1u);
}

TEST_F(BusMonitor, ASlaveOutsideOpIsAWarning)
{
  set_bus(true, 0x0A, WC_COMPLETE);
  update();
  EXPECT_EQ(monitor_.report().level, DiagnosticStatus::WARN);
  EXPECT_EQ(monitor_.report().message, "EtherCAT link up, process data COMPLETE, not every slave in OP");
}

TEST_F(BusMonitor, AnIncompleteDomainIsAWarning)
{
  set_bus(true, 0x08, WC_INCOMPLETE);
  update();
  EXPECT_EQ(monitor_.report().level, DiagnosticStatus::WARN);
  EXPECT_EQ(monitor_.value("domain_wc_state"), "INCOMPLETE");
}

TEST_F(BusMonitor, ALinkDownIsAnError)
{
  set_bus(false, 0x00, WC_COMPLETE);
  update();
  EXPECT_EQ(monitor_.report().level, DiagnosticStatus::ERROR);
  EXPECT_EQ(monitor_.report().message, "No EtherCAT link");
}

struct AlStatesCase
{
  uint8_t mask;
  const char* text;
};

class AlStates : public BusMonitor, public ::testing::WithParamInterface<AlStatesCase>
{
};

TEST_P(AlStates, AreReportedAsAMask)
{
  set_bus(true, GetParam().mask, WC_COMPLETE);
  update();
  EXPECT_EQ(monitor_.value("al_states"), GetParam().text);
}

INSTANTIATE_TEST_SUITE_P(Masks, AlStates,
                         ::testing::Values(AlStatesCase{ 0x00, "none" }, AlStatesCase{ 0x01, "INIT" },
                                           AlStatesCase{ 0x0A, "PREOP, OP" },
                                           AlStatesCase{ 0x18, "OP, other bits 0x10" },
                                           AlStatesCase{ 0x20, "other bits 0x20" }));

}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
