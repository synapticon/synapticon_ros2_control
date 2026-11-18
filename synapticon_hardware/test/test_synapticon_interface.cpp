// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Interface binding and refused-access reporting in SynapticonSystemInterface.
///
/// Drives the component through the lifecycle the ResourceManager would, on the mock HAL,
/// and then holds an exported interface's lock to force the handle layer to refuse an access.
/// That refusal cannot happen in the sequential read/update/write model, so forcing it is the
/// only way to check that it is counted and reported rather than silently dropped.

#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

#include <gmock/gmock.h>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "synapticon_hardware/mock_hal.hpp"
#include "synapticon_hardware/cia402/state_machine.hpp"
#include "synapticon_hardware/synapticon_interface.hpp"
#include "description_builders.hpp"

namespace
{
using hardware_interface::CallbackReturn;
using synapticon_hardware::DriveAction;
using synapticon_hardware::DriveCommand;
using synapticon_hardware::MockHal;
using synapticon_hardware::SynapticonHalI;
using synapticon_hardware::SynapticonSystemInterface;
using synapticon_hardware::test::bus_gpio;
using synapticon_hardware::test::interface_info;
using synapticon_hardware::test::joint_info;
using synapticon_hardware::test::motion_states;
using synapticon_hardware::test::simple_transmission;
using synapticon_hardware::test::system_info;

const std::vector<std::string> JOINT_NAMES{ "joint_1", "joint_2" };

/// A drive-backed joint carrying the typed drive interfaces. Without `commanded`, it exposes
/// no setpoint command: a motor shaft that a transmission commands.
hardware_interface::ComponentInfo drive_joint(const std::string& name, bool commanded = true)
{
  std::vector<hardware_interface::InterfaceInfo> commands;
  if (commanded)
  {
    commands.push_back(interface_info("position"));
  }
  commands.push_back(interface_info("drive_action", "uint8"));
  auto states = motion_states();
  states.insert(states.end(), {
                                  interface_info("status_word", "uint16"),
                                  interface_info("device_state", "uint8"),
                                  interface_info("error_code", "uint16"),
                                  interface_info("mode_of_operation", "int8"),
                                  interface_info("slave_al_state", "uint8"),
                                  interface_info("slave_online", "bool"),
                              });
  return joint_info(name, commands, states);
}

/// A minimal but representative <ros2_control> description: the mock HAL, two drive-backed
/// joints carrying the typed drive interfaces, and the bus-health <gpio>.
hardware_interface::HardwareInfo make_hardware_info()
{
  auto info = system_info("test_synapticon_interface");
  for (const auto& joint_name : JOINT_NAMES)
  {
    info.joints.push_back(drive_joint(joint_name));
  }
  info.gpios.push_back(bus_gpio());
  return info;
}

constexpr double REDUCTION = 4.0;
constexpr double OFFSET = 0.25;

/// The description of a robot with one output joint driven through a standard simple
/// transmission by one motor drive: the motor is drive-backed but not commanded, the output has
/// no drive of its own and only motion interfaces.
hardware_interface::HardwareInfo make_transmission_hardware_info()
{
  auto info = system_info("test_synapticon_interface_transmission");
  info.joints.push_back(joint_info("output", { interface_info("position") }, motion_states()));
  info.joints.push_back(drive_joint("motor", false));
  info.transmissions.push_back(simple_transmission("tx_output", "output", "motor", REDUCTION, OFFSET));
  return info;
}

rclcpp_lifecycle::State lifecycle_state(uint8_t id, const std::string& label)
{
  return rclcpp_lifecycle::State(id, label);
}

/// A mock HAL that keeps the last command batch it was handed, so a test can check what crossed
/// the HAL boundary rather than inferring it from echoed state. A test can also set what it measures: the drive's
/// position and statusword, per drive, where `positions` or `status_words` has an entry.
class RecordingMockHal : public MockHal
{
public:
  bool read(std::vector<synapticon_hardware::DriveState>& states) override
  {
    if (!MockHal::read(states))
    {
      return false;
    }
    for (std::size_t drive = 0; drive < states.size(); ++drive)
    {
      if (drive < positions.size())
      {
        states[drive].position = positions[drive];
      }
      if (drive < status_words.size())
      {
        states[drive].status_word = status_words[drive];
      }
    }
    return true;
  }

  bool write(const std::vector<DriveCommand>& commands) override
  {
    last_commands = commands;
    return MockHal::write(commands);
  }

  std::vector<DriveCommand> last_commands;
  std::vector<double> positions;
  std::vector<uint16_t> status_words;
};

/// Statuswords of a drive with and without power.
constexpr uint16_t OPERATION_ENABLED = 0x27;
constexpr uint16_t SWITCH_ON_DISABLED = 0x40;

/// The component with the recording HAL injected through make_hal().
class RecordingSystem : public SynapticonSystemInterface
{
public:
  RecordingMockHal* client{ nullptr };

protected:
  std::shared_ptr<SynapticonHalI> make_hal() override
  {
    auto owned = std::make_shared<RecordingMockHal>();
    client = owned.get();
    return owned;
  }
};

/// The component brought up to ACTIVE on the mock HAL, exactly as the ResourceManager does
/// it: on_init, export the interfaces, then configure and activate.
class SynapticonSystemInterfaceTest : public ::testing::Test
{
protected:
  /// The description the component is initialised with.
  virtual hardware_interface::HardwareInfo hardware_info() const
  {
    return make_hardware_info();
  }

  void SetUp() override
  {
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = hardware_info();
    ASSERT_EQ(system_.on_init(params), CallbackReturn::SUCCESS);
    ASSERT_FALSE(system_.on_export_state_interfaces().empty());
    ASSERT_FALSE(system_.on_export_command_interfaces().empty());
    ASSERT_EQ(
        system_.on_configure(lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured")),
        CallbackReturn::SUCCESS);
    ASSERT_EQ(system_.on_activate(lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive")),
              CallbackReturn::SUCCESS);
  }

  void TearDown() override
  {
    system_.on_deactivate(lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"));
    system_.on_cleanup(lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"));
  }

  /// One read() and write() cycle.
  void cycle()
  {
    EXPECT_EQ(system_.read(rclcpp::Time(0), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
    EXPECT_EQ(system_.write(rclcpp::Time(0), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
  }

  /// The request this joint was given on the last cycle.
  DriveAction commanded_action(std::size_t joint_index) const
  {
    return system_.client->last_commands.at(joint_index).drive_action;
  }

  RecordingSystem system_;
};

// ---------------------------------------------------------------------------
// Binding
// ---------------------------------------------------------------------------

/// A description with only known interfaces reports none as unknown.
TEST_F(SynapticonSystemInterfaceTest, KnownInterfacesAreNotReportedAsUnknown)
{
  EXPECT_TRUE(system_.unknown_interfaces().empty());
}

/// The description with a misspelled command, an unrelated state and an unknown gpio state.
class SynapticonSystemInterfaceUnknownInterfaceTest : public SynapticonSystemInterfaceTest
{
protected:
  hardware_interface::HardwareInfo hardware_info() const override
  {
    auto info = make_hardware_info();
    info.joints[0].command_interfaces.push_back(interface_info("stifness", "double"));
    info.joints[1].state_interfaces.push_back(interface_info("brake_released", "double"));
    info.gpios[0].state_interfaces.push_back(interface_info("cable_temperature", "double"));
    return info;
  }
};

/// Unknown interfaces do not fail the bring-up, and every one of them is reported.
TEST_F(SynapticonSystemInterfaceUnknownInterfaceTest, EveryUnknownInterfaceIsReported)
{
  EXPECT_THAT(system_.unknown_interfaces(),
              ::testing::UnorderedElementsAre("joint_1/stifness (command)", "joint_2/brake_released (state)",
                                              "ethercat_bus/cable_temperature (state)"));
  cycle();
}

/// Every exported interface carries the data type the description declares.
TEST_F(SynapticonSystemInterfaceTest, TypedInterfacesAreExportedWithTheirDeclaredTypes)
{
  EXPECT_EQ(system_.get_state_interface_handle("joint_1/status_word")->get_data_type(),
            hardware_interface::HandleDataType::UINT16);
  EXPECT_EQ(system_.get_state_interface_handle("joint_1/mode_of_operation")->get_data_type(),
            hardware_interface::HandleDataType::INT8);
  EXPECT_EQ(system_.get_state_interface_handle("joint_1/slave_online")->get_data_type(),
            hardware_interface::HandleDataType::BOOL);
  EXPECT_EQ(system_.get_state_interface_handle("ethercat_bus/slaves_responding")->get_data_type(),
            hardware_interface::HandleDataType::UINT32);
  EXPECT_EQ(system_.get_command_interface_handle("joint_1/drive_action")->get_data_type(),
            hardware_interface::HandleDataType::UINT8);
  EXPECT_EQ(system_.get_state_interface_handle("joint_1/position")->get_data_type(),
            hardware_interface::HandleDataType::DOUBLE);
}

/// device_state is a uint8 state filled from the statusword after every read, so controllers never decode it.
TEST_F(SynapticonSystemInterfaceTest, DeviceStateIsTheDecodedStatusword)
{
  EXPECT_EQ(system_.get_state_interface_handle("joint_1/device_state")->get_data_type(),
            hardware_interface::HandleDataType::UINT8);
  cycle();
  const uint16_t status_word =
      system_.get_state_interface_handle("joint_1/status_word")->get_optional<uint16_t>().value();
  EXPECT_EQ(system_.get_state_interface_handle("joint_1/device_state")->get_optional<uint8_t>().value(),
            static_cast<uint8_t>(cia402::device_state(status_word)));
}

/// On a HAL without a link layer the health states read as healthy.
TEST_F(SynapticonSystemInterfaceTest, HealthStatesReadAsHealthyOnANonEthercatHal)
{
  cycle();
  EXPECT_TRUE(system_.get_state_interface_handle("joint_1/slave_online")->get_optional<bool>().value());
  EXPECT_TRUE(system_.get_state_interface_handle("ethercat_bus/link_up")->get_optional<bool>().value());
  EXPECT_EQ(system_.get_state_interface_handle("ethercat_bus/slaves_responding")->get_optional<uint32_t>().value(),
            JOINT_NAMES.size());
}

/// The drive_action interface reaches the HAL unchanged every cycle, for its own joint only, and does not latch.
TEST_F(SynapticonSystemInterfaceTest, TheDriveCommandInterfaceReachesTheClient)
{
  cycle();
  EXPECT_EQ(commanded_action(0), DriveAction::NONE);

  const auto handle = system_.get_command_interface_handle("joint_1/drive_action");
  ASSERT_TRUE(handle->set_value(static_cast<uint8_t>(DriveAction::ENABLE)));
  cycle();
  EXPECT_EQ(commanded_action(0), DriveAction::ENABLE);
  EXPECT_EQ(commanded_action(1), DriveAction::NONE) << "a request must not leak to another joint";

  ASSERT_TRUE(handle->set_value(static_cast<uint8_t>(DriveAction::DISABLE)));
  cycle();
  EXPECT_EQ(commanded_action(0), DriveAction::DISABLE);

  ASSERT_TRUE(handle->set_value(static_cast<uint8_t>(DriveAction::NONE)));
  cycle();
  EXPECT_EQ(commanded_action(0), DriveAction::NONE);
}

/// An infinite setpoint reaches the HAL unchanged, the HAL refuses it, and write() reports an error.
TEST_F(SynapticonSystemInterfaceTest, AnInfiniteSetpointIsAWriteError)
{
  cycle();
  const auto position = system_.get_command_interface_handle("joint_1/position");
  ASSERT_TRUE(position->set_value(std::numeric_limits<double>::infinity()));
  EXPECT_EQ(system_.read(rclcpp::Time(0), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
  EXPECT_EQ(system_.write(rclcpp::Time(0), rclcpp::Duration(0, 0)), hardware_interface::return_type::ERROR);
  EXPECT_TRUE(std::isinf(system_.client->last_commands.at(0).position)) << "infinity must not become a hold";

  ASSERT_TRUE(position->set_value(0.5));
  cycle();
}

// ---------------------------------------------------------------------------
// Refused interface accesses
// ---------------------------------------------------------------------------

/// Uncontended cycling counts no refused access.
TEST_F(SynapticonSystemInterfaceTest, NormalCyclingRefusesNothing)
{
  for (int i = 0; i < 20; ++i)
  {
    cycle();
  }
  EXPECT_EQ(system_.handle_access_failures().total(), 0u);
}

/// A state write refused by a locked handle is counted as a state write failure.
TEST_F(SynapticonSystemInterfaceTest, ALockedStateInterfaceIsCountedNotDropped)
{
  const auto handle = system_.get_state_interface_handle("joint_1/status_word");
  {
    const std::unique_lock<std::shared_mutex> hold(handle->get_mutex());
    cycle();
  }
  EXPECT_EQ(system_.handle_access_failures().state_write, 1u);
  EXPECT_EQ(system_.handle_access_failures().action_read, 0u);
  EXPECT_EQ(system_.handle_access_failures().setpoint_read, 0u);
}

/// A refused drive_action read loses the request, so it is counted apart from the setpoint reads, which go stale.
TEST_F(SynapticonSystemInterfaceTest, ALockedDriveCommandInterfaceIsCountedSeparately)
{
  const auto handle = system_.get_command_interface_handle("joint_1/drive_action");
  {
    const std::unique_lock<std::shared_mutex> hold(handle->get_mutex());
    cycle();
  }
  EXPECT_EQ(system_.handle_access_failures().action_read, 1u);
  EXPECT_EQ(system_.handle_access_failures().setpoint_read, 0u);
  EXPECT_EQ(system_.handle_access_failures().state_write, 0u);
}

/// A refused setpoint read is counted as a setpoint read failure only.
TEST_F(SynapticonSystemInterfaceTest, ALockedSetpointInterfaceIsCountedSeparately)
{
  const auto handle = system_.get_command_interface_handle("joint_1/position");
  {
    const std::unique_lock<std::shared_mutex> hold(handle->get_mutex());
    cycle();
  }
  EXPECT_EQ(system_.handle_access_failures().setpoint_read, 1u);
  EXPECT_EQ(system_.handle_access_failures().action_read, 0u);
  EXPECT_EQ(system_.handle_access_failures().state_write, 0u);
}

/// Refusals add up over cycles and are cleared by the next activation.
TEST_F(SynapticonSystemInterfaceTest, RefusalsAccumulateAcrossCyclesAndResetOnActivation)
{
  const auto handle = system_.get_state_interface_handle("joint_1/status_word");
  {
    const std::unique_lock<std::shared_mutex> hold(handle->get_mutex());
    cycle();
    cycle();
    cycle();
  }
  EXPECT_EQ(system_.handle_access_failures().state_write, 3u);

  ASSERT_EQ(system_.on_deactivate(lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active")),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(system_.on_activate(lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive")),
            CallbackReturn::SUCCESS);
  EXPECT_EQ(system_.handle_access_failures().total(), 0u);
}

// ---------------------------------------------------------------------------
// Transmissions
// ---------------------------------------------------------------------------

/// The component brought up on the transmission robot.
class SynapticonSystemInterfaceTransmissionTest : public SynapticonSystemInterfaceTest
{
protected:
  hardware_interface::HardwareInfo hardware_info() const override
  {
    return make_transmission_hardware_info();
  }

  /// The value of a double state interface.
  double state(const std::string& interface) const
  {
    return system_.get_state_interface_handle(interface)->get_optional<double>().value();
  }
};

/// The HAL sees only the drive-backed joint, not the transmission output.
TEST_F(SynapticonSystemInterfaceTransmissionTest, OnlyTheDriveBackedJointReachesTheClient)
{
  cycle();
  EXPECT_EQ(system_.client->last_commands.size(), 1u);
  EXPECT_EQ(system_.client->joint_count(), 1u);
}

/// An output joint command reaches the HAL in actuator space.
TEST_F(SynapticonSystemInterfaceTransmissionTest, TheJointCommandIsMappedToTheActuatorDrive)
{
  ASSERT_TRUE(system_.get_command_interface_handle("output/position")->set_value(0.5));
  cycle();
  ASSERT_EQ(system_.client->last_commands.size(), 1u);
  EXPECT_NEAR(system_.client->last_commands[0].position, (0.5 - OFFSET) * REDUCTION, 1e-9);
}

/// A transmission that maps no gains leaves the drive with the values of its own joint.
TEST(SynapticonSystemInterfaceCommandDefaults, ADriveBehindATransmissionGetsItsOwnJointsValues)
{
  auto info = make_transmission_hardware_info();
  info.joints[0].parameters = { { "stiffness", "0.4" }, { "damping", "0.1" }, { "max_joint_torque", "5.0" } };
  info.joints[1].parameters = { { "stiffness", "1.5" }, { "damping", "0.3" }, { "max_joint_torque", "2.0" } };
  RecordingSystem system;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(system.on_init(params), CallbackReturn::SUCCESS);
  system.on_export_state_interfaces();
  system.on_export_command_interfaces();
  ASSERT_EQ(
      system.on_configure(lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured")),
      CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive")),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
  const DriveCommand& motor = system.client->last_commands.at(0);
  EXPECT_DOUBLE_EQ(motor.stiffness, 1.5);
  EXPECT_DOUBLE_EQ(motor.damping, 0.3);
  EXPECT_DOUBLE_EQ(motor.max_joint_torque, 5.0) << "the torque limit comes from the output joint";
  system.on_deactivate(lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"));
}

/// The component with a stiffness command on joint_1, and command defaults on joint_1 only.
class SynapticonSystemInterfaceCommandDefaultsTest : public SynapticonSystemInterfaceTest
{
protected:
  hardware_interface::HardwareInfo hardware_info() const override
  {
    auto info = make_hardware_info();
    info.joints[0].command_interfaces.push_back(interface_info("stiffness"));
    info.joints[0].parameters = { { "stiffness", "0.4" }, { "damping", "0.1" }, { "max_joint_torque", "5.0" } };
    return info;
  }
};

/// A stiffness left out gets the default, also right after a command that gave one. A joint without defaults keeps
/// NaN, so the HAL leaves the drive's value.
TEST_F(SynapticonSystemInterfaceCommandDefaultsTest, AMissingValueGetsTheDefault)
{
  cycle();
  EXPECT_DOUBLE_EQ(system_.client->last_commands[0].stiffness, 0.4);
  EXPECT_DOUBLE_EQ(system_.client->last_commands[0].damping, 0.1);
  EXPECT_DOUBLE_EQ(system_.client->last_commands[0].max_joint_torque, 5.0);
  EXPECT_TRUE(std::isnan(system_.client->last_commands[1].stiffness));

  const auto stiffness = system_.get_command_interface_handle("joint_1/stiffness");
  ASSERT_TRUE(stiffness->set_value(2.0));
  cycle();
  EXPECT_DOUBLE_EQ(system_.client->last_commands[0].stiffness, 2.0);
  ASSERT_TRUE(stiffness->set_value(std::numeric_limits<double>::quiet_NaN()));
  cycle();
  EXPECT_DOUBLE_EQ(system_.client->last_commands[0].stiffness, 0.4);
}

/// The actuator state the HAL reports is mapped back to the output joint.
TEST_F(SynapticonSystemInterfaceTransmissionTest, TheActuatorStateIsMappedBackToTheJoint)
{
  ASSERT_TRUE(system_.get_command_interface_handle("output/position")->set_value(0.5));
  cycle();  // the mock echoes the motor command as the motor state
  cycle();  // which the next read maps back through the transmission
  EXPECT_NEAR(state("motor/position"), (0.5 - OFFSET) * REDUCTION, 1e-9);
  EXPECT_NEAR(state("output/position"), 0.5, 1e-9);
}

// ---------------------------------------------------------------------------
// Lifecycle failures
// ---------------------------------------------------------------------------

/// The HAL step that fails in a lifecycle failure test.
enum class FailingStep
{
  SETUP,
  BRING_UP,
  START,
  FIRST_READ,
  LATER_READ,
  DISABLE,
};

/// A mock HAL that fails one step of the lifecycle.
class FailingMockHal : public MockHal
{
public:
  explicit FailingMockHal(FailingStep step) : step_(step)
  {
  }

  bool setup(const synapticon_hardware::HalConfig& config) override
  {
    return step_ != FailingStep::SETUP && MockHal::setup(config);
  }
  bool connect() override
  {
    return step_ != FailingStep::BRING_UP && MockHal::connect();
  }
  bool start() override
  {
    return step_ != FailingStep::START && MockHal::start();
  }
  bool read(std::vector<synapticon_hardware::DriveState>& states) override
  {
    ++reads_;
    const bool fails = (step_ == FailingStep::FIRST_READ) || (step_ == FailingStep::LATER_READ && reads_ > 1);
    return !fails && MockHal::read(states);
  }
  bool disable() override
  {
    return MockHal::disable() && step_ != FailingStep::DISABLE;
  }

private:
  FailingStep step_;
  int reads_{ 0 };
};

class FailingSystem : public SynapticonSystemInterface
{
public:
  explicit FailingSystem(FailingStep step) : step_(step)
  {
  }

  /// The HAL of the last configuration, kept after the component dropped it.
  std::shared_ptr<FailingMockHal> hal;

protected:
  std::shared_ptr<SynapticonHalI> make_hal() override
  {
    hal = std::make_shared<FailingMockHal>(step_);
    return hal;
  }

private:
  FailingStep step_;
};

/// `system` initialised from `info`, with its interfaces exported.
void initialise(SynapticonSystemInterface& system, const hardware_interface::HardwareInfo& info)
{
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(system.on_init(params), CallbackReturn::SUCCESS);
  system.on_export_state_interfaces();
  system.on_export_command_interfaces();
}

const rclcpp_lifecycle::State UNCONFIGURED =
    lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
const rclcpp_lifecycle::State INACTIVE =
    lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
const rclcpp_lifecycle::State ACTIVE = lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

/// A HAL name that no installed plugin declares fails the configuration.
TEST(SynapticonSystemInterfaceLifecycle, AnUnknownHalFailsTheConfiguration)
{
  auto info = make_hardware_info();
  info.hardware_parameters["hal"] = "no_such_hal";
  SynapticonSystemInterface system;
  initialise(system, info);
  EXPECT_EQ(system.on_configure(UNCONFIGURED), CallbackReturn::ERROR);
}

TEST(SynapticonSystemInterfaceLifecycle, ActivationBeforeConfigurationFails)
{
  SynapticonSystemInterface system;
  initialise(system, make_hardware_info());
  EXPECT_EQ(system.on_activate(INACTIVE), CallbackReturn::ERROR);
}

TEST(SynapticonSystemInterfaceLifecycle, AFailedSetupFailsTheConfiguration)
{
  FailingSystem system(FailingStep::SETUP);
  initialise(system, make_hardware_info());
  EXPECT_EQ(system.on_configure(UNCONFIGURED), CallbackReturn::ERROR);
}

TEST(SynapticonSystemInterfaceLifecycle, AFailedBringUpFailsTheConfiguration)
{
  FailingSystem system(FailingStep::BRING_UP);
  initialise(system, make_hardware_info());
  EXPECT_EQ(system.on_configure(UNCONFIGURED), CallbackReturn::ERROR);
}

TEST(SynapticonSystemInterfaceLifecycle, AFailedStartFailsTheActivation)
{
  FailingSystem system(FailingStep::START);
  initialise(system, make_hardware_info());
  ASSERT_EQ(system.on_configure(UNCONFIGURED), CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_activate(INACTIVE), CallbackReturn::ERROR);
}

/// Activation reads once to seed the commands from the measured positions.
TEST(SynapticonSystemInterfaceLifecycle, AFailedFirstReadFailsTheActivation)
{
  FailingSystem system(FailingStep::FIRST_READ);
  initialise(system, make_hardware_info());
  ASSERT_EQ(system.on_configure(UNCONFIGURED), CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_activate(INACTIVE), CallbackReturn::ERROR);
}

TEST(SynapticonSystemInterfaceLifecycle, AFailedCyclicReadIsAnError)
{
  FailingSystem system(FailingStep::LATER_READ);
  initialise(system, make_hardware_info());
  ASSERT_EQ(system.on_configure(UNCONFIGURED), CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(INACTIVE), CallbackReturn::SUCCESS);
  EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 0)), hardware_interface::return_type::ERROR);
}

/// A failed disable() is an error of the deactivation, which still stops the exchange.
TEST(SynapticonSystemInterfaceLifecycle, AFailedDisableIsAnErrorOfTheDeactivation)
{
  FailingSystem system(FailingStep::DISABLE);
  initialise(system, make_hardware_info());
  ASSERT_EQ(system.on_configure(UNCONFIGURED), CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(INACTIVE), CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_deactivate(ACTIVE), CallbackReturn::ERROR);
  EXPECT_FALSE(system.hal->is_streaming());
}

/// The error handling releases the HAL, so that the hardware can be configured again.
TEST(SynapticonSystemInterfaceLifecycle, TheErrorHandlingReleasesTheHal)
{
  FailingSystem system(FailingStep::DISABLE);
  initialise(system, make_hardware_info());
  ASSERT_EQ(system.on_configure(UNCONFIGURED), CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(INACTIVE), CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_deactivate(ACTIVE), CallbackReturn::ERROR);
  const auto released = system.hal;
  EXPECT_EQ(system.on_error(ACTIVE), CallbackReturn::SUCCESS);
  EXPECT_FALSE(released->is_connected());
  EXPECT_EQ(system.on_configure(UNCONFIGURED), CallbackReturn::SUCCESS);
  EXPECT_NE(system.hal, released);
}

/// An interface declared with another data_type than the hardware provides fails the configuration.
TEST(SynapticonSystemInterfaceLifecycle, AnInterfaceOfTheWrongTypeFailsTheConfiguration)
{
  auto info = make_hardware_info();
  for (auto& state : info.joints[0].state_interfaces)
  {
    if (state.name == "status_word")
    {
      state.data_type = "double";
    }
  }
  SynapticonSystemInterface system;
  initialise(system, info);
  EXPECT_EQ(system.on_configure(UNCONFIGURED), CallbackReturn::ERROR);
}

/// The command seeds written at activation are best effort: a locked command handle does not stop the activation.
TEST(SynapticonSystemInterfaceLifecycle, ALockedCommandHandleDoesNotStopTheActivation)
{
  SynapticonSystemInterface system;
  initialise(system, make_hardware_info());
  ASSERT_EQ(system.on_configure(UNCONFIGURED), CallbackReturn::SUCCESS);
  const auto position = system.get_command_interface_handle("joint_1/position");
  const auto action = system.get_command_interface_handle("joint_1/drive_action");
  {
    const std::unique_lock<std::shared_mutex> hold_position(position->get_mutex());
    const std::unique_lock<std::shared_mutex> hold_action(action->get_mutex());
    EXPECT_EQ(system.on_activate(INACTIVE), CallbackReturn::SUCCESS);
  }
  system.on_deactivate(lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"));
}

// ---------------------------------------------------------------------------
// Hold, released interfaces and shutdown
// ---------------------------------------------------------------------------

/// Without a position command, a joint holds where it was when its drive got power, however far it sags. Before
/// that, the hold follows the measurement, so a joint moved by hand does not jump back.
TEST_F(SynapticonSystemInterfaceTest, AJointWithoutACommandHoldsWhereItGotPower)
{
  system_.client->status_words = { SWITCH_ON_DISABLED, SWITCH_ON_DISABLED };
  system_.client->positions = { 1.0, 0.0 };
  cycle();
  EXPECT_DOUBLE_EQ(system_.client->last_commands.at(0).position, 1.0);
  system_.client->positions[0] = 1.2;  // moved by hand
  cycle();
  EXPECT_DOUBLE_EQ(system_.client->last_commands.at(0).position, 1.2) << "no jump back when the power comes";
  system_.client->status_words[0] = OPERATION_ENABLED;
  for (int i = 0; i < 50; ++i)
  {
    system_.client->positions[0] -= 0.01;  // sags under load
    cycle();
    EXPECT_DOUBLE_EQ(system_.client->last_commands.at(0).position, 1.2) << "cycle " << i;
  }
}

/// A position command sets where a powered joint holds once the commands stop.
TEST_F(SynapticonSystemInterfaceTest, APoweredJointHoldsItsLastTarget)
{
  system_.client->status_words = { OPERATION_ENABLED, OPERATION_ENABLED };
  system_.client->positions = { 1.0, 0.0 };
  const auto position = system_.get_command_interface_handle("joint_1/position");
  ASSERT_TRUE(position->set_value(0.6));
  cycle();
  ASSERT_TRUE(position->set_value(std::numeric_limits<double>::quiet_NaN()));
  system_.client->positions[0] = 0.4;
  cycle();
  EXPECT_DOUBLE_EQ(system_.client->last_commands.at(0).position, 0.6);
}

/// A joint behind a transmission holds through its motor: the motor's command stays while the motor sags.
TEST_F(SynapticonSystemInterfaceTransmissionTest, AJointBehindATransmissionHoldsWithItsMotorsPower)
{
  system_.client->status_words = { SWITCH_ON_DISABLED };
  system_.client->positions = { 2.0 };
  cycle();
  system_.client->status_words[0] = OPERATION_ENABLED;
  cycle();
  const double held = system_.client->last_commands.at(0).position;
  EXPECT_NEAR(held, 2.0, 1e-9) << "the hold of the measured position from before the power came";
  for (int i = 0; i < 20; ++i)
  {
    system_.client->positions[0] -= 0.05;
    cycle();
    EXPECT_NEAR(system_.client->last_commands.at(0).position, held, 1e-9) << "cycle " << i;
  }
}

}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
