// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Tests for how DriveController turns service requests and its fault reaction into drive_action writes.
///
/// Covers the single-cycle writes, writes the interface refuses (by holding its lock), the precedence of requests for
/// the same joint, the published drive status and diagnostics, and the drive fault reaction with its latch, clear and
/// events.

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gmock/gmock.h>

#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "synapticon_control_msgs/msg/drive_status_array.hpp"
#include "synapticon_control_msgs/msg/fault_reaction_event.hpp"
#include "synapticon_controllers/drive_controller.hpp"
#include "controller_test_utils.hpp"

namespace
{
namespace test = synapticon_controllers::test;
using synapticon_controllers::DriveAction;
using Names = std::vector<std::string>;
using ActionNames = synapticon_controllers::srv::SetDriveAction::Request;

constexpr const char* LEFT = "joint_1";
constexpr const char* RIGHT = "joint_2";
constexpr uint8_t OPERATION_ENABLED = 5;
constexpr uint8_t FAULT = 8;

/// The state interfaces DriveController claims per joint, with the types the hardware exports.
const std::vector<std::pair<const char*, const char*>> CLAIMED_STATES = {
  { "mode_of_operation", "int8" },  { "device_state", "uint8" },     { "status_word", "uint16" },
  { "error_code", "uint16" },       { "error_report", "double" },    { "slave_al_state", "uint8" },
  { "slave_online", "bool" },       { "dc_link_voltage", "double" }, { "temperature_inverter", "double" },
  { "temperature_core", "double" }, { "motor_current", "double" },
};

/// DriveController with its service callbacks and counters in reach.
class ProbedDriveController : public synapticon_controllers::DriveController
{
public:
  uint64_t refused_writes() const
  {
    return refused_writes_total_;
  }
  uint64_t overruled() const
  {
    return overruled_total_;
  }

  /// The diagnostics report of joint `index`, as the updater would publish it now.
  diagnostic_msgs::msg::DiagnosticStatus diagnostics(std::size_t index = 0)
  {
    diagnostic_updater::DiagnosticStatusWrapper status;
    diagnostics_.describe(index, status);
    return status;
  }

  /// Publish the queued diagnostics changes, as the check timer does. Returns how many there were.
  std::size_t publish_diagnostics_changes()
  {
    return diagnostics_.publish_changes();
  }

  /// The value of `key` in the diagnostics report of joint `index`, "missing" when it has none.
  std::string diagnostics_value(const std::string& key, std::size_t index = 0)
  {
    for (const auto& entry : diagnostics(index).values)
    {
      if (entry.key == key)
      {
        return entry.value;
      }
    }
    return "missing";
  }

  /// The common actions through ~/set_drive_action. An empty name list means every joint.
  bool enable(const Names& joints = {})
  {
    return act(ActionNames::ENABLE, joints);
  }
  bool disable(const Names& joints = {})
  {
    return act(ActionNames::DISABLE, joints);
  }
  bool reset_fault(const std::string& joint)
  {
    return act(ActionNames::RESET_FAULT, { joint });
  }
  /// The answer of ~/set_drive_action starts with "Refused: ..." when it refuses a request.
  bool act(const std::string& action, const Names& joints = {})
  {
    return call<synapticon_controllers::srv::SetDriveAction>(&ProbedDriveController::set_drive_action_callback,
                                                             [&](auto& r) {
                                                               r.action = action;
                                                               r.dof_names = joints;
                                                             })
        .success;
  }

  /// The two fault reaction services wait for update() to answer, see Bench::with_loop().
  bool clear(const Names& joints = {}, std::string* answer = nullptr)
  {
    const auto response = call<synapticon_controllers::srv::ClearFaultReaction>(
        &ProbedDriveController::clear_fault_reaction_callback, [&](auto& r) { r.dof_names = joints; });
    keep(answer, response.message);
    return response.success;
  }
  bool send_event(const std::string& event, const Names& joints = {}, std::string* answer = nullptr)
  {
    const auto response = call<synapticon_controllers::srv::InjectFaultEvent>(
        &ProbedDriveController::inject_fault_event_callback, [&](auto& r) {
          r.event = event;
          r.dof_names = joints;
        });
    keep(answer, response.message);
    return response.accepted;
  }

private:
  template <typename Srv, typename Callback, typename Fill>
  typename Srv::Response call(Callback callback, const Fill& fill)
  {
    auto request = std::make_shared<typename Srv::Request>();
    fill(*request);
    auto response = std::make_shared<typename Srv::Response>();
    (this->*callback)(request, response);
    return *response;
  }

  static void keep(std::string* answer, const std::string& message)
  {
    if (answer)
    {
      *answer = message;
    }
  }
};

/// The controller over `joints`, each with every claimed interface, brought up for each test.
class Bench : public ::testing::Test
{
protected:
  explicit Bench(Names joints, bool reaction = false) : joints_(std::move(joints)), reaction_(reaction)
  {
  }

  void SetUp() override
  {
    for (const auto& joint : joints_)
    {
      for (const auto& [name, type] : CLAIMED_STATES)
      {
        handles_.add_state(joint, name, type);
      }
      handles_.add_command(joint, "drive_action", "uint8");
    }
    test::bring_up(*dc_, "test_drive_controller",
                   { rclcpp::Parameter("joints", joints_), rclcpp::Parameter("fault_reaction.enabled", reaction_) },
                   handles_);
  }

  void TearDown() override
  {
    test::shut_down(*dc_);
  }

  void cycle(int count = 1)
  {
    for (int k = 0; k < count; ++k)
    {
      test::update(*dc_);
    }
  }

  /// Deactivate the controller, run `while_inactive`, and activate it again.
  void reactivate(const std::function<void()>& while_inactive = [] {})
  {
    test::shut_down(*dc_);
    while_inactive();
    ASSERT_EQ(test::activate(*dc_, handles_), controller_interface::CallbackReturn::SUCCESS);
  }

  /// Run a service call that waits for update(), cycling the controller until it returns.
  bool with_loop(const std::function<bool()>& call)
  {
    std::atomic<bool> finished{ false };
    bool answer = false;
    std::thread caller([&] {
      answer = call();
      finished = true;
    });
    while (!finished)
    {
      cycle();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    caller.join();
    return answer;
  }

  hardware_interface::StateInterface& state(const std::string& joint, const std::string& name)
  {
    for (const auto& handle : handles_.states)
    {
      if (handle->get_prefix_name() == joint && handle->get_interface_name() == name)
      {
        return *handle;
      }
    }
    throw std::runtime_error("no state interface " + joint + "/" + name);
  }

  hardware_interface::CommandInterface& action_handle(const std::string& joint = LEFT)
  {
    for (const auto& handle : handles_.commands)
    {
      if (handle->get_prefix_name() == joint)
      {
        return *handle;
      }
    }
    throw std::runtime_error("no drive_action interface on " + joint);
  }

  /// What the controller wrote to the joint's drive_action interface.
  DriveAction written(const std::string& joint = LEFT)
  {
    return static_cast<DriveAction>(action_handle(joint).get_optional<uint8_t>().value());
  }

  /// Holding the interface lock refuses the controller's write, as an asynchronous hardware component can.
  std::unique_lock<std::shared_mutex> lock_action(const std::string& joint = LEFT)
  {
    return std::unique_lock<std::shared_mutex>(action_handle(joint).get_mutex());
  }

  /// The joint online, in OP and in CiA 402 state `device_state`. A typed handle never set does not read as zero, so
  /// every one the state depends on is set.
  void report(const std::string& joint, uint8_t device_state)
  {
    ASSERT_TRUE(state(joint, "device_state").set_value<uint8_t>(device_state));
    ASSERT_TRUE(state(joint, "status_word").set_value<uint16_t>(0));
    ASSERT_TRUE(state(joint, "slave_online").set_value<bool>(true));
    ASSERT_TRUE(state(joint, "slave_al_state").set_value<uint8_t>(8));
  }

  Names joints_;
  bool reaction_;
  std::unique_ptr<ProbedDriveController> dc_ = std::make_unique<ProbedDriveController>();
  test::InterfaceHandles handles_;
};

class OneDrive : public Bench
{
protected:
  OneDrive() : Bench({ LEFT })
  {
  }
};

class TwoDrives : public Bench
{
protected:
  TwoDrives() : Bench({ LEFT, RIGHT })
  {
  }
};

/// Two enabled joints in csp with the fault reaction on. SetUp runs the first cycle, the reaction's baseline.
class Reacting : public Bench
{
protected:
  Reacting() : Bench({ LEFT, RIGHT }, true)
  {
  }

  void SetUp() override
  {
    Bench::SetUp();
    for (const auto& joint : joints_)
    {
      report(joint, OPERATION_ENABLED);
      ASSERT_TRUE(state(joint, "mode_of_operation").set_value<int8_t>(8));
    }
    cycle();
  }

  void fault(const std::string& joint = LEFT)
  {
    report(joint, FAULT);
    ASSERT_TRUE(state(joint, "error_code").set_value<uint16_t>(0x8611));
  }

  void expect_written(DriveAction left, DriveAction right)
  {
    EXPECT_EQ(written(LEFT), left);
    EXPECT_EQ(written(RIGHT), right);
  }
};

// Single-cycle writes and refused writes.

TEST_F(OneDrive, AWriteLastsOneCycle)
{
  dc_->enable();
  cycle();
  EXPECT_EQ(written(), DriveAction::ENABLE);
  cycle();
  EXPECT_EQ(written(), DriveAction::NONE);
  EXPECT_EQ(dc_->refused_writes(), 0u);
}

TEST_F(OneDrive, EachServiceWritesItsAction)
{
  dc_->disable();
  cycle();
  EXPECT_EQ(written(), DriveAction::DISABLE);
  cycle();
  dc_->enable();
  cycle();
  EXPECT_EQ(written(), DriveAction::ENABLE);
}

TEST_F(OneDrive, IdleCyclesWriteNone)
{
  cycle();
  EXPECT_EQ(written(), DriveAction::NONE);
  EXPECT_EQ(dc_->refused_writes(), 0u);
}

TEST_F(OneDrive, ARefusedWriteIsRetriedOnTheNextCycle)
{
  dc_->enable();
  {
    const auto held = lock_action();
    cycle();
    EXPECT_EQ(dc_->refused_writes(), 1u);
  }
  cycle();
  EXPECT_EQ(written(), DriveAction::ENABLE);
  EXPECT_EQ(dc_->refused_writes(), 1u);
  cycle();
  EXPECT_EQ(written(), DriveAction::NONE) << "the retried write lasts one cycle too";
}

TEST_F(OneDrive, RefusalsAddUpWhileTheInterfaceStaysLocked)
{
  dc_->enable();
  {
    const auto held = lock_action();
    cycle(3);
    EXPECT_EQ(dc_->refused_writes(), 3u);
  }
  cycle();
  EXPECT_EQ(written(), DriveAction::ENABLE);
}

TEST_F(OneDrive, ANewRequestWaitsBehindARefusedOne)
{
  dc_->enable();
  {
    const auto held = lock_action();
    cycle();
    dc_->reset_fault(LEFT);  // must not take the slot of the refused enable
    cycle();
    EXPECT_EQ(dc_->refused_writes(), 2u);
  }
  cycle();
  EXPECT_EQ(written(), DriveAction::ENABLE);
  cycle();
  EXPECT_EQ(written(), DriveAction::RESET_FAULT);
}

// Activation energises nothing.

TEST_F(OneDrive, AnInactiveControllerRefusesEveryRequest)
{
  reactivate([&] {
    EXPECT_FALSE(dc_->enable());
    EXPECT_FALSE(dc_->disable());
    EXPECT_FALSE(dc_->reset_fault(LEFT));
    EXPECT_FALSE(dc_->act("quick_stop"));
  });
  cycle();
  EXPECT_EQ(written(), DriveAction::NONE);
}

TEST_F(OneDrive, ARequestNotCarriedOutBeforeTheDeactivationIsDropped)
{
  dc_->enable();
  reactivate();
  cycle();
  EXPECT_EQ(written(), DriveAction::NONE);
}

TEST_F(OneDrive, ARefusedWriteIsNotRetriedAfterTheDeactivation)
{
  dc_->enable();
  {
    const auto held = lock_action();
    cycle();
  }
  reactivate();
  cycle();
  EXPECT_EQ(written(), DriveAction::NONE);
  dc_->enable();
  cycle();
  EXPECT_EQ(written(), DriveAction::ENABLE) << "a request after the activation is carried out";
}

// Precedence: requests for one joint in the same cycle.

struct Contest
{
  const char* name;
  std::vector<std::string> actions;  ///< Requested in this order, within one cycle.
  DriveAction winner;
};

class Precedence : public OneDrive, public ::testing::WithParamInterface<Contest>
{
protected:
  /// Each action keeps only its latest request, so the actions of a contest differ.
  bool request(const std::string& action)
  {
    return action == "reset_fault" ? dc_->reset_fault(LEFT) : dc_->act(action);
  }
};

TEST_P(Precedence, TheStrongerActionWinsAndTheOthersAreDropped)
{
  for (const auto& action : GetParam().actions)
  {
    ASSERT_TRUE(request(action));
  }
  cycle();
  EXPECT_EQ(written(), GetParam().winner);
  EXPECT_EQ(dc_->overruled(), GetParam().actions.size() - 1);
  // The losers are settled, not queued: a late enable after a disable would energise the drive again.
  cycle();
  EXPECT_EQ(written(), DriveAction::NONE);
  cycle();
  EXPECT_EQ(written(), DriveAction::NONE);
}

INSTANTIATE_TEST_SUITE_P(
    Pairs, Precedence,
    ::testing::Values(Contest{ "DisableOverEnable", { "enable", "disable" }, DriveAction::DISABLE },
                      Contest{ "DisableOverALaterEnable", { "disable", "enable" }, DriveAction::DISABLE },
                      Contest{ "EnableOverResetFault", { "reset_fault", "enable" }, DriveAction::ENABLE },
                      Contest{ "QuickStopOverEnable", { "enable", "quick_stop" }, DriveAction::QUICK_STOP },
                      Contest{ "DisableOverQuickStop", { "quick_stop", "disable" }, DriveAction::DISABLE },
                      Contest{ "DisableOverAll", { "reset_fault", "enable", "disable" }, DriveAction::DISABLE },
                      Contest{ "Alone", { "enable" }, DriveAction::ENABLE }),
    [](const auto& info) { return std::string(info.param.name); });

TEST_F(OneDrive, PrecedenceDoesNotDependOnTheOrder)
{
  dc_->enable();
  dc_->act(ActionNames::QUICK_STOP);
  dc_->reset_fault(LEFT);
  cycle();
  EXPECT_EQ(written(), DriveAction::QUICK_STOP);
  EXPECT_EQ(dc_->overruled(), 2u);
}

// The action names of set_drive_action.

class ForwardedAction : public OneDrive, public ::testing::WithParamInterface<std::pair<std::string, DriveAction>>
{
};

TEST_P(ForwardedAction, ReachesTheInterface)
{
  EXPECT_TRUE(dc_->act(GetParam().first));
  cycle();
  EXPECT_EQ(written(), GetParam().second);
}

INSTANTIATE_TEST_SUITE_P(Supported, ForwardedAction,
                         ::testing::Values(std::pair{ ActionNames::ENABLE, DriveAction::ENABLE },
                                           std::pair{ ActionNames::QUICK_STOP, DriveAction::QUICK_STOP },
                                           std::pair{ ActionNames::DISABLE, DriveAction::DISABLE },
                                           std::pair{ ActionNames::DISABLE_VOLTAGE, DriveAction::DISABLE_VOLTAGE },
                                           std::pair{ ActionNames::RESET_FAULT, DriveAction::RESET_FAULT }));

class RefusedAction : public OneDrive, public ::testing::WithParamInterface<std::string>
{
};

TEST_P(RefusedAction, ReachesNoDrive)
{
  EXPECT_FALSE(dc_->act(GetParam()));
  cycle();
  EXPECT_EQ(written(), DriveAction::NONE);
  cycle();
  EXPECT_EQ(written(), DriveAction::NONE);
}

// Known names without a drive walk behind them, and a name that is no action at all.
INSTANTIATE_TEST_SUITE_P(KnownOrNot, RefusedAction,
                         ::testing::Values(ActionNames::SWITCH_ON, ActionNames::SHUTDOWN, ActionNames::NONE,
                                           "quickstop"));

// Joint names.

TEST_F(TwoDrives, NoNameMeansEveryJoint)
{
  dc_->enable();
  cycle();
  EXPECT_EQ(written(LEFT), DriveAction::ENABLE);
  EXPECT_EQ(written(RIGHT), DriveAction::ENABLE);
}

TEST_F(TwoDrives, NamedJointsAreTheOnlyOnesWritten)
{
  dc_->enable({ RIGHT });
  dc_->reset_fault(LEFT);
  cycle();
  EXPECT_EQ(written(LEFT), DriveAction::RESET_FAULT);
  EXPECT_EQ(written(RIGHT), DriveAction::ENABLE);
  EXPECT_EQ(dc_->overruled(), 0u);
}

TEST_F(TwoDrives, ADifferentRequestPerJointMeansNoContest)
{
  dc_->enable({ LEFT });
  dc_->disable({ RIGHT });
  cycle();
  EXPECT_EQ(written(LEFT), DriveAction::ENABLE);
  EXPECT_EQ(written(RIGHT), DriveAction::DISABLE);
  EXPECT_EQ(dc_->overruled(), 0u);
}

TEST_F(TwoDrives, ARequestLosingOnOneJointStillActsOnTheOther)
{
  dc_->enable();
  dc_->disable({ RIGHT });
  cycle();
  EXPECT_EQ(written(LEFT), DriveAction::ENABLE);
  EXPECT_EQ(written(RIGHT), DriveAction::DISABLE);
  EXPECT_EQ(dc_->overruled(), 1u);
  // The enable is done with both joints: written on one, overruled on the other.
  cycle();
  EXPECT_EQ(written(LEFT), DriveAction::NONE);
  EXPECT_EQ(written(RIGHT), DriveAction::NONE);
}

TEST_F(TwoDrives, ARequestStaysUntilEveryJointHasWrittenIt)
{
  dc_->enable();
  {
    const auto held = lock_action(RIGHT);
    cycle();
    EXPECT_EQ(written(LEFT), DriveAction::ENABLE);
    EXPECT_EQ(dc_->refused_writes(), 1u);
  }
  cycle();
  EXPECT_EQ(written(RIGHT), DriveAction::ENABLE);
  cycle();
  EXPECT_EQ(written(LEFT), DriveAction::NONE);
  EXPECT_EQ(written(RIGHT), DriveAction::NONE);
}

TEST_F(TwoDrives, AnUnknownJointNameRejectsTheWholeRequest)
{
  EXPECT_FALSE(dc_->enable({ LEFT, "no_such_joint" }));
  cycle();
  EXPECT_EQ(written(LEFT), DriveAction::NONE);
  EXPECT_EQ(written(RIGHT), DriveAction::NONE);
}

TEST_F(TwoDrives, EveryServiceRejectsAnUnknownJoint)
{
  EXPECT_FALSE(dc_->reset_fault("no_such_joint"));
  EXPECT_FALSE(dc_->reset_fault(""));
  EXPECT_FALSE(dc_->disable({ "no_such_joint" }));
  EXPECT_FALSE(dc_->act(ActionNames::QUICK_STOP, { LEFT, "no_such_joint" }));
  cycle();
  EXPECT_EQ(written(LEFT), DriveAction::NONE);
  EXPECT_EQ(written(RIGHT), DriveAction::NONE);
}

// Setup failures.

TEST(DriveControllerSetup, AJointsParameterOfTheWrongTypeFailsTheInit)
{
  synapticon_controllers::DriveController dc;
  EXPECT_EQ(test::init(dc, "test_drive_controller", { rclcpp::Parameter("joints", 5) }),
            controller_interface::return_type::ERROR);
}

TEST(DriveControllerSetup, NoJointsFailTheConfiguration)
{
  synapticon_controllers::DriveController dc;
  ASSERT_EQ(test::init(dc, "test_drive_controller", {}), controller_interface::return_type::OK);
  EXPECT_EQ(dc.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::FAILURE);
}

/// A claimed interface the hardware does not provide fails the activation.
class MissingInterface : public ::testing::TestWithParam<const char*>
{
};

TEST_P(MissingInterface, FailsTheActivation)
{
  test::InterfaceHandles handles;
  for (const auto& [name, type] : CLAIMED_STATES)
  {
    if (std::string(name) != GetParam())
    {
      handles.add_state(LEFT, name, type);
    }
  }
  if (std::string("drive_action") != GetParam())
  {
    handles.add_command(LEFT, "drive_action", "uint8");
  }
  synapticon_controllers::DriveController dc;
  ASSERT_EQ(test::init(dc, "test_drive_controller", { rclcpp::Parameter("joints", Names{ LEFT }) }),
            controller_interface::return_type::OK);
  ASSERT_EQ(dc.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(test::activate(dc, handles), controller_interface::CallbackReturn::ERROR);
  dc.release_interfaces();
}

// A state slot, a diagnostics field and the command.
INSTANTIATE_TEST_SUITE_P(Each, MissingInterface, ::testing::Values("status_word", "motor_current", "drive_action"));

// The drive status topic.

TEST_F(OneDrive, TheDriveStatusCarriesTheDecodedState)
{
  using DriveStatusArray = synapticon_control_msgs::msg::DriveStatusArray;
  auto listener = std::make_shared<rclcpp::Node>("drive_status_listener");
  DriveStatusArray::SharedPtr latest;
  auto subscription = listener->create_subscription<DriveStatusArray>(
      "/test_drive_controller/drive_status", 10, [&latest](DriveStatusArray::SharedPtr msg) { latest = msg; });

  report(LEFT, FAULT);
  ASSERT_TRUE(state(LEFT, "error_code").set_value<uint16_t>(0x7121));
  ASSERT_TRUE(state(LEFT, "mode_of_operation").set_value<int8_t>(8));

  // The status goes out on a change, so the state toggles until the subscription has matched.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  const auto shows_fault = [&] { return latest && !latest->drives.empty() && latest->drives[0].device_state == FAULT; };
  for (uint8_t device_state = FAULT; !shows_fault() && std::chrono::steady_clock::now() < deadline; device_state ^= 1)
  {
    ASSERT_TRUE(state(LEFT, "device_state").set_value<uint8_t>(device_state));
    cycle();
    dc_->publish_diagnostics_changes();  // as the check timer does
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(shows_fault()) << "no drive status with the fault arrived";
  const auto& drive = latest->drives[0];
  EXPECT_EQ(drive.name, LEFT);
  EXPECT_TRUE(drive.online);
  EXPECT_EQ(drive.al_state, 8);
  EXPECT_EQ(drive.device_state_name, "Fault");
  EXPECT_EQ(drive.mode_of_operation, 8);
  EXPECT_EQ(drive.error_code, 0x7121);
  EXPECT_FALSE(drive.latched);
}

// The diagnostics.

TEST_F(OneDrive, BeforeTheFirstCycleTheDiagnosticsAreStale)
{
  EXPECT_EQ(dc_->diagnostics().level, diagnostic_msgs::msg::DiagnosticStatus::STALE);
  EXPECT_EQ(dc_->diagnostics().hardware_id, LEFT);
}

TEST_F(OneDrive, AnEnabledDriveIsOk)
{
  report(LEFT, OPERATION_ENABLED);
  ASSERT_TRUE(state(LEFT, "mode_of_operation").set_value<int8_t>(8));
  cycle();
  const auto status = dc_->diagnostics();
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "Operation Enabled");
  EXPECT_EQ(status.hardware_id, LEFT);
  EXPECT_EQ(dc_->diagnostics_value("online"), "yes");
  EXPECT_EQ(dc_->diagnostics_value("al_state"), "OP");
  EXPECT_EQ(dc_->diagnostics_value("mode_of_operation"), "CSP");
  EXPECT_EQ(dc_->diagnostics_value("error_code"), "missing") << "error data only counts in Fault";
}

TEST_F(OneDrive, AFaultIsAnErrorWithItsErrorCode)
{
  report(LEFT, FAULT);
  ASSERT_TRUE(state(LEFT, "error_code").set_value<uint16_t>(0x7121));
  cycle();
  EXPECT_EQ(dc_->diagnostics().level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(dc_->diagnostics().message, "Fault");
  EXPECT_EQ(dc_->diagnostics_value("error_code"), "0x7121");
  EXPECT_NE(dc_->diagnostics_value("error_report"), "missing");
}

TEST_F(OneDrive, ADriveOffTheBusIsDisconnectedWhateverItsLastData)
{
  report(LEFT, OPERATION_ENABLED);
  ASSERT_TRUE(state(LEFT, "slave_online").set_value<bool>(false));
  cycle();
  EXPECT_EQ(dc_->diagnostics().level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(dc_->diagnostics().message, "Disconnected");
  EXPECT_EQ(dc_->diagnostics_value("online"), "no");
}

TEST_F(OneDrive, EveryStateChangeIsQueuedOnce)
{
  report(LEFT, OPERATION_ENABLED);
  cycle(2);  // the first reading counts as a change, the second cycle changes nothing
  report(LEFT, FAULT);
  cycle();
  report(LEFT, OPERATION_ENABLED);
  cycle(3);
  EXPECT_EQ(dc_->publish_diagnostics_changes(), 3u);
  EXPECT_EQ(dc_->publish_diagnostics_changes(), 0u) << "each change goes out once";
}

TEST_F(TwoDrives, EachJointHasItsOwnReport)
{
  report(LEFT, OPERATION_ENABLED);
  report(RIGHT, FAULT);
  cycle();
  EXPECT_EQ(dc_->diagnostics(0).level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(dc_->diagnostics(1).level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(dc_->diagnostics(1).hardware_id, RIGHT);
}

TEST_F(OneDrive, EveryStateChangeReachesTheDriveStatusInOrder)
{
  using DriveStatusArray = synapticon_control_msgs::msg::DriveStatusArray;
  auto listener = std::make_shared<rclcpp::Node>("drive_status_order_listener");
  std::vector<uint8_t> states;
  auto subscription = listener->create_subscription<DriveStatusArray>(
      "/test_drive_controller/drive_status", 10,
      [&states](DriveStatusArray::SharedPtr msg) { states.push_back(msg->drives.at(0).device_state); });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (subscription->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(subscription->get_publisher_count(), 0u);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));  // until the publisher has matched as well

  // Three changes within three cycles, published together afterwards.
  for (const uint8_t device_state : { OPERATION_ENABLED, FAULT, OPERATION_ENABLED })
  {
    report(LEFT, device_state);
    cycle();
  }
  EXPECT_EQ(dc_->publish_diagnostics_changes(), 3u);
  while (states.size() < 3 && std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_THAT(states, ::testing::ElementsAre(OPERATION_ENABLED, FAULT, OPERATION_ENABLED));
}

// The drive fault reaction.

TEST_F(TwoDrives, WithoutTheReactionNothingReactsAndItsServicesRefuse)
{
  report(LEFT, OPERATION_ENABLED);
  report(RIGHT, OPERATION_ENABLED);
  cycle();
  report(LEFT, FAULT);
  cycle();
  EXPECT_EQ(written(LEFT), DriveAction::NONE);
  EXPECT_EQ(written(RIGHT), DriveAction::NONE);
  EXPECT_TRUE(dc_->enable());
  EXPECT_FALSE(dc_->clear());
  EXPECT_FALSE(dc_->send_event("emergency_stop"));
}

TEST_F(Reacting, AFaultQuickStopsAllJointsOnceAndLatches)
{
  fault();
  cycle();
  expect_written(DriveAction::QUICK_STOP, DriveAction::QUICK_STOP);
  cycle();
  expect_written(DriveAction::NONE, DriveAction::NONE);
  cycle();
  EXPECT_EQ(written(LEFT), DriveAction::NONE) << "staying in Fault is no new event";
}

TEST_F(Reacting, ALostDriveQuickStopsAllJoints)
{
  ASSERT_TRUE(state(RIGHT, "slave_online").set_value<bool>(false));
  cycle();
  expect_written(DriveAction::QUICK_STOP, DriveAction::QUICK_STOP);
}

TEST_F(Reacting, AFollowingErrorCountsOnlyInThePositionModes)
{
  ASSERT_TRUE(state(LEFT, "mode_of_operation").set_value<int8_t>(-6));
  ASSERT_TRUE(state(LEFT, "status_word").set_value<uint16_t>(0x2237));  // bit 13
  cycle();
  EXPECT_EQ(written(LEFT), DriveAction::NONE) << "bit 13 has another meaning outside pp and csp";
  ASSERT_TRUE(state(LEFT, "mode_of_operation").set_value<int8_t>(8));
  cycle();
  expect_written(DriveAction::QUICK_STOP, DriveAction::QUICK_STOP);
}

TEST_F(Reacting, ThePendingEnableLosesToTheQuickStop)
{
  EXPECT_TRUE(dc_->enable());
  fault();
  cycle();
  expect_written(DriveAction::QUICK_STOP, DriveAction::QUICK_STOP);
  cycle();
  expect_written(DriveAction::NONE, DriveAction::NONE);
}

/// An enable whose write the interface refused is not written later, once a fault latches its joint.
TEST_F(Reacting, ARefusedEnableIsDroppedWhenTheReactionLatches)
{
  EXPECT_TRUE(dc_->enable({ LEFT }));
  {
    const auto held = lock_action(LEFT);
    cycle();  // refused, the enable waits for its retry
  }
  fault(RIGHT);
  cycle();
  EXPECT_EQ(written(LEFT), DriveAction::QUICK_STOP) << "the quick stop, not the retried enable";
  cycle(3);
  EXPECT_NE(written(LEFT), DriveAction::ENABLE);
}

TEST_F(Reacting, TheLatchRefusesEnablesOnly)
{
  fault();
  cycle(2);
  EXPECT_FALSE(dc_->enable());
  EXPECT_FALSE(dc_->act("enable", { RIGHT }));
  EXPECT_TRUE(dc_->reset_fault(LEFT));
  cycle();
  EXPECT_EQ(written(LEFT), DriveAction::RESET_FAULT);
  EXPECT_TRUE(dc_->disable());
  cycle();
  EXPECT_EQ(written(RIGHT), DriveAction::DISABLE);
}

TEST_F(Reacting, AfterAClearEnablesPassAgain)
{
  fault();
  cycle(2);
  EXPECT_TRUE(with_loop([&] { return dc_->clear(); }));
  EXPECT_TRUE(dc_->enable({ RIGHT }));
  cycle();
  expect_written(DriveAction::NONE, DriveAction::ENABLE);
}

TEST_F(Reacting, AClearCanNameJoints)
{
  fault();
  cycle(2);
  EXPECT_TRUE(with_loop([&] { return dc_->clear({ RIGHT }); }));
  EXPECT_TRUE(dc_->enable({ RIGHT }));
  EXPECT_FALSE(dc_->enable({ LEFT })) << "still latched";
  EXPECT_FALSE(dc_->clear({ "no_such_joint" })) << "refused without asking update()";
  std::string answer;
  EXPECT_TRUE(with_loop([&] { return dc_->clear({ RIGHT }, &answer); }));
  EXPECT_EQ(answer, "Cleared: nothing was latched");
}

TEST_F(Reacting, TheEventTopicShowsTheFiringAndTheClear)
{
  using Event = synapticon_control_msgs::msg::FaultReactionEvent;
  auto listener = std::make_shared<rclcpp::Node>("fault_reaction_listener");
  std::vector<Event::SharedPtr> events;
  auto subscription = listener->create_subscription<Event>("/test_drive_controller/fault_reaction",
                                                           rclcpp::QoS(10).reliable().transient_local(),
                                                           [&events](Event::SharedPtr msg) { events.push_back(msg); });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  const auto receive = [&](std::size_t count) {
    while (events.size() < count && std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(listener);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return events.size() >= count;
  };

  fault();
  cycle();
  ASSERT_TRUE(receive(1)) << "no event arrived";
  const Event& fired = *events.front();
  EXPECT_TRUE(fired.latched);
  EXPECT_THAT(fired.triggered_by, ::testing::ElementsAre(LEFT));
  EXPECT_THAT(fired.condition, ::testing::ElementsAre(Event::CONDITION_FAULT));
  EXPECT_THAT(fired.condition_name, ::testing::ElementsAre("fault"));
  EXPECT_THAT(fired.error_code, ::testing::ElementsAre(0x8611));
  EXPECT_EQ(fired.error_report.size(), 1u);
  EXPECT_EQ(fired.action, "quick_stop");
  EXPECT_THAT(fired.acted_on, ::testing::ElementsAre(LEFT, RIGHT));
  EXPECT_THAT(fired.latched_joints, ::testing::ElementsAre(LEFT, RIGHT));

  ASSERT_TRUE(with_loop([&] { return dc_->clear(); }));
  ASSERT_TRUE(receive(2)) << "no clear event arrived";
  const Event& cleared = *events.back();
  EXPECT_FALSE(cleared.latched);
  EXPECT_TRUE(cleared.triggered_by.empty());
  EXPECT_TRUE(cleared.action.empty());
  EXPECT_TRUE(cleared.latched_joints.empty());
}

TEST_F(Reacting, AnEmergencyStopLatchesAllJoints)
{
  std::string answer;
  EXPECT_TRUE(with_loop([&] { return dc_->send_event("emergency_stop", {}, &answer); })) << answer;
  EXPECT_EQ(answer, "The fault policy accepted event 'emergency_stop'");
  EXPECT_FALSE(dc_->enable());
  EXPECT_FALSE(dc_->enable({ RIGHT }));
  EXPECT_TRUE(with_loop([&] { return dc_->clear(); }));
  EXPECT_TRUE(dc_->enable());
}

TEST_F(Reacting, AnEmergencyStopForNamedJointsLatchesOnlyThem)
{
  EXPECT_TRUE(with_loop([&] { return dc_->send_event("emergency_stop", { RIGHT }); }));
  EXPECT_TRUE(dc_->enable({ LEFT }));
  EXPECT_FALSE(dc_->enable({ RIGHT }));
}

TEST_F(Reacting, OtherEventsAreRefusedWithThePolicysReason)
{
  std::string answer;
  EXPECT_FALSE(with_loop([&] { return dc_->send_event("coffee_break", {}, &answer); }));
  EXPECT_EQ(answer, "Refused: the built-in policy knows only the event 'emergency_stop'");
  EXPECT_TRUE(dc_->enable()) << "nothing latched";
  EXPECT_FALSE(dc_->send_event("", {}, &answer)) << "refused without asking update()";
  EXPECT_FALSE(dc_->send_event("emergency_stop", { "no_such_joint" }, &answer));
}

TEST_F(Reacting, TheLatchOutlastsADeactivation)
{
  fault();
  cycle(2);
  reactivate();
  cycle();
  EXPECT_FALSE(dc_->enable({ RIGHT })) << "only ~/clear_fault_reaction ends the latch";
  EXPECT_TRUE(with_loop([&] { return dc_->clear(); }));
  EXPECT_TRUE(dc_->enable({ RIGHT }));
}

TEST_F(Reacting, AJointThatFaultedWhileInactiveTriggersTheReactionOnActivation)
{
  reactivate([&] { fault(LEFT); });
  cycle();
  expect_written(DriveAction::QUICK_STOP, DriveAction::QUICK_STOP);
}

TEST_F(Reacting, AQueryTimesOutWhenUpdateDoesNotRun)
{
  std::string answer;
  EXPECT_FALSE(dc_->clear({}, &answer));
  EXPECT_EQ(answer, "Refused: no decision within 100 ms: is the controller active?");
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
