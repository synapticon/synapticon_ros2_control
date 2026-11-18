// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The controller that watches the drives, passes on drive actions and runs the fault reaction.

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "synapticon_control_msgs/msg/drive_status_array.hpp"
#include "synapticon_control_msgs/msg/fault_reaction_event.hpp"
#include "synapticon_control_msgs/srv/clear_fault_reaction.hpp"
#include "synapticon_control_msgs/srv/inject_fault_event.hpp"
#include "synapticon_control_msgs/srv/set_drive_action.hpp"
#include "synapticon_controllers/fault_reaction.hpp"
#include "synapticon_controllers/quick_stop_policy.hpp"
#include "synapticon_controllers/realtime_diagnostics.hpp"
#include "synapticon_hardware/drive_data.hpp"

namespace synapticon_controllers
{

using synapticon_hardware::DriveAction;

/// The services of the drive controller.
namespace srv = synapticon_control_msgs::srv;

/// What update() reports for a joint: its CiA 402 state, or that its drive is not on the bus in OP.
enum class ReportedState : uint8_t
{
  NOT_READY_TO_SWITCH_ON,
  SWITCH_ON_DISABLED,
  READY_TO_SWITCH_ON,
  SWITCHED_ON,
  OPERATION_ENABLED,
  QUICK_STOP_ACTIVE,
  FAULT_REACTION_ACTIVE,
  FAULT,
  UNDEFINED,
  DISCONNECTED,
};

/// The actions ~/set_drive_action takes. A waiting request is kept per action, so that requests for different
/// actions in the same cycle meet and the stronger one wins: a request to stop never loses to a request to run.
constexpr std::array<DriveAction, 5> REQUESTABLE_ACTIONS = { DriveAction::ENABLE, DriveAction::QUICK_STOP,
                                                             DriveAction::DISABLE, DriveAction::DISABLE_VOLTAGE,
                                                             DriveAction::RESET_FAULT };
constexpr std::size_t NUM_REQUEST_SLOTS = REQUESTABLE_ACTIONS.size();

/// A requested drive action, for the joints it names (all joints when it names none).
struct ActionRequest
{
  DriveAction action = DriveAction::NONE;
  std::vector<std::string> joints;
  uint64_t id = 0;  ///< Counts the requests of its slot, so that update() takes each one once.
  /// The id of the oldest request merged into this one. A joint that a request from first_id on is done with does
  /// not get the action again.
  uint64_t first_id = 0;
};

/// A question for the fault reaction that only update() can answer: release the latch, or apply a
/// named event of the policy. The service callback waits for the answer.
struct ReactionQuery
{
  enum class Kind : uint8_t
  {
    CLEAR,
    EVENT
  };
  Kind kind = Kind::CLEAR;
  std::string event;
  std::vector<std::string> joints;  ///< All joints when empty.
  uint64_t id = 0;
};

/// Where one joint's interfaces sit in the loaned interface vectors. Resolved by name at activation,
/// since the controller manager does not promise the order of the interface configuration.
struct JointSlots
{
  std::size_t device_state = 0;
  std::size_t status_word = 0;
  std::size_t mode_of_operation = 0;
  std::size_t error_code = 0;
  std::size_t error_report = 0;
  /// The master's view of the slave: a slave that drops off keeps its last PDO data, so only these
  /// two show a disconnect.
  std::size_t al_state = 0;
  std::size_t online = 0;
  /// dc_link_voltage, temperature_inverter, temperature_core and motor_current.
  std::array<std::size_t, 4> diagnostics{};
  std::size_t drive_action = 0;  ///< Command interface.
};

/// What update() saw of one joint, for its report on /diagnostics.
struct DriveReport
{
  ReportedState state = ReportedState::UNDEFINED;
  bool online = false;
  uint8_t al_state = 0;
  uint8_t device_state = 0;  ///< The CiA 402 state as the interface carries it.
  int8_t mode_of_operation = 0;
  uint16_t error_code = 0;
  double error_report = 0.0;  ///< As the interface carries it, decoded for the report.
  bool latched = false;       ///< The fault reaction holds this joint.
  std::array<double, 4> diagnostics{};
};

/// @brief Watches the drives, passes on drive actions, and runs the fault reaction.
///
/// - It reports every drive on ~/drive_status and /diagnostics: every change of a drive's state, within
///   `diagnostics_check_period`, and all drives once a second.
/// - It takes drive actions (enable, quick stop, ...) on ~/set_drive_action and writes them to the joints'
///   drive_action interfaces. When requests for one joint meet in a cycle, the stronger action wins.
/// - With fault_reaction.enabled, it stops the joints when a drive fails, until ~/clear_fault_reaction.
class DriveController : public controller_interface::ControllerInterface
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
  /// How many requests one cycle could not carry out, for the throttled reports.
  struct CycleRefusals
  {
    std::size_t refused_writes = 0;   ///< Writes the interface refused. They are retried.
    std::size_t overruled = 0;        ///< Requests a higher-precedence one replaced.
    std::size_t latched_enables = 0;  ///< Enables the latched fault reaction removed.
  };

  // Lifecycle and update steps.
  void create_services();
  /// Fill slots_. Returns false, after logging which, when a claimed interface is missing.
  bool resolve_slots();
  /// Build the fault reaction, unlatched. Returns false, after logging why, when the policy rejects the joints.
  bool create_reaction();
  /// Reset what the reaction keeps per activation. Its latch stays.
  void reset_reaction();
  /// Read every joint's reported state into current_states_.
  void read_reported_states();
  /// What update() hands over for the diagnostics of one joint.
  void fill_report(std::size_t joint, DriveReport& report) const;
  /// The diagnostics of one joint, outside the control loop. Stale before the first report.
  void describe_drive(const std::optional<DriveReport>& report, std::size_t joint,
                      diagnostic_updater::DiagnosticStatusWrapper& status) const;
  /// ~/drive_status with joint's entry from a change at `time`, outside the control loop.
  void publish_drive_status_change(std::size_t joint, const rclcpp::Time& time, const DriveReport& report);
  /// ~/drive_status with every drive's latest reading, after the queued changes. The heartbeat timer calls this.
  void publish_drive_status_heartbeat();
  /// Hand this cycle's service requests and reaction actions to the drive_action interfaces.
  void dispatch_actions();
  /// Write joint's unwritten action. A refused write keeps it unwritten, since the hardware acts on the change of
  /// value. Returns whether the write was refused.
  bool write_unwritten_action(std::size_t joint);
  void report_refusals(const CycleRefusals& refusals);

  // Status reading.
  ReportedState reported_state(std::size_t joint) const;
  /// The joint's condition for the fault reaction, from its reported state and the following error bit of the
  /// position modes.
  JointCondition condition_of(std::size_t joint, ReportedState state) const;
  void fill_status_entry(const DriveReport& report, synapticon_control_msgs::msg::DriveStatus& entry) const;
  std::string error_report_of(std::size_t joint) const;

  // Fault reaction.
  /// One cycle of the fault reaction: a pending query from the services, then the policy, then the latch flags.
  void step_reaction(const rclcpp::Time& time);
  void answer_reaction_query(const rclcpp::Time& time, const ReactionQuery& query);
  /// Publish an event (`fired` set) or a clear (`fired` null). It allocates nothing once the message is prepared.
  void publish_reaction_event(const rclcpp::Time& time, const ReactionEvent* fired);
  /// Size the event message and the spare strings for the joints, outside the control loop.
  void prepare_reaction_event_message();
  /// Hand a query to update() and wait for its answer. Returns whether it was accepted, with the reason in `reason`.
  bool ask_update_loop(ReactionQuery query, std::string& reason);
  /// The first of `joints` (all joints when empty) that the fault reaction holds latched.
  std::optional<std::string> first_latched_joint(const std::vector<std::string>& joints) const;

  // Service callbacks: validate, hand the request to update(), answer.
  std::optional<std::string> first_unknown_joint(const std::vector<std::string>& joints) const;
  void post_request(DriveAction action, std::vector<std::string> joints);
  void set_drive_action_callback(const std::shared_ptr<srv::SetDriveAction::Request> request,
                                 std::shared_ptr<srv::SetDriveAction::Response> response);
  void clear_fault_reaction_callback(const std::shared_ptr<srv::ClearFaultReaction::Request> request,
                                     std::shared_ptr<srv::ClearFaultReaction::Response> response);
  void inject_fault_event_callback(const std::shared_ptr<srv::InjectFaultEvent::Request> request,
                                   std::shared_ptr<srv::InjectFaultEvent::Response> response);

  // Joints and their interfaces, set up at configure and activation.
  std::vector<std::string> joint_names_;
  std::vector<JointSlots> slots_;
  /// The action each joint still has to write to its drive_action interface, NONE when there is none.
  std::vector<DriveAction> unwritten_actions_;

  // Requests of ~/set_drive_action, one slot per action, written by the callback and read by update(). update() never
  // clears a slot: it notes the id of each request it is done with, and a slot whose id it has noted holds nothing new.
  // A request for a slot that still holds a waiting one merges into it.
  std::array<realtime_tools::RealtimeBuffer<std::shared_ptr<ActionRequest>>, NUM_REQUEST_SLOTS> request_slots_;
  std::array<std::atomic<uint64_t>, NUM_REQUEST_SLOTS> posted_request_ids_{};    ///< by the callback
  std::array<std::atomic<uint64_t>, NUM_REQUEST_SLOTS> finished_request_ids_{};  ///< by update()
  /// Per slot and joint, the id of the last request that was done with the joint. Sized at activation.
  std::array<std::vector<uint64_t>, NUM_REQUEST_SLOTS> done_by_;
  /// One callback posts at a time, so that a merge sees the request it merges into.
  std::mutex post_mutex_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  /// Whether the controller is active, for the service callbacks: an inactive one takes no request.
  std::atomic<bool> active_{ false };

  // The fault reaction, opt-in through fault_reaction.enabled. It requests actions with the same precedence as the
  // services and stays latched until ~/clear_fault_reaction.
  bool reaction_enabled_ = false;
  std::unique_ptr<FaultReaction> reaction_;
  std::vector<JointCondition> joint_conditions_;
  /// The action the reaction wants per joint, NONE where none. Dropped once written or overruled.
  std::vector<DriveAction> reaction_wants_;
  /// Per joint, whether the reaction holds it latched. Written by update(), read by the service callbacks.
  std::unique_ptr<std::atomic<bool>[]> latched_flags_;
  /// The callback posts a query, update() answers it under the query's id, and the callback waits for that id.
  /// update() answers each id once, and a callback that gives up replaces its query with an empty one. The mutex lets
  /// one callback ask at a time.
  realtime_tools::RealtimeBuffer<std::shared_ptr<ReactionQuery>> reaction_inbox_;
  std::mutex reaction_call_mutex_;
  uint64_t last_query_id_ = 0;
  std::atomic<uint64_t> answered_query_id_{ 0 };
  std::atomic<bool> answer_accepted_{ false };
  std::atomic<const char*> answer_reason_{ "" };
  std::vector<std::size_t> queried_joints_;  ///< Reused by update(), reserved at activation.

  // Reports.
  std::vector<ReportedState> current_states_;
  std::vector<ReportedState> last_reported_states_;
  /// /diagnostics, one report per joint, and this cycle's reading of each joint for it. Sized at activation.
  RealtimeDiagnostics<DriveReport> diagnostics_;
  std::vector<DriveReport> reports_;
  using DriveStatusMsg = synapticon_control_msgs::msg::DriveStatusArray;
  using ReactionEventMsg = synapticon_control_msgs::msg::FaultReactionEvent;
  /// ~/drive_status, the per-joint status in machine-readable form. Its message is sized and named at activation,
  /// and filled outside the control loop.
  rclcpp::Publisher<DriveStatusMsg>::SharedPtr drive_status_pub_;
  rclcpp::TimerBase::SharedPtr drive_status_timer_;
  DriveStatusMsg drive_status_msg_;
  std::mutex drive_status_mutex_;
  /// ~/fault_reaction, transient local, so a late subscriber still gets the last event.
  rclcpp::Publisher<ReactionEventMsg>::SharedPtr reaction_event_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<ReactionEventMsg>> reaction_event_rt_;
  /// Strings with room for the longest name the event message carries. The message borrows them, so that filling it
  /// in update() allocates nothing.
  std::vector<std::string> spare_strings_;

  // Totals since activation, for the throttled reports.
  uint64_t refused_writes_total_ = 0;
  uint64_t overruled_total_ = 0;
  uint64_t latched_enables_total_ = 0;
  rclcpp::Clock throttle_clock_{ RCL_STEADY_TIME };
};

}  // namespace synapticon_controllers
