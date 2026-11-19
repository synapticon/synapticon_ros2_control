// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Reporting on /diagnostics from a controller's update(): every change, and the latest state once per period.

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/lock_free_queue.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

namespace synapticon_controllers
{

/// The parameter of a reporting controller that sets how often its changes go out, in seconds.
inline constexpr const char* CHECK_PERIOD_PARAMETER = "diagnostics_check_period";
inline constexpr double DEFAULT_CHECK_PERIOD_S = 0.02;

/// The check period that `node` sets, or nothing after logging when it is not positive.
template <typename NodeT>
std::optional<std::chrono::nanoseconds> configured_check_period(const NodeT& node)
{
  const double seconds = node->get_parameter(CHECK_PERIOD_PARAMETER).as_double();
  if (!(seconds > 0.0))
  {
    RCLCPP_ERROR(node->get_logger(), "Parameter %s must be positive, got %g.", CHECK_PERIOD_PARAMETER, seconds);
    return std::nullopt;
  }
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(seconds));
}

/// @brief Publishes /diagnostics for a controller, from what its update() reads.
///
/// A controller has one report per thing it watches, for example one per joint. Each holds an `Item`, the reading
/// of one cycle. The control loop never waits:
/// - It queues every change of a report with the time of its cycle. A timer outside the loop publishes the queued
///   changes in order, each as a message stamped with that time, every check period. A full queue counts the change
///   as dropped, and the report then shows the count under `changes_dropped`.
/// - It hands over the latest item of every report. A diagnostic_updater reports them once per its period
///   (`diagnostic_updater.period`, one second by default), so a late listener gets the whole picture. A cycle that
///   meets a report reading them skips the hand-over.
///
/// Reports are named after what they describe, behind the controller's node name, as the updater names its tasks,
/// for example "drive_controller: joint_1". Before the first item, the describe function gets none. While stopped,
/// there are no reports.
template <typename Item>
class RealtimeDiagnostics
{
public:
  using Status = diagnostic_updater::DiagnosticStatusWrapper;
  /// Describes `item` of report `index` in `status`. An empty item means no reading yet.
  using Describe = std::function<void(std::size_t index, const std::optional<Item>& item, Status& status)>;
  using Latest = std::vector<std::optional<Item>>;
  /// Called for every change after its /diagnostics message, in order, outside the control loop.
  using OnChange = std::function<void(std::size_t index, const rclcpp::Time& time, const Item& item)>;

  /// Changes that one check period can hold. A healthy controller queues a few at most.
  static constexpr std::size_t QUEUE_CAPACITY = 1024;

  /// Create the updater and the publisher on `node`, unless they exist, and check for changes every `check_period`.
  /// Call outside the control loop.
  template <typename NodeT>
  void create(NodeT node, std::chrono::nanoseconds check_period)
  {
    if (!updater_)
    {
      updater_ =
          std::make_unique<diagnostic_updater::Updater>(node, 1.0, diagnostic_msgs::msg::DiagnosticStatus::STALE);
      updater_->setHardwareID("none");
      const bool full_name = node->get_parameter("diagnostic_updater.use_fqn").as_bool();
      name_prefix_ = std::string(full_name ? node->get_fully_qualified_name() : node->get_name()) + ": ";
      publisher_ = node->template create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
      queue_ = std::make_unique<Queue>(QUEUE_CAPACITY);
    }
    check_timer_ = node->create_wall_timer(check_period, [this]() { publish_changes(); });
  }

  /// Start reporting, with one report per name and no item yet. `on_change` may pass each change on further, for
  /// example to another topic. Call outside the control loop.
  void start(const std::vector<std::string>& names, Describe describe, OnChange on_change = {})
  {
    stop();
    std::lock_guard<std::mutex> guard(drain_mutex_);
    Event discarded;
    while (queue_->pop(discarded)) {}
    latest_.set(Latest(names.size()));
    dropped_ = std::make_unique<std::atomic<uint64_t>[]>(names.size());
    describe_ = std::move(describe);
    on_change_ = std::move(on_change);
    names_ = names;
    for (std::size_t index = 0; index < names.size(); ++index)
    {
      updater_->add(names[index], [this, index](Status& status) { this->describe(index, status); });
    }
  }

  /// Stop reporting. Call outside the control loop.
  void stop()
  {
    for (const std::string& name : names_)
    {
      updater_->removeByName(name);
    }
    std::lock_guard<std::mutex> guard(drain_mutex_);
    names_.clear();
  }

  /// From the control loop: queue `item` as a change of report `index` at `time`.
  void push_change(std::size_t index, const rclcpp::Time& time, const Item& item)
  {
    if (!queue_->push(Event{ index, time, item }))
    {
      dropped_[index].fetch_add(1, std::memory_order_relaxed);
    }
  }

  /// From the control loop: update the latest items in place with `fill`, which takes a Latest&.
  template <typename Fill>
  void update_latest(Fill&& fill)
  {
    // A reference capture keeps the std::function free of allocations.
    latest_.try_set([&fill](Latest& latest) { fill(latest); });
  }

  /// A copy of the latest items, outside the control loop.
  Latest latest() const
  {
    return latest_.get();
  }

  /// Describe the latest item of report `index`. The updater calls this, and tests may.
  void describe(std::size_t index, Status& status)
  {
    latest_.get([&](const Latest& latest) { describe_(index, latest[index], status); });
    add_dropped(index, status);
  }

  /// Publish the queued changes, in order. Returns how many. The check timer calls this, and tests may.
  std::size_t publish_changes()
  {
    std::lock_guard<std::mutex> guard(drain_mutex_);
    std::size_t count = 0;
    Event event;
    while (queue_->pop(event))
    {
      if (event.index >= names_.size())
      {
        continue;  // queued before a stop
      }
      diagnostic_msgs::msg::DiagnosticArray msg;
      msg.header.stamp = event.time;
      Status status;
      status.name = name_prefix_ + names_[event.index];
      status.hardware_id = "none";
      describe_(event.index, event.item, status);
      add_dropped(event.index, status);
      msg.status.push_back(status);
      publisher_->publish(msg);
      if (on_change_)
      {
        on_change_(event.index, event.time, event.item);
      }
      ++count;
    }
    return count;
  }

  /// How many changes of report `index` did not fit into the queue since the start.
  uint64_t dropped(std::size_t index) const
  {
    return dropped_[index].load(std::memory_order_relaxed);
  }

private:
  struct Event
  {
    std::size_t index{ 0 };
    rclcpp::Time time;
    Item item{};
  };
  using Queue = realtime_tools::LockFreeSPSCQueue<Event>;

  void add_dropped(std::size_t index, Status& status) const
  {
    if (const uint64_t count = dropped(index); count != 0)
    {
      status.add("changes_dropped", std::to_string(count));
    }
  }

  realtime_tools::RealtimeThreadSafeBox<Latest> latest_;
  std::unique_ptr<Queue> queue_;
  std::unique_ptr<std::atomic<uint64_t>[]> dropped_;
  Describe describe_;
  OnChange on_change_;
  std::vector<std::string> names_;
  std::string name_prefix_;
  /// Lets one side at a time take from the queue: the check timer, or start() and stop().
  std::mutex drain_mutex_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr check_timer_;
};

}  // namespace synapticon_controllers
