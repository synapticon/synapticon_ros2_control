// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Tests for RealtimeDiagnostics: which changes reach /diagnostics, when, in which order and under which name.

#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gmock/gmock.h>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "synapticon_controllers/realtime_diagnostics.hpp"

namespace
{
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using Diagnostics = synapticon_controllers::RealtimeDiagnostics<int>;
using namespace std::chrono_literals;

/// One message on /diagnostics: its stamp and its one report.
struct Seen
{
  rclcpp::Time stamp;
  DiagnosticStatus status;
};

/// A reporter on node "reporter" with an updater period far longer than any test, so only a change explains a
/// report. Its one report, "counter", shows the item as its message.
class Reporter : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({ rclcpp::Parameter("diagnostic_updater.period", 60.0) });
    node_ = std::make_shared<rclcpp::Node>("reporter", options);
    listener_ = std::make_shared<rclcpp::Node>("diagnostics_listener");
    subscription_ =
        listener_->create_subscription<DiagnosticArray>("/diagnostics", 100, [this](DiagnosticArray::SharedPtr msg) {
          for (const auto& status : msg->status)
          {
            if (status.name == "reporter: counter" && status.level != DiagnosticStatus::STALE)
            {
              seen_.push_back({ rclcpp::Time(msg->header.stamp), status });
            }
          }
        });
    executor_.add_node(node_);
    executor_.add_node(listener_);
  }

  void start(std::chrono::nanoseconds check_period)
  {
    diagnostics_.create(node_, check_period);
    diagnostics_.start({ "counter" }, [](std::size_t, const std::optional<int>& item, Diagnostics::Status& status) {
      status.summary(DiagnosticStatus::OK, item ? std::to_string(*item) : "none");
    });
    spin_for(300ms);  // until the subscription matches
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline)
    {
      executor_.spin_some(10ms);
    }
  }

  std::vector<std::string> messages() const
  {
    std::vector<std::string> result;
    for (const auto& seen : seen_)
    {
      result.push_back(seen.status.message);
    }
    return result;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> listener_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  Diagnostics diagnostics_;
  std::vector<Seen> seen_;
};

TEST_F(Reporter, EveryChangeIsPublishedInOrderWithItsOwnTime)
{
  start(20ms);
  diagnostics_.push_change(0, rclcpp::Time(10, 1), 1);
  diagnostics_.push_change(0, rclcpp::Time(10, 2), 2);
  diagnostics_.push_change(0, rclcpp::Time(10, 3), 3);
  spin_for(300ms);
  ASSERT_THAT(messages(), ::testing::ElementsAre("1", "2", "3"));
  EXPECT_EQ(seen_[0].stamp.nanoseconds(), rclcpp::Time(10, 1).nanoseconds());
  EXPECT_EQ(seen_[2].stamp.nanoseconds(), rclcpp::Time(10, 3).nanoseconds());
}

TEST_F(Reporter, ChangesWaitForTheCheckPeriod)
{
  start(2s);
  diagnostics_.push_change(0, rclcpp::Time(10, 0), 7);
  spin_for(300ms);
  EXPECT_THAT(messages(), ::testing::IsEmpty());
}

TEST_F(Reporter, TheLatestItemAloneWaitsForTheUpdaterPeriod)
{
  start(20ms);
  diagnostics_.update_latest([](auto& latest) { latest[0] = 7; });
  spin_for(300ms);
  EXPECT_THAT(messages(), ::testing::IsEmpty());
  Diagnostics::Status status;
  diagnostics_.describe(0, status);
  EXPECT_EQ(status.message, "7");
}

TEST_F(Reporter, BeforeTheFirstItemTheReportHasNone)
{
  start(20ms);
  Diagnostics::Status status;
  diagnostics_.describe(0, status);
  EXPECT_EQ(status.message, "none");
}

TEST_F(Reporter, AStoppedReporterPublishesNothing)
{
  start(20ms);
  diagnostics_.stop();
  diagnostics_.push_change(0, rclcpp::Time(10, 0), 7);
  spin_for(300ms);
  EXPECT_THAT(messages(), ::testing::IsEmpty());
}

TEST_F(Reporter, AFullQueueCountsTheChangesItDrops)
{
  start(1h);  // the test empties the queue itself
  const std::size_t extra = 5;
  for (std::size_t k = 0; k < Diagnostics::QUEUE_CAPACITY + extra; ++k)
  {
    diagnostics_.push_change(0, rclcpp::Time(10, 0), static_cast<int>(k));
  }
  EXPECT_EQ(diagnostics_.dropped(0), extra);
  EXPECT_EQ(diagnostics_.publish_changes(), Diagnostics::QUEUE_CAPACITY);
  Diagnostics::Status status;
  diagnostics_.describe(0, status);
  ASSERT_FALSE(status.values.empty());
  EXPECT_EQ(status.values.back().key, "changes_dropped");
  EXPECT_EQ(status.values.back().value, std::to_string(extra));
}

TEST(CheckPeriod, MustBePositive)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({ rclcpp::Parameter(synapticon_controllers::CHECK_PERIOD_PARAMETER, 0.0) });
  auto node = std::make_shared<rclcpp::Node>("zero_period", options);
  node->declare_parameter(synapticon_controllers::CHECK_PERIOD_PARAMETER, 0.02);
  EXPECT_FALSE(synapticon_controllers::configured_check_period(node));
  node->set_parameter(rclcpp::Parameter(synapticon_controllers::CHECK_PERIOD_PARAMETER, 0.005));
  EXPECT_EQ(synapticon_controllers::configured_check_period(node), std::chrono::nanoseconds(5'000'000));
}

}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
