#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <future>

#include "rclcpp/rclcpp.hpp"

#include <gtest/gtest.h>

constexpr double TEST_TIMEOUT_S = 25.0;
constexpr double CHECK_INTERVAL_S = 1.0;

class ControllerManagerIntegrationTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    // Initialize ROS2 if not already done
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("controller_manager_integration_test");

    // Give some time for the system to stabilize
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  void TearDown() override
  {
    node_.reset();
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  // Helper method to check if controller_manager node exists
  bool findControllerManagerNode()
  {
    auto node_names = node_->get_node_names();

    for (const auto& node_name : node_names)
    {
      if (node_name.find("controller_manager") != std::string::npos)
      {
        RCLCPP_INFO(node_->get_logger(), "Found controller_manager node: %s", node_name.c_str());
        return true;
      }
    }

    return false;
  }

  // Helper method to wait for controller_manager node with timeout
  bool waitForControllerManagerNode(double timeout_seconds = TEST_TIMEOUT_S)
  {
    auto start_time = node_->now();

    while (rclcpp::ok())
    {
      auto current_time = node_->now();
      auto elapsed = (current_time - start_time).seconds();

      if (elapsed > timeout_seconds)
      {
        RCLCPP_ERROR(node_->get_logger(), "Timeout: controller_manager node not found within %.1f seconds",
                     timeout_seconds);
        return false;
      }

      if (findControllerManagerNode())
      {
        return true;
      }

      RCLCPP_INFO(node_->get_logger(), "controller_manager node not found yet, checking again...");
      rclcpp::sleep_for(std::chrono::seconds(static_cast<int>(CHECK_INTERVAL_S)));
    }

    return false;
  }

protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(ControllerManagerIntegrationTestFixture, ControllerManagerNodeExists)
{
  // Test that controller_manager node is present in the system
  EXPECT_TRUE(waitForControllerManagerNode()) << "controller_manager node should be running in the system";
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
