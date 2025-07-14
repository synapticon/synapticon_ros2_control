// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025 Synapticon GmbH
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <gtest/gtest.h>
#include <chrono>
#include <optional>

#include "synapticon_ros2_control/synapticon_interface.hpp"

namespace synapticon_ros2_control {

namespace
{
  constexpr double SPRING_ADJUST_MAX_TORQUE = 2500.0;
  constexpr double SPRING_ADJUST_MIN_TORQUE = 900.0;
}

class SpringAdjustPDControllersTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Initialize state with error_prev set to std::nullopt
    state_.error_prev_ = std::nullopt;
    state_.time_prev_ = std::chrono::steady_clock::now();
    allow_mode_change_ = false;
  }

  SpringAdjustState state_;
  bool allow_mode_change_;
};

TEST_F(SpringAdjustPDControllersTest, TestTargetEqualsCurrentPosition) {
  // Target position of 0.04 rad, current position of 0.04 rad
  // Output torque should be 900
  double target_position = 0.04;
  double current_position = 0.04;
  
  double result = spring_adjust_by_inertial_actuator_position(
    target_position, current_position, state_, allow_mode_change_);
  
  // When target equals current, error is 0, so torque should be MIN_TORQUE
  EXPECT_NEAR(result, 900, 1e-6);
  
  // Verify state was updated
  EXPECT_TRUE(state_.error_prev_.has_value());
  EXPECT_NEAR(state_.error_prev_.value(), 0.0, 1e-6);
}

TEST_F(SpringAdjustPDControllersTest, TestMaximumPositiveError) {
  // Target position of 0.04 rad, current position of 0 rad
  // This creates maximum positive error of 0.04 rad
  // Output torque should be 2500 (SPRING_ADJUST_MAX_TORQUE)
  double target_position = 0.04;
  double current_position = 0.0;
  
  double result = spring_adjust_by_inertial_actuator_position(
    target_position, current_position, state_, allow_mode_change_);
  
  // With maximum error of 0.04 rad, torque should be MAX_TORQUE
  EXPECT_NEAR(result, 2500, 1e-6);
  
  // Verify state was updated
  EXPECT_TRUE(state_.error_prev_.has_value());
  EXPECT_NEAR(state_.error_prev_.value(), 0.04, 1e-6);
}

TEST_F(SpringAdjustPDControllersTest, TestMaximumNegativeError) {
  // Target position of 0 rad, current position of 0.04 rad
  // This creates maximum negative error of -0.04 rad
  // Output torque should be -2500 (-SPRING_ADJUST_MAX_TORQUE)
  double target_position = 0.0;
  double current_position = 0.04;
  
  double result = spring_adjust_by_inertial_actuator_position(
    target_position, current_position, state_, allow_mode_change_);
  
  // With maximum negative error of -0.04 rad, torque should be -MAX_TORQUE
  EXPECT_NEAR(result, -2500, 1e-6);
  
  // Verify state was updated
  EXPECT_TRUE(state_.error_prev_.has_value());
  EXPECT_NEAR(state_.error_prev_.value(), -0.04, 1e-6);
}

TEST_F(SpringAdjustPDControllersTest, TestHalfError) {
  // Target position of 0.04 rad, current position of 0.02 rad
  // This creates error of 0.02 rad (half of maximum)
  double target_position = 0.04;
  double current_position = 0.02;
  
  double result = spring_adjust_by_inertial_actuator_position(
    target_position, current_position, state_, allow_mode_change_);
  
  // With half error, torque should be MIN_TORQUE + (MAX_TORQUE - MIN_TORQUE) / 2
  double expected_torque = SPRING_ADJUST_MIN_TORQUE + (SPRING_ADJUST_MAX_TORQUE - SPRING_ADJUST_MIN_TORQUE) / 2;
  EXPECT_NEAR(result, expected_torque, 1e-6);
}

TEST_F(SpringAdjustPDControllersTest, TestAllowModeChangeWhenStable) {
  // Test that allow_mode_change is set to true when error is small and stable
  double target_position = 0.04;
  double current_position = 0.04; // No error
  
  // First call to establish state
  spring_adjust_by_inertial_actuator_position(
    target_position, current_position, state_, allow_mode_change_);
  
  // Second call with small error and no derivative
  current_position = 0.039; // Small error of 0.001 rad
  double result = spring_adjust_by_inertial_actuator_position(
    target_position, current_position, state_, allow_mode_change_);
  
  // Should set torque to 0 and allow_mode_change to true
  EXPECT_NEAR(result, 0.0, 1e-6);
  EXPECT_TRUE(allow_mode_change_);
}

TEST_F(SpringAdjustPDControllersTest, TestDerivativeCalculation) {
  // Test that derivative calculation works correctly
  double target_position = 0.04;
  double current_position = 0.02;
  
  // First call
  spring_adjust_by_inertial_actuator_position(
    target_position, current_position, state_, allow_mode_change_);
  
  // Second call with different position to create derivative
  current_position = 0.03;
  double result = spring_adjust_by_inertial_actuator_position(
    target_position, current_position, state_, allow_mode_change_);
  
  // Should include derivative term in calculation
  // The exact value depends on timing, but it should be different from just proportional
  EXPECT_GT(result, 0.0);
  EXPECT_LE(result, SPRING_ADJUST_MAX_TORQUE);
}

} // namespace synapticon_ros2_control

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
