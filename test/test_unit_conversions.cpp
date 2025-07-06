#include <gtest/gtest.h>
#include <cmath>

#include "synapticon_ros2_control/unit_conversions.hpp"

class UnitConversionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
  }

  void TearDown() override
  {
  }
};

TEST_F(UnitConversionTest, ZeroTicksReturnsZero)
{
  // Test case: ticks of 0 --> output is 0
  const int32_t ticks = 0;
  const double mechanical_reduction = 170.0;
  const uint32_t encoder_resolution = 2560;
  
  const double result = synapticon_ros2_control::input_ticks_to_output_shaft_rad(
    ticks, mechanical_reduction, encoder_resolution);
  
  EXPECT_DOUBLE_EQ(result, 0.0) 
    << "Expected 0.0 for zero ticks, but got " << result;
}

TEST_F(UnitConversionTest, PositiveTicksReturnsCorrectRadians)
{
  // Test case: ticks of 2560, mechanical reduction of 170, encoder resolution of 2560 --> output is 2*pi / 170
  const int32_t ticks = 2560;
  const double mechanical_reduction = 170.0;
  const uint32_t encoder_resolution = 2560;
  const double expected_radians = 2.0 * M_PI / 170.0;
  
  const double result = synapticon_ros2_control::input_ticks_to_output_shaft_rad(
    ticks, mechanical_reduction, encoder_resolution);
  
  EXPECT_NEAR(result, expected_radians, 1e-10) 
    << "Expected " << expected_radians << " radians for " << ticks 
    << " ticks, but got " << result;
}

TEST_F(UnitConversionTest, NegativeTicksReturnsCorrectRadians)
{
  // Test case: ticks of -2560, mechanical reduction of 170, encoder resolution of 2560 --> output is -2*pi / 170
  const int32_t ticks = -2560;
  const double mechanical_reduction = 170.0;
  const uint32_t encoder_resolution = 2560;
  const double expected_radians = -2.0 * M_PI / 170.0;
  
  const double result = synapticon_ros2_control::input_ticks_to_output_shaft_rad(
    ticks, mechanical_reduction, encoder_resolution);
  
  EXPECT_NEAR(result, expected_radians, 1e-10) 
    << "Expected " << expected_radians << " radians for " << ticks 
    << " ticks, but got " << result;
}

TEST_F(UnitConversionTest, ZeroRadiansReturnsZeroTicks)
{
  // Test case: output shaft radians of 0 --> output is 0 ticks
  const double output_shaft_rad = 0.0;
  const double mechanical_reduction = 170.0;
  const uint32_t encoder_resolution = 2560;
  
  const int32_t result = synapticon_ros2_control::output_shaft_rad_to_input_ticks(
    output_shaft_rad, mechanical_reduction, encoder_resolution);
  
  EXPECT_EQ(result, 0) 
    << "Expected 0 ticks for zero radians, but got " << result;
}

TEST_F(UnitConversionTest, PositiveRadiansReturnsCorrectTicks)
{
  // Test case: output shaft radians of 2*pi/170, mechanical reduction of 170, encoder resolution of 2560 --> output is 2560 ticks
  const double output_shaft_rad = 2.0 * M_PI / 170.0;
  const double mechanical_reduction = 170.0;
  const uint32_t encoder_resolution = 2560;
  const int32_t expected_ticks = 2560;
  
  const int32_t result = synapticon_ros2_control::output_shaft_rad_to_input_ticks(
    output_shaft_rad, mechanical_reduction, encoder_resolution);
  
  EXPECT_EQ(result, expected_ticks) 
    << "Expected " << expected_ticks << " ticks for " << output_shaft_rad 
    << " radians, but got " << result;
}

TEST_F(UnitConversionTest, NegativeRadiansReturnsCorrectTicks)
{
  // Test case: output shaft radians of -2*pi/170, mechanical reduction of 170, encoder resolution of 2560 --> output is -2560 ticks
  const double output_shaft_rad = -2.0 * M_PI / 170.0;
  const double mechanical_reduction = 170.0;
  const uint32_t encoder_resolution = 2560;
  const int32_t expected_ticks = -2560;
  
  const int32_t result = synapticon_ros2_control::output_shaft_rad_to_input_ticks(
    output_shaft_rad, mechanical_reduction, encoder_resolution);
  
  EXPECT_EQ(result, expected_ticks) 
    << "Expected " << expected_ticks << " ticks for " << output_shaft_rad 
    << " radians, but got " << result;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
} 