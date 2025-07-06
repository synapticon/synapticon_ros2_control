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

#pragma once

#include <cstdint>
#include <cmath>

namespace synapticon_ros2_control {

/**
 * @brief Convert an incremental position command in input shaft encoder ticks to output shaft radians
 */
inline double input_ticks_to_output_shaft_rad(int32_t ticks, double mechanical_reduction, uint32_t encoder_resolution) {
  return (static_cast<double>(ticks) / encoder_resolution) * 2.0 * M_PI / mechanical_reduction;
}

/**
 * @brief Convert an incremental output shaft radian command to input shaft encoder ticks
 */
inline int32_t output_shaft_rad_to_input_ticks(double output_shaft_rad, double mechanical_reduction, uint32_t encoder_resolution) {
  return (output_shaft_rad * encoder_resolution * mechanical_reduction / (2.0 * M_PI));
}

} // namespace synapticon_ros2_control