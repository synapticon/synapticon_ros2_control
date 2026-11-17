// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Collects the rclcpp log messages of a test, to check what was logged.

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include <rcutils/logging.h>

namespace synapticon_hardware::test
{

/// @brief While it exists, every log message goes into messages() instead of the console. One at a time.
class LogCapture
{
public:
  LogCapture()
  {
    if (rcutils_logging_initialize() != RCUTILS_RET_OK)
    {
      throw std::runtime_error("the rcutils logging could not be initialised");
    }
    previous_ = rcutils_logging_get_output_handler();
    messages().clear();
    rcutils_logging_set_output_handler(&LogCapture::collect);
  }
  ~LogCapture()
  {
    rcutils_logging_set_output_handler(previous_);
  }
  LogCapture(const LogCapture&) = delete;
  LogCapture& operator=(const LogCapture&) = delete;

  /// The messages logged so far, formatted.
  static std::vector<std::string>& messages()
  {
    static std::vector<std::string> collected;
    return collected;
  }

  /// How many messages contain `text`.
  static std::size_t count(const std::string& text)
  {
    const auto& all = messages();
    return static_cast<std::size_t>(
        std::count_if(all.begin(), all.end(), [&](const std::string& m) { return m.find(text) != std::string::npos; }));
  }

private:
  static void collect(const rcutils_log_location_t* /*location*/, int /*severity*/, const char* /*name*/,
                      rcutils_time_point_value_t /*timestamp*/, const char* format, va_list* args)
  {
    va_list copy;
    va_copy(copy, *args);
    char text[2048];
    std::vsnprintf(text, sizeof(text), format, copy);
    va_end(copy);
    messages().emplace_back(text);
  }

  rcutils_logging_output_handler_t previous_{ nullptr };
};

}  // namespace synapticon_hardware::test
