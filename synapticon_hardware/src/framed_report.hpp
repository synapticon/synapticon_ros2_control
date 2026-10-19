// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief A framed block of log text for a problem the user has to act on, so that it stands out in launch output.

#include <string>
#include <vector>

namespace synapticon_hardware
{

/// @brief A rule, the title after "!! ", a rule, the text and a closing rule.
class FramedReport
{
public:
  explicit FramedReport(const std::string& title) : text_("\n" + rule() + "\n!! " + title + "\n" + rule() + "\n")
  {
  }

  /// Add lines of text.
  FramedReport& say(const std::string& lines)
  {
    text_ += lines;
    return *this;
  }

  /// Add a paragraph "label: name, name, ..." when `names` is not empty.
  FramedReport& list(const std::string& label, const std::vector<std::string>& names)
  {
    if (names.empty())
    {
      return *this;
    }
    text_ += "\n" + label + ": ";
    for (std::size_t i = 0; i < names.size(); ++i)
    {
      text_ += (i == 0 ? "" : ", ") + names[i];
    }
    text_ += "\n";
    return *this;
  }

  /// The finished block.
  std::string close() const
  {
    return text_ + rule();
  }

private:
  static std::string rule()
  {
    return std::string(78, '-');
  }

  std::string text_;
};

}  // namespace synapticon_hardware
