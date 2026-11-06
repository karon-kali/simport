// Copyright 2026 simport contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SIMPORT_ROS__ROS_LOGGER_HPP_
#define SIMPORT_ROS__ROS_LOGGER_HPP_

#include <string_view>
#include <utility>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/logging.h"
#include "simport_core/logger.hpp"

namespace simport
{

/// Forwards simport logging to an rclcpp logger.
class RosLogger : public Logger
{
public:
  explicit RosLogger(rclcpp::Logger logger)
  : logger_(std::move(logger)) {}

  void log(LogLevel level, std::string_view message) override
  {
    const int n = static_cast<int>(message.size());
    switch (level) {
      case LogLevel::kDebug:
        RCLCPP_DEBUG(logger_, "%.*s", n, message.data());
        break;
      case LogLevel::kInfo:
        RCLCPP_INFO(logger_, "%.*s", n, message.data());
        break;
      case LogLevel::kWarn:
        RCLCPP_WARN(logger_, "%.*s", n, message.data());
        break;
      case LogLevel::kError:
        RCLCPP_ERROR(logger_, "%.*s", n, message.data());
        break;
    }
  }

  bool enabled(LogLevel level) const override
  {
    int severity = RCUTILS_LOG_SEVERITY_INFO;
    switch (level) {
      case LogLevel::kDebug: severity = RCUTILS_LOG_SEVERITY_DEBUG; break;
      case LogLevel::kInfo: severity = RCUTILS_LOG_SEVERITY_INFO; break;
      case LogLevel::kWarn: severity = RCUTILS_LOG_SEVERITY_WARN; break;
      case LogLevel::kError: severity = RCUTILS_LOG_SEVERITY_ERROR; break;
    }
    return rcutils_logging_logger_is_enabled_for(logger_.get_name(), severity);
  }

private:
  rclcpp::Logger logger_;
};

}  // namespace simport

#endif  // SIMPORT_ROS__ROS_LOGGER_HPP_
