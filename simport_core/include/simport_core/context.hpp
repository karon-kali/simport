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

#ifndef SIMPORT_CORE__CONTEXT_HPP_
#define SIMPORT_CORE__CONTEXT_HPP_

#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include "simport_core/clock.hpp"
#include "simport_core/logger.hpp"

namespace simport
{

/// Services available to a plugin. Passed to configure(); plugins may keep a copy.
class Context
{
public:
  Context(std::string device_name, LoggerPtr logger, ClockPtr clock)
  : device_name_(std::move(device_name)), scope_(device_name_),
    logger_(std::move(logger)), clock_(std::move(clock)) {}

  /// Name of the device this plugin belongs to.
  const std::string & device_name() const {return device_name_;}

  /// Slash-separated position in the device, e.g. `imu0/imu_data/codec`.
  const std::string & scope() const {return scope_;}

  Logger & logger() const {return *logger_;}
  const LoggerPtr & logger_ptr() const {return logger_;}

  Clock & clock() const {return *clock_;}
  const ClockPtr & clock_ptr() const {return clock_;}

  /// Context for a nested component: same clock, logger prefixed with the extended scope.
  Context child(std::string_view name) const
  {
    Context c = *this;
    c.scope_ = scope_ + "/" + std::string(name);
    c.logger_ = std::make_shared<ScopedLogger>(root_logger(), c.scope_);
    c.root_logger_ = root_logger();
    return c;
  }

private:
  const LoggerPtr & root_logger() const {return root_logger_ ? root_logger_ : logger_;}

  std::string device_name_;
  std::string scope_;
  LoggerPtr logger_;
  LoggerPtr root_logger_;
  ClockPtr clock_;
};

}  // namespace simport

#endif  // SIMPORT_CORE__CONTEXT_HPP_
