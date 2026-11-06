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

#ifndef SIMPORT_CORE__CLOCK_HPP_
#define SIMPORT_CORE__CLOCK_HPP_

#include <memory>
#include <stop_token>
#include <string>
#include <utility>

#include "simport_core/config.hpp"
#include "simport_core/logger.hpp"
#include "simport_core/types.hpp"

namespace simport
{

/// The device's notion of time. All Timestamps a device handles come from its Clock.
///
/// Implementations must be thread-safe. Timed triggers call sleep_until() from their own
/// threads, so implementations must not depend on the ROS executor to wake sleepers: with
/// simulation time, /clock may tick more slowly than a device sends, and sleepers must still
/// wake at sub-tick resolution.
class Clock
{
public:
  virtual ~Clock() = default;

  /// Parse configuration. Called once, before any other method.
  virtual void configure(const Config & config, Logger & logger)
  {
    (void)logger;
    config.expect_keys({"plugin"});
  }

  /// Current time.
  virtual Timestamp now() const = 0;

  /// Block until now() >= `deadline` or a stop is requested on `stop`.
  /// Returns true if the deadline was reached, false if stopped.
  virtual bool sleep_until(Timestamp deadline, std::stop_token stop) = 0;

  /// True if time is driven by a simulator rather than the system clock.
  virtual bool is_simulated() const {return false;}

  /// Short description for logs, e.g. "system" or "ros (sim time)".
  virtual std::string describe() const = 0;

  bool sleep_for(Duration duration, std::stop_token stop)
  {
    return sleep_until(now() + duration, std::move(stop));
  }
};

using ClockPtr = std::shared_ptr<Clock>;

/// Wall-clock time (std::chrono::system_clock, i.e. Unix time).
class SystemClock : public Clock
{
public:
  Timestamp now() const override;
  bool sleep_until(Timestamp deadline, std::stop_token stop) override;
  std::string describe() const override {return "system";}
};

}  // namespace simport

#endif  // SIMPORT_CORE__CLOCK_HPP_
