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

#ifndef SIMPORT_ROS__ROS_CLOCK_HPP_
#define SIMPORT_ROS__ROS_CLOCK_HPP_

#include <mutex>
#include <stop_token>
#include <string>
#include <utility>

#include "rclcpp/clock.hpp"
#include "simport_core/clock.hpp"
#include "simport_core/time_extrapolator.hpp"

namespace simport
{

/// The node's ROS clock: simulation time when `use_sim_time` is true, system time otherwise.
///
/// With sim time, /clock arrives in discrete ticks; now() smooths it with a TimeExtrapolator
/// (advancing at the measured real-time factor between ticks), so timed triggers emit evenly
/// even when /clock is slower than the device rate. sleep_until() sleeps on the steady clock,
/// scaled by the real-time factor, and re-checks at least every millisecond.
class RosClock : public Clock
{
public:
  explicit RosClock(rclcpp::Clock::SharedPtr clock)
  : clock_(std::move(clock)) {}

  Timestamp now() const override;
  bool sleep_until(Timestamp deadline, std::stop_token stop) override;
  bool is_simulated() const override;
  std::string describe() const override;

  /// Estimated real-time factor (1 when not simulated).
  double real_time_factor() const;

private:
  rclcpp::Clock::SharedPtr clock_;
  mutable std::mutex mutex_;
  mutable TimeExtrapolator extrapolator_;
};

}  // namespace simport

#endif  // SIMPORT_ROS__ROS_CLOCK_HPP_
