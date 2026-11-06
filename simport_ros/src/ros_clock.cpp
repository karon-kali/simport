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

#include "simport_ros/ros_clock.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <format>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

namespace simport
{

Timestamp RosClock::now() const
{
  const Timestamp raw(clock_->now().nanoseconds());
  if (!is_simulated()) {
    return raw;
  }
  const Duration steady = std::chrono::duration_cast<Duration>(
    std::chrono::steady_clock::now().time_since_epoch());
  std::lock_guard<std::mutex> lock(mutex_);
  return extrapolator_.update(raw, steady);
}

bool RosClock::is_simulated() const
{
  return clock_->get_clock_type() == RCL_ROS_TIME && clock_->ros_time_is_active();
}

double RosClock::real_time_factor() const
{
  if (!is_simulated()) {
    return 1.0;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return extrapolator_.rate();
}

bool RosClock::sleep_until(Timestamp deadline, std::stop_token stop)
{
  std::mutex mutex;
  std::condition_variable_any cv;
  std::unique_lock<std::mutex> lock(mutex);
  for (;; ) {
    const Duration remaining = deadline - now();
    if (remaining <= Duration::zero()) {
      return true;
    }
    if (stop.stop_requested()) {
      return false;
    }
    Duration step = remaining;
    if (is_simulated()) {
      // Convert to wall time, and wake at least every millisecond to observe /clock ticks.
      const double rtf = std::max(real_time_factor(), 1e-3);
      step = std::min<Duration>(
        Duration(static_cast<Duration::rep>(static_cast<double>(remaining.count()) / rtf)), 1ms);
    }
    cv.wait_for(lock, stop, step, [] {return false;});
  }
}

std::string RosClock::describe() const
{
  return is_simulated() ? std::format("ros (sim time, rtf {:.2f})", real_time_factor()) :
         "ros (system time)";
}

}  // namespace simport
