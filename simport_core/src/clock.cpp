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

#include "simport_core/clock.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace simport
{

Timestamp SystemClock::now() const
{
  return std::chrono::duration_cast<Timestamp>(
    std::chrono::system_clock::now().time_since_epoch());
}

bool SystemClock::sleep_until(Timestamp deadline, std::stop_token stop)
{
  std::mutex mutex;
  std::condition_variable_any cv;
  std::unique_lock<std::mutex> lock(mutex);
  // Wait in relative steps and re-check, so system clock adjustments are tolerated.
  for (;; ) {
    const Duration remaining = deadline - now();
    if (remaining <= Duration::zero()) {
      return true;
    }
    if (stop.stop_requested()) {
      return false;
    }
    cv.wait_for(lock, stop, remaining, [] {return false;});
  }
}

}  // namespace simport
