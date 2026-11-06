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

#include "simport_core/time_extrapolator.hpp"

#include <algorithm>
#include <cmath>

namespace simport
{

namespace
{
constexpr double kSmoothing = 0.1;     // EWMA weight of a new sample
constexpr double kMinRate = 1e-3;
constexpr double kMaxRate = 1e3;
constexpr int kPauseFactor = 4;        // a gap this many tick intervals long is a pause
}  // namespace

void TimeExtrapolator::reset(Timestamp sim, Duration steady)
{
  have_tick_ = true;
  have_estimate_ = false;
  tick_sim_ = sim;
  tick_steady_ = steady;
  tick_interval_ = Duration::zero();
  last_out_ = sim;
}

Timestamp TimeExtrapolator::update(Timestamp sim, Duration steady)
{
  if (!have_tick_ || sim < tick_sim_) {
    reset(sim, steady);  // first reading, or the simulation was reset
    return sim;
  }

  if (sim > tick_sim_) {
    const Duration dt_wall = steady - tick_steady_;
    const Duration dt_sim = sim - tick_sim_;
    const bool paused = have_estimate_ && dt_wall > kPauseFactor * tick_interval_;
    if (dt_wall > Duration::zero() && !paused) {
      const double sample = std::clamp(
        static_cast<double>(dt_sim.count()) / static_cast<double>(dt_wall.count()),
        kMinRate, kMaxRate);
      if (!have_estimate_) {
        rate_ = sample;
        tick_interval_ = dt_wall;
        have_estimate_ = true;
      } else {
        rate_ += kSmoothing * (sample - rate_);
        tick_interval_ += Duration(
          std::llround(kSmoothing * static_cast<double>((dt_wall - tick_interval_).count())));
      }
    }
    tick_sim_ = sim;
    tick_steady_ = steady;
  }

  Timestamp out = tick_sim_;
  if (have_estimate_) {
    const Duration since = std::min(steady - tick_steady_, 2 * tick_interval_);
    out += Duration(std::llround(static_cast<double>(since.count()) * rate_));
  }
  out = std::max(out, last_out_);
  last_out_ = out;
  return out;
}

}  // namespace simport
