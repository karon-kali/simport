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

#include <gtest/gtest.h>

#include <chrono>

#include "simport_core/time_extrapolator.hpp"

using namespace std::chrono_literals;

namespace simport
{
namespace
{

/// Simulated clock ticking every `tick` of sim time at real-time factor `rtf`, observed every
/// millisecond of wall time.
struct Sim
{
  Duration tick = 10ms;
  double rtf = 1.0;
  Duration wall{};

  Timestamp raw() const
  {
    const auto sim = Duration(static_cast<Duration::rep>(static_cast<double>(wall.count()) * rtf));
    return Timestamp((sim / tick) * tick);
  }
};

TEST(TimeExtrapolator, SmoothsStepsAtRealTime)
{
  TimeExtrapolator x;
  Sim sim;
  Timestamp prev{};
  double max_error_ms = 0;
  for (int ms = 0; ms < 2000; ++ms) {
    sim.wall = Duration(std::chrono::milliseconds(ms));
    const Timestamp out = x.update(sim.raw(), sim.wall);
    EXPECT_GE(out, prev);
    prev = out;
    if (ms > 500) {  // after the estimate settles
      const double err = std::abs(static_cast<double>((out - sim.wall).count())) / 1e6;
      max_error_ms = std::max(max_error_ms, err);
    }
  }
  EXPECT_NEAR(x.rate(), 1.0, 0.02);
  EXPECT_LT(max_error_ms, 1.5);  // vs. up to 10 ms for the raw stepwise clock
}

TEST(TimeExtrapolator, TracksSlowSimulation)
{
  TimeExtrapolator x;
  Sim sim;
  sim.rtf = 0.5;
  for (int ms = 0; ms < 3000; ++ms) {
    sim.wall = Duration(std::chrono::milliseconds(ms));
    x.update(sim.raw(), sim.wall);
  }
  EXPECT_NEAR(x.rate(), 0.5, 0.02);
}

TEST(TimeExtrapolator, StopsWhenPausedAndResumesCleanly)
{
  TimeExtrapolator x;
  Sim sim;
  Duration wall{};
  for (; wall < 1s; wall += 1ms) {
    sim.wall = wall;
    x.update(sim.raw(), wall);
  }
  const Timestamp frozen_raw = sim.raw();
  Timestamp last{};
  for (Duration w = wall; w < wall + 500ms; w += 1ms) {  // paused: raw clock stays put
    last = x.update(frozen_raw, w);
  }
  // At most two tick intervals beyond the last tick.
  EXPECT_LE(last - frozen_raw, 21ms);
  // Resuming after a long pause must not wreck the rate estimate.
  x.update(frozen_raw + 10ms, wall + 500ms);
  EXPECT_NEAR(x.rate(), 1.0, 0.05);
}

TEST(TimeExtrapolator, PassesThroughBackwardJump)
{
  TimeExtrapolator x;
  x.update(Timestamp(5s), Duration(0ms));
  x.update(Timestamp(5s + 10ms), Duration(10ms));
  EXPECT_EQ(x.update(Timestamp(1s), Duration(20ms)), Timestamp(1s));
}

}  // namespace
}  // namespace simport
