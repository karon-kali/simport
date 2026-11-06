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

#ifndef SIMPORT_CORE__TIME_EXTRAPOLATOR_HPP_
#define SIMPORT_CORE__TIME_EXTRAPOLATOR_HPP_

#include "simport_core/types.hpp"

namespace simport
{

/// Turns a stepwise simulated clock into a smooth one.
///
/// Simulators publish time in discrete ticks (e.g. /clock at 100 Hz), so a 200 Hz device
/// scheduled directly on it would emit in bursts. Between ticks this class advances time at
/// the observed real-time factor, but never by more than two tick intervals, so a paused
/// simulation stops time instead of running away. Output is monotonic, except that a backwards
/// jump of the input (simulation reset) is passed through.
///
/// Not thread-safe; callers serialize.
class TimeExtrapolator
{
public:
  /// `sim`: the current raw reading of the stepwise clock. `steady`: a monotonic wall-clock
  /// reading taken at the same moment. Returns the extrapolated simulation time.
  Timestamp update(Timestamp sim, Duration steady);

  /// Estimated real-time factor (simulated seconds per wall second); 1 until measured.
  double rate() const {return rate_;}

private:
  void reset(Timestamp sim, Duration steady);

  bool have_tick_ = false;
  bool have_estimate_ = false;
  Timestamp tick_sim_{};        ///< raw value at the last observed tick
  Duration tick_steady_{};      ///< when that tick was first observed
  Duration tick_interval_{};    ///< smoothed wall time between ticks
  double rate_ = 1.0;
  Timestamp last_out_{};
};

}  // namespace simport

#endif  // SIMPORT_CORE__TIME_EXTRAPOLATOR_HPP_
