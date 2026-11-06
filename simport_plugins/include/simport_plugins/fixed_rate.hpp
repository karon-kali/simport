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

#ifndef SIMPORT_PLUGINS__FIXED_RATE_HPP_
#define SIMPORT_PLUGINS__FIXED_RATE_HPP_

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>

#include "simport_core/trigger.hpp"

namespace simport::plugins
{

/// Emits the latest source record at the device's own rate, independent of how often the
/// simulator publishes, the way a real sensor samples on its internal clock. Timing follows
/// the device clock, so it slows down, speeds up and pauses with the simulation.
///
/// ```yaml
/// trigger:
///   plugin: simport/FixedRate
///   rate_hz: 200
///   stale_timeout: 1.0   # s; stop emitting when no new data arrived for this long
///                        # (0 = keep repeating the last sample forever)
/// ```
///
/// Nothing is emitted before the first record arrives. Fresh timestamps come from `$time_*`
/// builtins, which are evaluated per emitted frame.
class FixedRate : public Trigger
{
public:
  ~FixedRate() override;

  void configure(const Config & config, Context & context) override;
  void start(EmitCallback emit) override;
  void stop() override;
  void on_record(RecordPtr record) override;

  struct Stats
  {
    std::uint64_t ticks = 0;
    std::uint64_t emitted = 0;
    std::uint64_t stale = 0;     ///< ticks skipped because data was stale
    std::uint64_t overruns = 0;  ///< times the schedule fell behind and was reset
  };
  Stats stats() const;

private:
  void run(std::stop_token stop);

  std::optional<Context> context_;
  Duration period_{};
  Duration stale_timeout_{};

  EmitCallback emit_;
  std::jthread thread_;
  std::mutex mutex_;
  RecordPtr latest_;
  Timestamp arrival_{};

  std::atomic<std::uint64_t> ticks_{0}, emitted_{0}, stale_{0}, overruns_{0};
};

}  // namespace simport::plugins

#endif  // SIMPORT_PLUGINS__FIXED_RATE_HPP_
