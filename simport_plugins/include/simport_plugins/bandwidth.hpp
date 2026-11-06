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

#ifndef SIMPORT_PLUGINS__BANDWIDTH_HPP_
#define SIMPORT_PLUGINS__BANDWIDTH_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>

#include "simport_core/frame_filter.hpp"

namespace simport::plugins
{

/// Models a finite line rate: each frame is delivered when its last byte would have finished
/// transmitting, and frames that would have to wait longer than `max_delay` are dropped, like
/// a device's transmit buffer overrunning.
///
/// The device adds one in front of its port when the port has a line rate: a serial port's
/// `baud`, or `bps` on any port, with the port's `max_delay`. It is shared by all send messages
/// of the device, since they share one line.
///
/// Line rate is physical, so timing uses wall-clock (steady) time even under simulation.
class Bandwidth : public FrameFilter
{
public:
  ~Bandwidth() override;

  void configure(const Config & config, Context & context) override;
  void bind(const TransportCaps & caps) override;
  void start() override;
  void stop() override;
  void process(Frame && frame, const FrameSink & next) override;

  struct Stats
  {
    std::uint64_t frames = 0;
    std::uint64_t dropped = 0;
  };
  Stats stats() const;

private:
  using SteadyTime = std::chrono::steady_clock::time_point;
  struct Pending
  {
    Frame frame;
    SteadyTime due;
    const FrameSink * sink;
  };

  void run(std::stop_token stop);

  std::optional<Context> context_;
  Config config_;
  std::optional<double> bps_;
  std::optional<double> bits_per_byte_;
  std::chrono::nanoseconds max_delay_{};

  std::mutex mutex_;
  std::condition_variable_any cv_;
  std::deque<Pending> queue_;
  SteadyTime line_free_{};
  std::jthread thread_;
  std::mutex users_mutex_;
  int users_ = 0;

  std::atomic<std::uint64_t> frames_{0};
  std::atomic<std::uint64_t> dropped_{0};
};

}  // namespace simport::plugins

#endif  // SIMPORT_PLUGINS__BANDWIDTH_HPP_
