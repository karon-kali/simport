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

#include "simport_plugins/fixed_rate.hpp"

#include <cmath>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace simport::plugins
{

namespace
{
/// Falling this many periods behind (or ahead, after a backwards time jump) resets the schedule.
constexpr int kResyncPeriods = 4;
}  // namespace

FixedRate::~FixedRate()
{
  stop();
}

void FixedRate::configure(const Config & config, Context & context)
{
  config.expect_keys({"plugin", "rate_hz", "stale_timeout"});
  context_.emplace(context);
  const double rate = config.get<double>("rate_hz");
  if (!(rate > 0.0) || !std::isfinite(rate) || rate > 1e6) {
    config["rate_hz"].error("rate_hz must be in (0, 1e6]");
  }
  period_ = Duration(static_cast<Duration::rep>(std::llround(1e9 / rate)));
  const double stale = config.get<double>("stale_timeout", 1.0);
  if (!(stale >= 0.0) || !std::isfinite(stale)) {
    config["stale_timeout"].error("stale_timeout must be >= 0 seconds");
  }
  stale_timeout_ = Duration(static_cast<Duration::rep>(std::llround(stale * 1e9)));
}

void FixedRate::start(EmitCallback emit)
{
  stop();
  emit_ = std::move(emit);
  thread_ = std::jthread([this](std::stop_token st) {run(st);});
}

void FixedRate::stop()
{
  if (thread_.joinable()) {
    thread_.request_stop();
    thread_.join();
  }
  emit_ = nullptr;
}

void FixedRate::on_record(RecordPtr record)
{
  const Timestamp now = context_->clock().now();
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = std::move(record);
  arrival_ = now;
}

void FixedRate::run(std::stop_token stop)
{
  Clock & clock = context_->clock();
  Timestamp next = clock.now() + period_;
  bool was_stale = false;
  while (clock.sleep_until(next, stop)) {
    ++ticks_;
    RecordPtr record;
    Timestamp arrival;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      record = latest_;
      arrival = arrival_;
    }
    const Timestamp now = clock.now();
    if (record) {
      const bool stale = stale_timeout_ > Duration::zero() && now - arrival > stale_timeout_;
      if (stale) {
        ++stale_;
        if (!was_stale) {
          context_->logger().warn(
            "no new data for {:.2f} s; output paused until data resumes",
            static_cast<double>(stale_timeout_.count()) * 1e-9);
        }
      } else {
        if (was_stale) {
          context_->logger().info("data resumed; output running");
        }
        emit_(record);
        ++emitted_;
      }
      was_stale = stale;
    }

    next += period_;
    if (now - next > kResyncPeriods * period_ || next - now > kResyncPeriods * period_) {
      // Fell far behind (overloaded, or simulation time jumped forward) or time went backwards
      // (simulation reset): restart the schedule instead of bursting or stalling.
      ++overruns_;
      next = now + period_;
    }
  }
}

FixedRate::Stats FixedRate::stats() const
{
  return Stats{ticks_, emitted_, stale_, overruns_};
}

}  // namespace simport::plugins

PLUGINLIB_EXPORT_CLASS(simport::plugins::FixedRate, simport::Trigger)
