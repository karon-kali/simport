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

#include "simport_plugins/bandwidth.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace simport::plugins
{

Bandwidth::~Bandwidth()
{
  if (thread_.joinable()) {
    thread_.request_stop();
    thread_.join();
  }
}

void Bandwidth::configure(const Config & config, Context & context)
{
  config.expect_keys({"plugin", "bps", "bits_per_byte", "max_delay"});
  context_.emplace(context);
  config_ = config;
  if (config.has("bps")) {
    bps_ = config.get<double>("bps");
    if (!(*bps_ > 0.0)) {
      config["bps"].error("bps must be positive");
    }
  }
  if (config.has("bits_per_byte")) {
    bits_per_byte_ = config.get<double>("bits_per_byte");
    if (!(*bits_per_byte_ > 0.0)) {
      config["bits_per_byte"].error("bits_per_byte must be positive");
    }
  }
  const double max_delay = config.get<double>("max_delay", 0.1);
  if (!(max_delay >= 0.0)) {
    config["max_delay"].error("max_delay must be >= 0 seconds");
  }
  max_delay_ = std::chrono::nanoseconds(std::llround(max_delay * 1e9));
}

void Bandwidth::bind(const TransportCaps & caps)
{
  if (!bps_) {
    if (!caps.line_rate_bps) {
      config_.error("set 'bps': the transport does not declare a line rate");
    }
    bps_ = caps.line_rate_bps;
  }
  if (!bits_per_byte_) {
    bits_per_byte_ = caps.bits_per_byte;
  }
}

void Bandwidth::start()
{
  std::lock_guard<std::mutex> users_lock(users_mutex_);
  if (users_++ > 0) {
    return;  // shared by several channels and already running
  }
  line_free_ = std::chrono::steady_clock::now();
  thread_ = std::jthread([this](std::stop_token st) {run(st);});
}

void Bandwidth::stop()
{
  std::lock_guard<std::mutex> users_lock(users_mutex_);
  if (users_ == 0 || --users_ > 0) {
    return;
  }
  thread_.request_stop();
  thread_.join();
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.clear();
}

void Bandwidth::process(Frame && frame, const FrameSink & next)
{
  const auto airtime = std::chrono::nanoseconds(std::llround(
      static_cast<double>(frame.data.size()) * *bits_per_byte_ / *bps_ * 1e9));
  const SteadyTime now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const SteadyTime start = std::max(now, line_free_);
    const SteadyTime done = start + airtime;
    if (done - now > max_delay_ + airtime) {
      if (dropped_++ == 0) {
        context_->logger().warn(
          "output exceeds the line rate ({} bit/s); dropping frames like a transmit overrun",
          *bps_);
      }
      return;
    }
    line_free_ = done;
    queue_.push_back(Pending{std::move(frame), done, &next});
  }
  cv_.notify_one();
}

void Bandwidth::run(std::stop_token stop)
{
  std::unique_lock<std::mutex> lock(mutex_);
  while (!stop.stop_requested()) {
    if (queue_.empty()) {
      cv_.wait(lock, stop, [this] {return !queue_.empty();});
      continue;
    }
    const SteadyTime due = queue_.front().due;
    if (std::chrono::steady_clock::now() < due) {
      cv_.wait_until(lock, stop, due, [] {return false;});
      continue;
    }
    Pending p = std::move(queue_.front());
    queue_.pop_front();
    lock.unlock();
    (*p.sink)(std::move(p.frame));
    ++frames_;
    lock.lock();
  }
}

Bandwidth::Stats Bandwidth::stats() const
{
  return Stats{frames_, dropped_};
}

}  // namespace simport::plugins

PLUGINLIB_EXPORT_CLASS(simport::plugins::Bandwidth, simport::FrameFilter)
