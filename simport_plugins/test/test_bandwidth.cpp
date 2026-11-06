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
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "simport_plugins/bandwidth.hpp"

using namespace std::chrono_literals;

namespace simport::plugins
{
namespace
{

using SteadyTime = std::chrono::steady_clock::time_point;

struct Harness
{
  Context ctx{"dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>()};
  Bandwidth filter;
  std::mutex mutex;
  std::vector<SteadyTime> delivered;
  FrameSink sink = [this](Frame &&) {
      std::lock_guard<std::mutex> lock(mutex);
      delivered.push_back(std::chrono::steady_clock::now());
    };

  Harness(const std::string & yaml, TransportCaps caps)
  {
    filter.configure(Config::from_string(yaml, "f.yaml"), ctx);
    filter.bind(caps);
    filter.start();
  }

  std::size_t count()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return delivered.size();
  }
};

TransportCaps serial(double baud)
{
  TransportCaps caps;
  caps.line_rate_bps = baud;
  caps.bits_per_byte = 10.0;
  return caps;
}

TEST(Bandwidth, PacesFramesAtLineRate)
{
  // 9600 baud 8N1 = 960 bytes/s; 48-byte frames take 50 ms each.
  Harness h("max_delay: 1.0\n", serial(9600));
  const SteadyTime t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < 5; ++i) {
    h.filter.process(Frame{Bytes(48, 0), {}, {}}, h.sink);
  }
  std::this_thread::sleep_for(450ms);
  h.filter.stop();
  ASSERT_EQ(h.count(), 5u);
  const auto last = h.delivered.back() - t0;
  EXPECT_GE(last, 245ms);  // 5 x 50 ms; never early
  EXPECT_LE(last, 400ms);  // may be late on a loaded CI runner
  const auto gap = h.delivered[2] - h.delivered[1];
  EXPECT_GE(gap, 30ms);
  EXPECT_LE(gap, 90ms);
}

TEST(Bandwidth, DropsWhenOverrun)
{
  // 10 x 50 ms of data offered at once with a 120 ms buffer: only ~3 fit.
  Harness h("max_delay: 0.12\n", serial(9600));
  for (int i = 0; i < 10; ++i) {
    h.filter.process(Frame{Bytes(48, 0), {}, {}}, h.sink);
  }
  std::this_thread::sleep_for(250ms);
  h.filter.stop();
  EXPECT_EQ(h.count(), 3u);
  EXPECT_EQ(h.filter.stats().dropped, 7u);
}

TEST(Bandwidth, SharedBetweenChannels)
{
  Harness h("max_delay: 1.0\n", serial(9600));
  h.filter.start();  // second user
  h.filter.stop();   // first user leaves: must keep running for the second
  h.filter.process(Frame{Bytes(10, 0), {}, {}}, h.sink);
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(h.count(), 1u);
  h.filter.stop();   // last user
  h.filter.stop();   // extra stop is harmless
}

TEST(Bandwidth, ExplicitRateAndErrors)
{
  Harness h("bps: 80000\nbits_per_byte: 8\n", TransportCaps{});  // 10 kB/s
  const SteadyTime t0 = std::chrono::steady_clock::now();
  h.filter.process(Frame{Bytes(500, 0), {}, {}}, h.sink);  // 50 ms
  std::this_thread::sleep_for(200ms);
  h.filter.stop();
  ASSERT_EQ(h.count(), 1u);
  EXPECT_GE(h.delivered[0] - t0, 45ms);

  Context ctx("dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>());
  Bandwidth b;
  b.configure(Config::from_string("max_delay: 0.1\n", "f.yaml"), ctx);
  EXPECT_THROW(b.bind(TransportCaps{}), ConfigError);  // no line rate anywhere
  EXPECT_THROW(b.configure(Config::from_string("bps: 0\n"), ctx), ConfigError);
}

}  // namespace
}  // namespace simport::plugins
