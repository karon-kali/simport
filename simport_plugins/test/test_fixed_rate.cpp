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

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "simport_plugins/fixed_rate.hpp"

using namespace std::chrono_literals;

namespace simport::plugins
{
namespace
{

struct Harness
{
  Context ctx{"dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>()};
  FixedRate trigger;
  std::atomic<int> emitted{0};

  explicit Harness(const std::string & yaml)
  {
    trigger.configure(Config::from_string(yaml, "t.yaml"), ctx);
    trigger.start([this](const RecordPtr &) {++emitted;});
  }
};

TEST(FixedRate, EmitsAtConfiguredRateIndependentOfInput)
{
  Harness h("rate_hz: 200\nstale_timeout: 0\n");
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(h.emitted, 0);  // nothing before the first record
  h.trigger.on_record(std::make_shared<const Record>());  // a single sample
  std::this_thread::sleep_for(500ms);
  h.trigger.stop();
  // ~100 emits in 0.5 s from one input sample; generous bounds for loaded CI runners.
  EXPECT_GE(h.emitted, 70);
  EXPECT_LE(h.emitted, 110);
  const int after_stop = h.emitted;
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(h.emitted, after_stop);
}

TEST(FixedRate, PausesWhenDataGoesStaleAndResumes)
{
  Harness h("rate_hz: 100\nstale_timeout: 0.1\n");
  h.trigger.on_record(std::make_shared<const Record>());
  std::this_thread::sleep_for(300ms);
  const int during = h.emitted;
  EXPECT_GE(during, 4);    // ~10 emits before 0.1 s staleness
  EXPECT_LE(during, 16);
  EXPECT_GT(h.trigger.stats().stale, 10u);
  h.trigger.on_record(std::make_shared<const Record>());
  std::this_thread::sleep_for(50ms);
  EXPECT_GT(h.emitted, during);
  h.trigger.stop();
}

TEST(FixedRate, ConfigErrors)
{
  Context ctx("dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>());
  FixedRate t;
  EXPECT_THROW(t.configure(Config::from_string("plugin: x\n"), ctx), ConfigError);
  EXPECT_THROW(t.configure(Config::from_string("rate_hz: 0\n"), ctx), ConfigError);
  EXPECT_THROW(t.configure(Config::from_string("rate_hz: 10\nstale_timeout: -1\n"), ctx),
    ConfigError);
  EXPECT_THROW(t.configure(Config::from_string("rate_hz: 10\nrate: 5\n"), ctx), ConfigError);
}

}  // namespace
}  // namespace simport::plugins
